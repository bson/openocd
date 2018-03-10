/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/target_type.h"
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_enetcore_stackings.h"

typedef unsigned int uint;

static bool enetcore_detect_rtos(struct target *target);
static int enetcore_create(struct target *target);
static int enetcore_update_threads(struct rtos *rtos);
static int enetcore_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, char **hex_reg_list);
static int enetcore_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[]);

struct enetcore_thread_state {
	int value;
	const char *desc;
};

static const struct enetcore_thread_state enetcore_thread_states[] = {
	{ 0, "Running" },
	{ 1, "Wait" },
	{ 2, "TimeWait" },
	{ 3, "Sleep" },
	{ 4, "Stopped" },
};

#define ENETCORE_NUM_STATES \
    (sizeof enetcore_thread_states/sizeof enetcore_thread_states[0])

struct enetcore_params {
	const char *target_name;
	unsigned char pointer_width;
	unsigned char thread_stack_offset;
	unsigned char thread_name_offset;
	unsigned char thread_state_offset;
	const struct rtos_register_stacking *stacking_info;
};

static const struct enetcore_params enetcore_params_list[] = {
	{
	"cortex_m",                   /* target_name */
	4,                            /* pointer_width; */
	0,                            /* thread_stack_offset; */
	4,                            /* thread_name_offset; */
	8,                            /* thread_state_offset; */
	&rtos_enetcore_Cortex_M3_stacking	/* stacking_info */
	}
};

#define ENETCORE_NUM_PARAMS \
    ((int)(sizeof enetcore_params_list/sizeof enetcore_params_list[0]))

enum enetcore_symbol_values {
	enetcore_VAL_runq = 0,
	enetcore_VAL_current_thread_ptr = 1
};

static const char * const enetcore_symbol_list[] = {
	"Thread::_runq",
	"Thread::_curthread",
	NULL
};

const struct rtos_type enetcore_rtos = {
	.name = "enetcore",

	.detect_rtos = enetcore_detect_rtos,
	.create = enetcore_create,
	.update_threads = enetcore_update_threads,
	.get_thread_reg_list = enetcore_get_thread_reg_list,
	.get_symbol_list_to_lookup = enetcore_get_symbol_list_to_lookup,

};

static int enetcore_update_threads(struct rtos *rtos)
{
	int retval;
	const struct enetcore_params *param;

	if (rtos == NULL)
		return -1;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct enetcore_params *) rtos->rtos_specific_params;

	if (rtos->symbols == NULL) {
		LOG_ERROR("No symbols for enetcore");
		return -4;
	}

	if (rtos->symbols[enetcore_VAL_runq].address == 0) {
		LOG_ERROR("Don't have the thread list head");
		return -2;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* determine the number of current threads */
	const uint32_t thread_q = rtos->symbols[enetcore_VAL_runq].address;
    uint32_t thread_list;       /* Pointer to array */
	uint32_t thread_num;        /* Number of items in array */

	target_read_buffer(rtos->target,
                       thread_q,
                       param->pointer_width,
                       (uint8_t *) &thread_list);

	target_read_buffer(rtos->target,
                       thread_q + 8,
                       param->pointer_width,
                       (uint8_t *) &thread_num);
    
	/* read the current thread id */
	uint32_t current_thread;
	retval = target_read_buffer(rtos->target,
                                rtos->symbols[enetcore_VAL_current_thread_ptr].address,
                                param->pointer_width,
                                (uint8_t *)&current_thread);
    
	if (retval != ERROR_OK)
		return retval;

	if (thread_num == 0 || current_thread == 0) {
		/* no threads - not bootstrapped yet */
		char tmp_str[] = "init";

		rtos->thread_details = malloc(sizeof(struct thread_detail));
		rtos->thread_details->threadid = 1;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = malloc(sizeof(tmp_str));
		strcpy(rtos->thread_details->thread_name_str, tmp_str);

        rtos->thread_count = 1;
        rtos->current_thread = 1;

        return ERROR_OK;
	}

    /* create space for new thread details */
    rtos->thread_details = malloc(sizeof(struct thread_detail) * thread_num);
    rtos->current_thread = 0;

	/* loop over all threads */
    for (uint nthread = 0; nthread < thread_num; ++nthread) {

		enum { THREAD_NAME_STR_SIZE = 32 };

		char tmp_str[THREAD_NAME_STR_SIZE];

        /* Get thread base */
        uint32_t thread_base;
        retval = target_read_buffer(rtos->target,
                                    thread_list + nthread * param->pointer_width,
                                    param->pointer_width,
                                    (uint8_t*)&thread_base);

		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read enetcore thread base from target");
			return retval;
		}

		rtos->thread_details[nthread].threadid = nthread + 1;
        if (thread_base == current_thread)
            rtos->current_thread = nthread + 1;

		/* read the name pointer */
		uint32_t name_ptr = 0;
		retval = target_read_buffer(rtos->target,
                                    thread_base + param->thread_name_offset,
                                    param->pointer_width,
                                    (uint8_t *)&name_ptr);
        
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read enetcore thread name pointer from target");
			return retval;
		}

		/* Read the thread name */
        if (name_ptr) {
            retval = target_read_buffer(rtos->target,
                                        name_ptr,
                                        THREAD_NAME_STR_SIZE-1,
                                        (uint8_t *)&tmp_str);

            if (retval != ERROR_OK) {
                LOG_ERROR("Error reading enetcore thread name from target");
                return retval;
            }

            tmp_str[THREAD_NAME_STR_SIZE-1] = 0;
        } else {
			strcpy(tmp_str, "(null)");
        }

		rtos->thread_details[nthread].thread_name_str = malloc(strlen(tmp_str)+1);
		strcpy(rtos->thread_details[nthread].thread_name_str, tmp_str);

		/* Read the thread status */
		uint8_t thread_status = 0;
		retval = target_read_buffer(rtos->target,
                                    thread_base + param->thread_state_offset,
                                    1,
                                    (uint8_t *)&thread_status);
        
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading thread state from enetcore target");
			return retval;
		}

        const char *state_desc = "(corrupt)";
        if (thread_status < ENETCORE_NUM_STATES) {
			state_desc = enetcore_thread_states[thread_status].desc;
        }

		rtos->thread_details[nthread].extra_info_str = malloc(strlen(state_desc)+8);
		sprintf(rtos->thread_details[nthread].extra_info_str, "State: %s", state_desc);

		rtos->thread_details[nthread].exists = true;
    }

	rtos->thread_count = thread_num;

	return 0;
}

static int enetcore_get_thread_reg_list(struct rtos *rtos, int64_t thread_id, char **hex_reg_list)
{
	int retval;
	const struct enetcore_params *param;

	*hex_reg_list = NULL;

	if (rtos == NULL)
		return -1;

	if (thread_id == 0)
		return -2;

	if (rtos->rtos_specific_params == NULL)
		return -3;

	param = (const struct enetcore_params*)rtos->rtos_specific_params;

    uint32_t thread_q = rtos->symbols[enetcore_VAL_runq].address;

    /* Fetch thread list */
    uint32_t thread_list;
    retval = target_read_buffer(rtos->target,
                                thread_q,
                                param->pointer_width,
                                (uint8_t*)&thread_list);
    
    if (retval != ERROR_OK) {
        LOG_ERROR("Error reading enetcore thread queue head");
        return retval;
    }

    /* Fetch thread data from list */
    uint32_t thread_base;
    retval = target_read_buffer(rtos->target,
                                thread_list + thread_id * param->pointer_width,
                                param->pointer_width,
                                (uint8_t*)&thread_base);

    if (retval != ERROR_OK) {
        LOG_ERROR("Error reading enetcore thread address");
        return retval;
    }

    /* Fetch stack pointer from data*/
    uint32_t stack_ptr = 0;
    retval = target_read_buffer(rtos->target,
                                thread_base,
                                param->pointer_width,
                                (uint8_t *)&stack_ptr);

    if (retval != ERROR_OK) {
        LOG_ERROR("Error reading stack frame from enetcore thread");
        return retval;
    }

    return rtos_generic_stack_read(rtos->target,
                                   param->stacking_info,
                                   stack_ptr,
                                   hex_reg_list);
}

static int enetcore_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(enetcore_symbol_list), sizeof(symbol_table_elem_t));

	for (i = 0; i < ARRAY_SIZE(enetcore_symbol_list); i++)
		(*symbol_list)[i].symbol_name = enetcore_symbol_list[i];

	return 0;
}

static bool enetcore_detect_rtos(struct target *target)
{
	return (target->rtos->symbols != NULL &&
            target->rtos->symbols[enetcore_VAL_runq].address != 0);
}

static int enetcore_create(struct target *target)
{
	int i = 0;
	while ((i < ENETCORE_NUM_PARAMS) &&
		(0 != strcmp(enetcore_params_list[i].target_name, target->type->name))) {
		i++;
	}
	if (i >= ENETCORE_NUM_PARAMS) {
		LOG_ERROR("Could not find target in enetcore compatibility list");
		return -1;
	}

	target->rtos->rtos_specific_params = (void *) &enetcore_params_list[i];
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
	return 0;
}
