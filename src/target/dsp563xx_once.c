/***************************************************************************
 *   Copyright (C) 2009 by Mathias Kuester                                 *
 *   mkdorg@users.sourceforge.net                                          *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jim.h>

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "dsp563xx.h"
#include "dsp563xx_once.h"

#define JTAG_STATUS_NORMAL		0x01
#define JTAG_STATUS_STOPWAIT		0x05
#define JTAG_STATUS_BUSY		0x09
#define JTAG_STATUS_DEBUG		0x0d

#define JTAG_INSTR_EXTEST		0x00
#define JTAG_INSTR_SAMPLE_PRELOAD	0x01
#define JTAG_INSTR_IDCODE		0x02
#define JTAG_INSTR_CLAMP		0x03
#define JTAG_INSTR_HIZ			0x04
#define JTAG_INSTR_ENABLE_ONCE		0x06
#define JTAG_INSTR_DEBUG_REQUEST	0x07
#define JTAG_INSTR_BYPASS		0x0F

static int dsp563xx_write_dr(struct jtag_tap *tap, uint8_t * dr_in, uint8_t * dr_out, int dr_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}

	jtag_add_plain_dr_scan(dr_len, dr_out, dr_in, TAP_IDLE);

	return ERROR_OK;
}

static int dsp563xx_write_dr_u8(struct jtag_tap *tap, uint8_t * dr_in, uint8_t dr_out, int dr_len, int rti)
{
	if (dr_len > 8)
	{
		LOG_ERROR("dr_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	return dsp563xx_write_dr(tap, dr_in, &dr_out, dr_len, rti);
}

static int dsp563xx_write_dr_u32(struct jtag_tap *tap, uint32_t * dr_in, uint32_t dr_out, int dr_len, int rti)
{
	if (dr_len > 32)
	{
		LOG_ERROR("dr_len overflow, maxium is 32");
		return ERROR_FAIL;
	}

	return dsp563xx_write_dr(tap, (uint8_t *) dr_in, (uint8_t *) & dr_out, dr_len, rti);
}

/** single word instruction */
static int dsp563xx_once_ir_exec(struct jtag_tap *tap, uint8_t instr, uint8_t rw, uint8_t go, uint8_t ex)
{
	int err;

	if ((err = dsp563xx_write_dr_u8(tap, 0, instr | (ex << 5) | (go << 6) | (rw << 7), 8, 0)) != ERROR_OK)
		return err;

	return jtag_execute_queue();
}

/** single word instruction */
static int dsp563xx_once_ir_exec_nq(struct jtag_tap *tap, uint8_t instr, uint8_t rw, uint8_t go, uint8_t ex)
{
	return dsp563xx_write_dr_u8(tap, 0, instr | (ex << 5) | (go << 6) | (rw << 7), 8, 0);
}

/* IR and DR functions */
static int dsp563xx_write_ir(struct jtag_tap *tap, uint8_t * ir_in, uint8_t * ir_out, int ir_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}
	if (ir_len != tap->ir_length)
	{
		LOG_ERROR("invalid ir_len");
		return ERROR_FAIL;
	}

	jtag_add_plain_ir_scan(tap->ir_length, ir_out, ir_in, TAP_IDLE);

	return ERROR_OK;
}

static int dsp563xx_write_ir_u8(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out, int ir_len, int rti)
{
	if (ir_len > 8)
	{
		LOG_ERROR("ir_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	return dsp563xx_write_ir(tap, ir_in, &ir_out, ir_len, rti);
}

static int dsp563xx_jtag_sendinstr(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out)
{
	return dsp563xx_write_ir_u8(tap, ir_in, ir_out, tap->ir_length, 1);
}

/** */
int dsp563xx_once_target_status(struct jtag_tap *tap)
{
	int err;
	uint8_t jtag_status;

	if ((err = dsp563xx_jtag_sendinstr(tap, &jtag_status, JTAG_INSTR_ENABLE_ONCE)) != ERROR_OK)
		return err;
	if ((err = jtag_execute_queue()) != ERROR_OK)
		return err;

	if ((jtag_status & 1) != 1)
	{
		return TARGET_UNKNOWN;
	}

	if (jtag_status != JTAG_STATUS_DEBUG)
	{
		return TARGET_RUNNING;
	}

	return TARGET_HALTED;
}

/** */
int dsp563xx_once_request_debug(struct jtag_tap *tap, int reset_state)
{
	int err;
	uint8_t ir_in = 0, pattern = 0;
	uint32_t retry = 0;

	/* in reset state we only get a ACK
	 * from the interface */
	if (reset_state)
	{
		pattern = 1;
	}
	else
	{
		pattern = JTAG_STATUS_DEBUG;
	}

	/* wait until we get the ack */
	while (ir_in != pattern)
	{
		if ((err = dsp563xx_jtag_sendinstr(tap, &ir_in, JTAG_INSTR_DEBUG_REQUEST)) != ERROR_OK)
			return err;
		if ((err = jtag_execute_queue()) != ERROR_OK)
			return err;

		LOG_DEBUG("debug request: %02X", ir_in);

		if (retry++ == 100)
		{
			return ERROR_TARGET_FAILURE;
		}
	}

	/* we cant enable the once in reset state */
	if (pattern == 1)
	{
		return ERROR_OK;
	}

	/* try to enable once */
	retry = 0;
	ir_in = 0;
	while (ir_in != pattern)
	{
		if ((err = dsp563xx_jtag_sendinstr(tap, &ir_in, JTAG_INSTR_ENABLE_ONCE)) != ERROR_OK)
			return err;
		if ((err = jtag_execute_queue()) != ERROR_OK)
			return err;

		LOG_DEBUG("enable once: %02X", ir_in);

		if (retry++ == 100)
		{
			LOG_DEBUG("error");
			return ERROR_TARGET_FAILURE;
		}
	}

	if (ir_in != JTAG_STATUS_DEBUG)
	{
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

/** once read registers */
int dsp563xx_once_read_register(struct jtag_tap *tap, struct once_reg *regs, int len)
{
	int i;
	int err;

	for (i = 0; i < len; i++)
	{
		if ((err = dsp563xx_once_reg_read_ex_nq(tap, regs[i].addr, regs[i].len, &regs[i].reg)) != ERROR_OK)
			return err;
	}

	return jtag_execute_queue();
/*
	for(i=0;i<len;i++)
	{
		printf("%08X\n",regs[i].reg);
	}
*/
}

/** once read register */
int dsp563xx_once_reg_read_ex_nq(struct jtag_tap *tap, uint8_t reg, uint8_t len, uint32_t * data)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, reg, 1, 0, 0)) != ERROR_OK)
		return err;
	return dsp563xx_write_dr_u32(tap, data, 0x00, len, 0);
}

/** once read register */
int dsp563xx_once_reg_read_ex(struct jtag_tap *tap, uint8_t reg, uint8_t len, uint32_t * data)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, reg, 1, 0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, data, 0x00, len, 0)) != ERROR_OK)
		return err;
	return jtag_execute_queue();
}

/** once read register */
int dsp563xx_once_reg_read(struct jtag_tap *tap, uint8_t reg, uint32_t * data)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, reg, 1, 0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, data, 0x00, 24, 0)) != ERROR_OK)
		return err;
	return jtag_execute_queue();
}

/** once write register */
int dsp563xx_once_reg_write(struct jtag_tap *tap, uint8_t reg, uint32_t data)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, reg, 0, 0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0x00, data, 24, 0)) != ERROR_OK)
		return err;
	return jtag_execute_queue();
}

/** single word instruction */
int dsp563xx_once_execute_sw_ir(struct jtag_tap *tap, uint32_t opcode)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0)) != ERROR_OK)
		return err;
	return jtag_execute_queue();
}

/** double word instruction */
int dsp563xx_once_execute_dw_ir(struct jtag_tap *tap, uint32_t opcode, uint32_t operand)
{
	int err;

	if ((err = dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0)) != ERROR_OK)
		return err;
	if ((err = jtag_execute_queue()) != ERROR_OK)
		return err;

	if ((err = dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, operand, 24, 0)) != ERROR_OK)
		return err;
	if ((err = jtag_execute_queue()) != ERROR_OK)
		return err;

	return ERROR_OK;
}

/** single word instruction */
int dsp563xx_once_execute_sw_ir_nq(struct jtag_tap *tap, uint32_t opcode)
{
	int err;

	if ((err = dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0)) != ERROR_OK)
		return err;

	return ERROR_OK;
}

/** double word instruction */
int dsp563xx_once_execute_dw_ir_nq(struct jtag_tap *tap, uint32_t opcode, uint32_t operand)
{
	int err;

	if ((err = dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_write_dr_u32(tap, 0, operand, 24, 0)) != ERROR_OK)
		return err;

	return ERROR_OK;
}
