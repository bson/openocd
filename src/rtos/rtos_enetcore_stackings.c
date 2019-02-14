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

#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"

/*
 *  0x40   (total size)
 *
 *  0x3C   PSR
 *  0x38   PC
 *  0x34   LR
 *  0x30   R12
 *  0x2C   R3
 *  0x28   R2
 *  0x24   R1
 *  0x20   R0
 *
 * The ones above are stacked by the PendSV exception,
 *
 * The ones below are stacked to save state for the context switch.
 *
 *  0x1C   R11
 *  0x18   R10
 *  0x14   R9
 *  0x10   R8
 *  0x0C   R7
 *  0x08   R6
 *  0x04   R5
 *  0x00   R4
 *
 * The thread PSP is stored in the thread PCB and points to R4 on the
 * stack.
 */
static const struct stack_register_offset 
rtos_enetcore_Cortex_M3_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
    { 0x20, 32 },       /* r0   */
    { 0x24, 32 },       /* r1   */
    { 0x28, 32 },       /* r2   */
    { 0x2c, 32 },       /* r3   */
    { 0x00, 32 },       /* r4   */
    { 0x04, 32 },       /* r5   */
    { 0x08, 32 },       /* r6   */
    { 0x0c, 32 },       /* r7   */
    { 0x10, 32 },       /* r8   */
    { 0x14, 32 },       /* r9   */
    { 0x18, 32 },       /* r10  */
    { 0x1c, 32 },       /* r11  */
    { 0x30, 32 },       /* r12  */
    { -2,   32 },       /* sp   */
    { 0x34, 32 },       /* lr   */
    { 0x38, 32 },       /* pc   */
    { 0x3c, 32 },       /* xPSR */
};

const struct rtos_register_stacking rtos_enetcore_Cortex_M3_stacking = {
    0x40,                   /* stack_registers_size */
    -1,                     /* stack_growth_direction */
    ARMV7M_NUM_CORE_REGS,   /* num_output_registers */
    rtos_generic_stack_align8,  /* stack_alignment */
    rtos_enetcore_Cortex_M3_stack_offsets   /* register_offsets */
};