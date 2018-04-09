/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * I2C Registers
 * =============
 *
 * VERSION 1:
 *
 * In general, the internal ADDR register is written first to specify the control/status
 * register for following read/write operations.
 *
 *   ID register: read-only register containing an expected value for the master to verify,
 *                perhaps equal to the I2C address of the device.
 *
 *   VERSION register: read-only register containing the version of this software, as an unsigned 8-bit integer.
 *
 *   ADDR register: specifies the address of the target register for subsequent read and write operations.
 *                  It is a single byte value. It defaults to 0x00 at power on.
 *
 *   CONTROL register: provides I2C master control of SSR and buzzer states:
 *     bit 7: reserved
 *     bit 6: reserved
 *     bit 5: reserved
 *     bit 4: set piezoelectric buzzer state (1 = on, 0 = off)
 *     bit 3: reserved
 *     bit 2: reserved
 *     bit 1: set SSR2 state in AUTO mode (1 = on, 0 = off)
 *     bit 0: set SSR1 state in AUTO mode (1 = on, 0 = off)
 *
 *   STATUS register: provides I2C master monitoring of switch and SSR states
 *     bit 7: read sw4 (PP Man) state (1 = on, 0 = off) on PA3
 *     bit 6: read sw3 (PP Mode) state (1 = manual, 0 = auto) on PA2
 *     bit 5: read sw2 (CP Man) state (1 = on, 0 = off) on PA1
 *     bit 4: read sw1 (CP Mode) state (1 = manual, 0 = auto) on PA0
 *     bit 3: reserved
 *     bit 2: reserved
 *     bit 1: read actual SSR2 state (1 = on, 0 = off)
 *     bit 0: read actual SSR1 state (1 = on, 0 = off)
 *
 *   SCRATCH register: non-functional read/write register for master use.
 *
 *   COUNT_CP,_PP registers: clear-on-read counts of SSR state changes.
 *
 *   COUNT_CP_MODE,_CP_MAN,_PP_MODE,_PP_MAN registers: clear-on-read counts of switch state changes.
 *
 *   COUNT_BUZZER register: clear-on-read count of buzzer state changes.
 */

#ifndef REGISTERS_H
#define REGISTERS_H

#define VERSION 1

typedef enum
{
    REGISTER_ID = 0x00,            // read only
    REGISTER_VERSION,              // read only
    REGISTER_CONTROL,              // read/write
    REGISTER_STATUS,               // read only
    REGISTER_SCRATCH,              // read/write
    REGISTER_COUNT_CP,             // clear on read
    REGISTER_COUNT_PP,             // clear on read
    REGISTER_COUNT_CP_MODE,        // clear on read
    REGISTER_COUNT_CP_MAN,         // clear on read
    REGISTER_COUNT_PP_MODE,        // clear on read
    REGISTER_COUNT_PP_MAN,         // clear on read
    REGISTER_COUNT_BUZZER,         // clear on read
    NUM_REGISTERS
} registers_t;

// Control Register
#define REGISTER_CONTROL_SSR1   (1 << 0)
#define REGISTER_CONTROL_SSR2   (1 << 1)
#define REGISTER_CONTROL_BUZZER (1 << 4)

#define REGISTER_CONTROL_ON     1
#define REGISTER_CONTROL_OFF    0

// Status Register
#define REGISTER_STATUS_SSR1    (1 << 0)    // Circulation Pump
#define REGISTER_STATUS_SSR2    (1 << 1)    // Purge Pump
#define REGISTER_STATUS_SW1     (1 << 4)    // CP Mode
#define REGISTER_STATUS_SW2     (1 << 5)    // CP Man
#define REGISTER_STATUS_SW3     (1 << 6)    // PP Mode
#define REGISTER_STATUS_SW4     (1 << 7)    // PP Man

// Aliases
#define REGISTER_STATUS_CP      REGISTER_STATUS_SSR1
#define REGISTER_STATUS_PP      REGISTER_STATUS_SSR2
#define REGISTER_STATUS_CP_MODE REGISTER_STATUS_SW1
#define REGISTER_STATUS_CP_MAN  REGISTER_STATUS_SW2
#define REGISTER_STATUS_PP_MODE REGISTER_STATUS_SW3
#define REGISTER_STATUS_PP_MAN  REGISTER_STATUS_SW4

#define REGISTER_STATUS_MODE_AUTO   0
#define REGISTER_STATUS_MODE_MANUAL 1
#define REGISTER_STATUS_ON          1
#define REGISTER_STATUS_OFF         0

#endif // REGISTERS_H
