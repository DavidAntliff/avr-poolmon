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
 * Function
 * ========
 *
 * The ATtiny84A controls two SSRs (solid-state relays) connected to hydro pumps:
 *   - Circulation pump (CP) - a smaller pump used to cycle water through the system
 *   - Purge pump (PP) - a larger pump used intermittently to force air out of the system
 *
 * In addition, a piezoelectric buzzer is also controlled.
 *
 * Communication between the ATtiny84 and the main controller (ESP32) is via the I2C bus.
 * The ATtiny84A implements an I2C slave, and responds to commands sent by the I2C master (ESP32).
 *
 * Due to limitations of the I2C Slave library, the results of a write must be queried by
 * a separate read transaction. It is not possible to combine a write and a read in a single
 * transaction (e.g. SMBus Read/Write Byte protocol) with this library. Use SMBus Send/Receive Byte
 * protocols instead.
 *
 * Four switches are used to provide a user-accessible interface to the system:
 *
 *   - CP Mode (auto/man)
 *   - CP Man (on/off)
 *   - PP Mode (auto/man)
 *   - PP Man (on/off)
 *
 * Therefore each SSR has two corresponding switches. When the Mode switch for an SSR is in the AUTO
 * position, the ESP32 can request a state change (i.e. turn the SSR on or off) and the ATtiny84A
 * will actuate the change. However if the Mode switch is in the MAN (manual) position, the ATtiny84A
 * will ignore ESP32 requests and use the position of the MAN switch to control the SSR. Therefore the
 * MAN switch provides a manual override of the ESP32 when required.
 *
 * Additionally, a piezoelectric buzzer can be turned on or off.
 *
 * I2C Registers
 * =============
 *
 * See registers.h for detailed description of each supported register.
 *
 * To set the value of a register:
 *   - Send two bytes (register address, register value)
 *
 * To read the value of a register:
 *   - Send one byte (register address),
 *   - Receive read 1 byte
 *
 *
 * ATtiny84 Configuration
 * ======================
 *
 * Clock output on PORTB2 must be disabled (clear Low Fuse bit 6).
 */


#ifndef F_CPU
#  error "F_CPU undefined"
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>                // for _delay_ms()
#include <stdbool.h>

#include "registers.h"
#include "usitwislave/usitwislave.h"

#define I2C_ADDRESS 0x44

// Number of idle cycles between processing switches/outputs
#define PERIOD_CYCLES 4000

static uint8_t register_addr = 255;
static uint8_t registers[AVR_NUM_REGISTERS] = { 0 };

// SSR "actual" states
static uint8_t ssr1 = 0;
static uint8_t ssr2 = 0;

typedef struct
{
    volatile bool enabled;
    volatile uint8_t on_count;    // number of interrupt cycles to hold buzzer on for
    volatile uint8_t reset_count; // number of interrupt cycles at which counter is reset
    volatile uint8_t counter;
} buzzer_state_t;

// I/O
typedef struct
{
    volatile uint8_t * ddr;
    volatile uint8_t * port;
    volatile uint8_t * pin;
} port_registers_t;

typedef struct
{
    const port_registers_t * reg;
    uint8_t pin;
} io_t;

const port_registers_t PORTA_REGS = { &DDRA, &PORTA, &PINA };
const port_registers_t PORTB_REGS = { &DDRB, &PORTB, &PINB };

const io_t sw1_io = { &PORTA_REGS, PA0 };
const io_t sw2_io = { &PORTA_REGS, PA1 };
const io_t sw3_io = { &PORTA_REGS, PA2 };
const io_t sw4_io = { &PORTA_REGS, PA3 };

const io_t led_io = { &PORTA_REGS, PA5 };

const io_t buzzer_io = { &PORTA_REGS, PA7 };

const io_t ssr1_io = { &PORTB_REGS, PB0 };
const io_t ssr2_io = { &PORTB_REGS, PB2 };

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer);

static void idle_callback(void);


static volatile buzzer_state_t buzzer_state = { 0 };

ISR(TIM1_COMPA_vect)
{
    if (buzzer_state.enabled)
    {
        if (buzzer_state.counter == 0)
        {
            *(buzzer_io.reg->port) |= (1 << buzzer_io.pin);  // buzzer on
        }
        else if (buzzer_state.counter == buzzer_state.on_count)
        {
            *(buzzer_io.reg->port) &= ~(1 << buzzer_io.pin);  // buzzer off
        }

        ++buzzer_state.counter;

        if (buzzer_state.counter == buzzer_state.reset_count)
        {
            buzzer_state.counter = 0;
        }
    }
}

static void blink_led(const io_t * io, int n)
{
    for (int i = 0; i < n; ++i)
    {
        *(io->reg->pin) |= (1 << io->pin);
        _delay_ms(100);
        *(io->reg->pin) |= (1 << io->pin);
        _delay_ms(100);
    }
}

static void data_callback(uint8_t input_buffer_length, const uint8_t *input_buffer,
                          uint8_t *output_buffer_length, uint8_t *output_buffer)
{
    *output_buffer_length = 1;
    output_buffer[0] = 0;

    // support SMBus Send Byte (1 or 2 incoming data bytes) or Receive Byte (0 incoming data byte)
    switch (input_buffer_length)
    {
        case 0:  // read currently addressed register
            //blink_led(&debug_led_io, 1);
            if (register_addr < AVR_NUM_REGISTERS)
            {
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];
            }
            break;

        case 1:  // write before read - set ADDR register only
            //blink_led(&debug_led_io, 2);
            register_addr = input_buffer[0];
            if (register_addr < AVR_NUM_REGISTERS)
            {
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];

                // clear on read registers
                if (register_addr == AVR_REGISTER_COUNT_CP ||
                    register_addr == AVR_REGISTER_COUNT_PP ||
                    register_addr == AVR_REGISTER_COUNT_BUZZER ||
                    register_addr == AVR_REGISTER_COUNT_CP_MODE ||
                    register_addr == AVR_REGISTER_COUNT_CP_MAN ||
                    register_addr == AVR_REGISTER_COUNT_PP_MODE ||
                    register_addr == AVR_REGISTER_COUNT_PP_MAN)
                {
                    registers[register_addr] = 0;
                }
            }
            break;

        case 2:  // write - set command register and addressed register
            //blink_led(&debug_led_io, 3);
            register_addr = input_buffer[0];
            if (register_addr < AVR_NUM_REGISTERS)
            {
                // only write to writable registers:
                if (register_addr == AVR_REGISTER_CONTROL ||
                    register_addr == AVR_REGISTER_SCRATCH)
                {
                    registers[register_addr] = input_buffer[1];
                }
                // as a convenience, allow for immediate read-back of selected register
                *output_buffer_length = 1;
                output_buffer[0] = registers[register_addr];
            }
            break;

        default:
            // ignore
            blink_led(&led_io, 4);
            break;
    }
}

static void read_switches(void)
{
    // no need to debounce due to long polling period

    // read switch states from first 4 bits of PINA
    uint8_t switches = (PINA & 0x0f) << 4;

    // count changes
    if ((registers[AVR_REGISTER_STATUS] ^ switches) & AVR_REGISTER_STATUS_CP_MODE)
    {
        ++registers[AVR_REGISTER_COUNT_CP_MODE];  // may overflow - no check performed
    }
    if ((registers[AVR_REGISTER_STATUS] ^ ~switches) & AVR_REGISTER_STATUS_CP_MAN)
    {
        ++registers[AVR_REGISTER_COUNT_CP_MAN];  // may overflow - no check performed
    }
    if ((registers[AVR_REGISTER_STATUS] ^ switches) & AVR_REGISTER_STATUS_PP_MODE)
    {
        ++registers[AVR_REGISTER_COUNT_PP_MODE];  // may overflow - no check performed
    }
    if ((registers[AVR_REGISTER_STATUS] ^ ~switches) & AVR_REGISTER_STATUS_PP_MAN)
    {
        ++registers[AVR_REGISTER_COUNT_PP_MAN];  // may overflow - no check performed
    }

    // manual switches are inverted
    registers[AVR_REGISTER_STATUS] = switches;
    registers[AVR_REGISTER_STATUS] ^= AVR_REGISTER_STATUS_CP_MAN;
    registers[AVR_REGISTER_STATUS] ^= AVR_REGISTER_STATUS_PP_MAN;
}

static uint8_t calculate_ssr_state(uint8_t mode, uint8_t control, uint8_t man)
{
    uint8_t ssr = 0;
    // Calculate SSR actual states
    if ((registers[AVR_REGISTER_STATUS] & mode) == AVR_REGISTER_STATUS_MODE_AUTO)
    {
        // use the requested control value
        ssr = (registers[AVR_REGISTER_CONTROL] & control) ? 1 : 0;
    }
    else
    {
        // use the manual switch value
        ssr = (registers[AVR_REGISTER_STATUS] & man) ? 1 : 0;
    }
    return ssr;
}

static void update_ssr_count(uint8_t ssr, const io_t * io, avr_register_t count_register)
{
    uint8_t current = *(io->reg->pin) & (1 << io->pin) ? 1 : 0;
    if (ssr == current)  // inverted
    {
        ++registers[count_register];
    }
}

static void update_ssr_output(uint8_t ssr, const io_t * io)
{
    if (ssr)
    {
        // drive low to turn SSR on
        *(io->reg->port) &= ~(1 << io->pin);
    }
    else
    {
        // drive high to turn SSR off
        *(io->reg->port) |= (1 << io->pin);
    }
}

static void update_buzzer_state(volatile buzzer_state_t * state)
{
    if (registers[AVR_REGISTER_CONTROL] & AVR_REGISTER_CONTROL_BUZZER)
    {
        // buzzer is enabled
        uint8_t mode = ((registers[AVR_REGISTER_CONTROL] & AVR_REGISTER_CONTROL_BUZZER_MODE_0) ? 0b01 : 0b00) |
                       ((registers[AVR_REGISTER_CONTROL] & AVR_REGISTER_CONTROL_BUZZER_MODE_1) ? 0b10 : 0b00);
        switch (mode)
        {
            case 0b00:  // continuous
                state->on_count = 16;
                state->reset_count = 16;
                break;
            case 0b01:  // slow pulse
                state->on_count = 24;
                state->reset_count = 32;
                break;
            case 0b10:  // fast pulse
                state->on_count = 12;
                state->reset_count = 16;
                break;
            case 0b11:  // blip
                state->on_count = 2;
                state->reset_count = 24;
                break;
            default:
                break;
        }

        if (!state->enabled)
        {
            ++registers[AVR_REGISTER_COUNT_BUZZER];
            state->enabled = true;
        }
    }
    else
    {
        if (state->enabled)
        {
            ++registers[AVR_REGISTER_COUNT_BUZZER];
            state->enabled = false;
            state->counter = 0;

            // ensure buzzer is off
            *(buzzer_io.reg->port) &= ~(1 << buzzer_io.pin);
        }
    }
}

static void update_status_register(uint8_t ssr, uint8_t status)
{
    if (ssr)
    {
        registers[AVR_REGISTER_STATUS] |= status;
    }
    else
    {
        registers[AVR_REGISTER_STATUS] &= ~status;
    }
}

static void idle_callback(void)
{
    static uint32_t count = 0;
    if (++count >= PERIOD_CYCLES)
    {
        // toggle LED
        *(led_io.reg->pin) |= (1 << led_io.pin);
        count = 0;

        read_switches();

        ssr1 = calculate_ssr_state(AVR_REGISTER_STATUS_CP_MODE, AVR_REGISTER_CONTROL_SSR1, AVR_REGISTER_STATUS_CP_MAN);
        ssr2 = calculate_ssr_state(AVR_REGISTER_STATUS_PP_MODE, AVR_REGISTER_CONTROL_SSR2, AVR_REGISTER_STATUS_PP_MAN);
        update_buzzer_state(&buzzer_state);

        update_ssr_count(ssr1, &ssr1_io, AVR_REGISTER_COUNT_CP);
        update_ssr_count(ssr2, &ssr2_io, AVR_REGISTER_COUNT_PP);

        update_ssr_output(ssr1, &ssr1_io);
        update_ssr_output(ssr2, &ssr2_io);

        update_status_register(ssr1, AVR_REGISTER_STATUS_CP);
        update_status_register(ssr2, AVR_REGISTER_STATUS_PP);
    }
}

static void init_as_input(const io_t * io, bool enable_pullup)
{
    *(io->reg->ddr) &= ~(1 << io->pin);
    if (enable_pullup)
    {
        *(io->reg->port) |= (1 << io->pin);
    }
    else
    {
        *(io->reg->port) &= ~(1 << io->pin);
    }
}

static void init_as_output(const io_t * io, bool default_value)
{
    *(io->reg->ddr) |= (1 << io->pin);
    if (default_value)
    {
        *(io->reg->port) |= (1 << io->pin);
    }
    else
    {
        *(io->reg->port) &= ~(1 << io->pin);
    }
}

static void init_timer_interrupt(void)
{
    TCCR1B |= (1 << WGM12);                 // configure timer 1 for CTC mode, Fcpu / 64
    TCCR1B |= ((1 << CS10) | (1 << CS11));  // start timer at Fcpu / 64
    TIMSK1 |= (1 << OCIE1A);                // enable CTC interrupt
    OCR1A = F_CPU / 64UL / 16UL - 1UL;       // set CTC compare value to 10Hz at 1MHz AVR clock, with a prescaler of 64
    sei();                                  // enable global interrupts
}

int main(void)
{
    // initialize switches as inputs with pull-ups
    init_as_input(&sw1_io, true);
    init_as_input(&sw2_io, true);
    init_as_input(&sw3_io, true);
    init_as_input(&sw4_io, true);

    // init SSR and buzzer as outputs with default values
    init_as_output(&ssr1_io, 1);
    init_as_output(&ssr2_io, 1);
    init_as_output(&buzzer_io, 0);
    init_as_output(&led_io, 1);

    // set the ID register to the I2C address
    registers[AVR_REGISTER_ID] = I2C_ADDRESS;
    registers[AVR_REGISTER_VERSION] = AVR_VERSION;

    // buzzer testing
    //registers[AVR_REGISTER_CONTROL] = AVR_REGISTER_CONTROL_BUZZER;  // continuous
    //registers[AVR_REGISTER_CONTROL] = AVR_REGISTER_CONTROL_BUZZER | AVR_REGISTER_CONTROL_BUZZER_MODE_0;  // slow pulsed
    //registers[AVR_REGISTER_CONTROL] = AVR_REGISTER_CONTROL_BUZZER | AVR_REGISTER_CONTROL_BUZZER_MODE_1;  // fast pulsed
    //registers[AVR_REGISTER_CONTROL] = AVR_REGISTER_CONTROL_BUZZER | AVR_REGISTER_CONTROL_BUZZER_MODE_0 | AVR_REGISTER_CONTROL_BUZZER_MODE_1;    // blip

    // initialise timer interrupt
    init_timer_interrupt();

    // start the slave loop
    usi_twi_slave(I2C_ADDRESS, false /*use_sleep*/, data_callback, idle_callback);
}

