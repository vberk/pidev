/*
 *  Copyright (c) 2021 by Vincent H. Berk
 *  All rights reserved.
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met: 
 * 
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer. 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution. 
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _PIDEV_H
#define _PIDEV_H


#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/time.h>



/*
 *  General notes regarding the Pi ARM CPU and the Peripherals.
 *
 *  For optimization, reading and writing to different peripherals should be serialized.
 *  Put another way, issuing a read of 2 subsequent registers controlling different
 *  peripherals is not guaranteed to return in-order, causing potential confusion.
 *
 *  In practice this means:  do not call the peripheral control methods in a multi-threaded
 *  fashion or ensure exclusivity through mutexes.
 *
 *  From Chapter 9: PWM
 *   MSEN=0 is the default mode which modulates N out of M bits as evenly as possible.
 *   Where N/M is the 'duty cycle' and 'M' is number of clock cycles to send data (range).
 *
 */




//  Base addres to access registers etc:
//  The GPIO memory is laid out as 32 words with specific functions
//  Offsets into /dev/mem
#define PIDEV_GPIO_BASE_2835    0x20000000   //  Older, plus the Pi Zeroes
#define PIDEV_GPIO_BASE_2836    0x3f000000   //  Also the BCM2837
#define PIDEV_GPIO_BASE_2711    0xfe000000   //  Pi 4 -- note:  noticed one P4 used the BCM2835 base address


//  The stuff that is hardware specific:
typedef struct
{
    //  Details of the board:
    uint32_t rev;
    uint8_t brev;
    uint8_t model;
    uint8_t arch;
    uint8_t mfg;
    uint8_t ram;
    //  Is this a simulation?  Set this to 1 before calling setupHardware.
    u_int8_t simulation;
    //  Hardware configuration:
    unsigned int gpioBaseAddr;
    long pageSize;
    //  Memory mapped ranges for the control regs:
    //  This is a bit odd, as the processor specifies these are
    //  addressed in blocks of 4 bytes.
    uint32_t *gpio;
    uint32_t *clock;
    uint32_t *pwm;
    uint32_t *spi;
}
PIDEV_HW;


//  Versions, models, manufacturers, etc:
//  NOTE:  ram size is 2**ram * 256MB (ie. ram=2 -> 2**2*256MB=1G)
/*
#define PIDEV_MODEL_A               0
#define PIDEV_MODEL_B               1
#define PIDEV_MODEL_APLUS           2
#define PIDEV_MODEL_BPLUS           3
#define PIDEV_MODEL_PI2             4
#define PIDEV_MODEL_ALPHA           5
#define PIDEV_MODEL_COMP            6
#define PIDEV_MODEL_UNKNOWN7        7
#define PIDEV_MODEL_PI3             8
#define PIDEV_MODEL_ZERO            9
#define PIDEV_MODEL_COMP3          10
#define PIDEV_MODEL_UNKNOWN11      11
#define PIDEV_MODEL_ZERO_W         12
#define PIDEV_MODEL_PI3_BPLUS      13
#define PIDEV_MODEL_PI3_APLUS      14
#define PIDEV_MODEL_UNKNOWN15      15
#define PIDEV_MODEL_COMP3_PLUS     16
#define PIDEV_MODEL_PI4_B          17
#define PIDEV_MODEL_UNKNOWN18      18
#define PIDEV_MODEL_PI400          19
#define PIDEV_MODEL_COMP4          20


#define PIDEV_MFG_SONY     0
#define PIDEV_MFG_EGOMAN   1
#define PIDEV_MFG_EMBEST   2
#define PIDEV_MFG_UNKNOWN  3
#define PIDEV_MFG_EMBEST   4
*/


#define PIDEV_GPIOMEM   "/dev/gpiomem"      //  NOTE: only gives access to gpio regs, not the peripherals like the clock
#define PIDEV_ALLMEM    "/dev/mem"


#define PIDEV_ARCH_2835     0x0
#define PIDEV_ARCH_2836     0x1
#define PIDEV_ARCH_2837     0x2
#define PIDEV_ARCH_2711     0x3

//  These are the lengths of the memory regions for the control registers:
#define PIDEV_GPIO_LEN      0xF4
#define PIDEV_CLOCK_LEN     0xA8
#define PIDEV_PWM_LEN       0x28
#define PIDEV_SPI_LEN       0x18



//  Configuration of the GPIO pin functions
//  Each is controlled through a 3-bit field (8 possible settings)
//  Pin registers are 32-bit, 10 pins are controlled per 32-bit word:
//    reg[gpio/10]&((gpio%10)*3)
//  This is from chap 6 in the manual:
#define PIDEV_IN    0x0     //  Pin is an 'input'
#define PIDEV_OUT   0x1     //  Pin is an 'output'
#define PIDEV_ALT0  0x4     //  Alternate function 1  (binary: 100)
#define PIDEV_ALT1  0x5     //  101
#define PIDEV_ALT2  0x6     //  110
#define PIDEV_ALT3  0x7     //  111
#define PIDEV_ALT5  0x2     //  011
#define PIDEV_ALT4  0x3     //  010


//  Locations of the PWM control registers.
//  There's 2 PWM channels (0 and 1)
//  Notes on PWM pins:
//    ALT5 (0x2): 18, 19
//    ALT0 (0x4): 12, 13, 40, 41, 45
//    ALT1 (0x?): 52, 53
//  Channel (data reg):
//    DATA0: 12, 18, 40
//    DATA1: 13, 19, 41, 45
//  From Chapter 9:
#define PIDEV_PWM_CTRL    0       //  Control
#define PIDEV_PWM_STATUS  1       //  Status
#define PIDEV_PWM_DMAC    2       //  DMA ctrl register
#define PIDEV_PWM_RANGE0  4       //  Channel 0 range register (M), defines period of length (default val: 0x20)
#define PIDEV_PWM_DATA0   5       //  Channel 0 data register (N), only valid when mode is NOT fifo.  Defines the number of pulses sent.
#define PIDEV_PWM_FIFO    6       //  PWM FIFO input
#define PIDEV_PWM_RANGE1  8       //  Channel 1 range register
#define PIDEV_PWM_DATA1   9       //  Channel 1 settings register

//  In the PWM control register, channel 0 is on byte 0
//  while channel 1 is on byte 1 (<<8)
//  Upper 2 bytes are unused.
//  PWM clock source and frequency is controlled in CPRMAN (not in peripheral manual)
#define PIDEV_PWM_CTRL_ENABLE     0x01    //  Use channel
#define PIDEV_PWM_CTRL_SERIAL     0x02    //  Set to 0 for PWM mode (modulation) or 1 for serial mode (one long stream)
#define PIDEV_PWM_CTRL_FIFOREP    0x04    //  Repeat if FIFO empty: 0 means 'transmission interrupts when fifo empty'
#define PIDEV_PWM_CTRL_OFFHIGH    0x08    //  Hold high in 'off' state when idle.
#define PIDEV_PWM_CTRL_REVPOL     0x10    //  Reverse the polarity (0: 0=low 1=high, and 1: 1=low 0=high)
#define PIDEV_PWM_CTRL_USEFIFO    0x20    //  Use fifo?  0: data register is used, and 1: fifo register is used
#define PIDEV_PWM_CTRL_CLRF       0x40    //  Send 1 to "clear fifo", 0 has 'no effect'
#define PIDEV_PWM_CTRL_MSEN       0x80    //  0: use PWM mode N/M, 1: use serial mode M/S

//  This is the default range value that is set upon init
//  for both of the PWM modules.  The data must therefore
//  be between 0 and 1024 when set.
#define PIDEV_PWM_RANGE     1024    //  default
#define PIDEV_PWM_FREQ     10000    //  10khz


//  GPIO clock control registers
//  Clocks can be output to the GPIO pins directly (see alt-func chart)
//  There's 3 GP clocks.  Each has a control and a DIV register.
//  A special password byte is used to control them, and all must be 'not busy' to change configuration.
//  Maximum frequency is 125Mhz
//    ALT0 (0x4):  4 (CLK0), 5 (CLK1), 6 (CLK2), 32 (CLK0), 34 (CLK0), 42 (CLK1), 43 (CLK2), 44, (CLK1)
//    ALT5 (0x2):  20 (CLK0), 21 (CLK1)
#define PIDEV_CLK0_CTRL       0x1c      //  Control
#define PIDEV_CLK0_DIV        0x1d      //  Divisor
#define PIDEV_CLK1_CTRL       0x1e      //  etc.
#define PIDEV_CLK1_DIV        0x1f      //  
#define PIDEV_CLK2_CTRL       0x20      //  
#define PIDEV_CLK2_DIV        0x21      //  

//  Control register bits:
//  Must OR with the password (upper byte must be 0x5A)
//  Do not switch controls while BUSY==1
#define PIDEV_CLK_CTRL_BUSY     0x80    //  Read only -- if 'high' clock is busy
#define PIDEV_CLK_CTRL_KILL     0x20    //  Kill -- it says: "don't use, for testing/debug only, may cause glitches"
#define PIDEV_CLK_CTRL_ENABLE   0x10    //  Enable/disable -- use this, but recognize the clock must finish the cycle, 'BUSY' will go low when cycle complete
//  Bits 0-3 determine clock source, but only a few are valid:
#define PIDEV_CLK_CTRL_GND      0x00    //  Tie to ground (no signal)
#define PIDEV_CLK_CTRL_OSC      0x01    //  Use oscilator
#define PIDEV_CLK_CTRL_PLLA     0x04    //  PLLA??
#define PIDEV_CLK_CTRL_PLLC     0x05    //  
#define PIDEV_CLK_CTRL_PLLD     0x06    //  
#define PIDEV_CLK_CTRL_HDMI     0x07    //  HDMI
//  Bits 9 and 10 set the MASH noise cancellation, and can be ignored
//  This is a password mask needed for the clock registers:
#define PIDEV_CLK_PASSWD    0x5A000000 

//  The PCM and PWM clocks use the same divider register:
#define PIDEV_CLKPCM_CTRL       38
#define PIDEV_CLKPCM_DIV        39
#define PIDEV_CLKPWM_CTRL       40
#define PIDEV_CLKPWM_DIV        41

//  Defines for selecting which clock to use (initClock)
#define PIDEV_CLK0      PIDEV_CLK0_CTRL
#define PIDEV_CLK1      PIDEV_CLK1_CTRL
#define PIDEV_CLK2      PIDEV_CLK2_CTRL
#define PIDEV_CLKPCM    PIDEV_CLKPCM_CTRL
#define PIDEV_CLKPWM    PIDEV_CLKPWM_CTRL

//  The devisor register has an Integer part and a Fractional part.
//  Do not set the divisor unless the clock is set to NOT BUSY
//  The upper byte of the control register is again the CLK_PASSWORD
//  Bits 23-12 are the i-div (12 bits, max value of 4096), the integer divisor
//  Bits 11-0  are the f-div (12 bits, max value of 4096), the fractional of the divisor
//  NOTE:  I have experimented with various f-div values at both Mhz and Khz frequencies
//  (1-stage MASH must be enabled), and the oscilloscope shows the odd 'shadow'
//  of the drift of the clock signal to average to the requested frequency.
//  I prefer sticking with clock frequencies that divide the OSC_FREQ w/o
//  a remainder to get the cleanest possible clock signal.

//  Oscillator for the BCM2835-7 is 19.2Mhz, for the 2711 it is 54Mhz
#define PIDEV_BCM2835_OSC_FREQ  19200000
#define PIDEV_BCM2711_OSC_FREQ  54000000


//  54 pins total
#define PIDEV_MAXPIN 54




//  Open and close the hardware layer.
int PIDEV_setupHardware(PIDEV_HW *pi);
int PIDEV_closeHardware(PIDEV_HW *pi);

//  Debug to print a few details about the hardware:
int PIDEV_fprintHardware(FILE *f, PIDEV_HW *pi);


//  Set the GPIO mode for a pin.
//  One of PIDEV_IN, PIDEV_OUT, or PIDEV_ALT[0-5]
int PIDEV_setGpioMode(PIDEV_HW *pi, unsigned int pin, uint8_t mode);
//  Read or write a single pin:
int PIDEV_readGpio(PIDEV_HW *pi, unsigned int pin);     //  Returns 0 or 1
int PIDEV_writeGpio(PIDEV_HW *pi, unsigned int pin, uint8_t state);

//  Since the timer on the Pi has limite accuracy, this method
//  switches to a spin-loop under 100 microseconds.
int PIDEV_nanosleep(struct timespec *tsWait, struct timespec *tsRem);

//  Configuration of a clock:
//  Valid values are:
//      PIDEV_CLK[0,1,2,PCM,PWM]
//  Frequency is in Hertz
//  This initClock method always uses the Oscillator as a source
//  which is:
//      19.2Mhz on all Pi, except;
//      54.0Mhz on the Pi4/400
//  PLLA/PLLD are much higher frequency and are NOT used.
//  For a clean wave, configure with a freq that devides the
//  oscilator clock cleanly, eg:
//      10KHz, 100KHz
//  NOTE:  the divider field is 12 bits, so the lowest attainable
//  frequency on the Oscillator clock is: Freq/4095 (4688Hz & 13186Hz)
int PIDEV_initClock(PIDEV_HW *pi, int clock, uint64_t freq);
int PIDEV_stopClock(PIDEV_HW *pi, int clock);


//  Starts both the PWM clocks and sets the 'range' for both registers
//  PWM uses the balanced mode and is high data/range fraction
//  Note that there's only 2 data registers so a conflict exists on pins.
//   pins 12, 18, 40, 52 use the same range/data registers
//   pins 13, 19, 41, 45, 53 overlap as well
//  Note:  the 'initPwm' method has over 140ms of 'wait' time to
//  let the clock settle, otherwise 'setPwmData' does not work.
//   
int PIDEV_initPwm(PIDEV_HW *pi, uint64_t freq, uint32_t range);
int PIDEV_setPwmData(PIDEV_HW *pi, int pin, uint32_t data);



#endif


