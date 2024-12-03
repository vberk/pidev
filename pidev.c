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

#include "pidev.h"




const char *PIDEV_modelNames[24]=
{
    "Pi A",
    "Pi B",
    "Pi A+",
    "Pi B+",
    "Pi 2B",
    "Alpha",
    "Compute Module",
    "UNKNOWN",
    "Pi 3",
    "Pi Zero",
    "Compute Module 3",
    "UNKNOWN",
    "Pi Zero W",
    "Pi 3B+",
    "Pi 3A+",
    "UNKNOWN",
    "Compute Module 3+",
    "Pi 4B",
    "UNKNOWN",
    "Pi 400",
    "Compute Module 4",
    "UNKNOWN",
    "UNKNOWN",
    "UNKNOWN"
};

const char *PIDEV_archNames[8]= 
{
    "BCM2835",
    "BCM2836",
    "BCM2837",
    "BCM2711",
    "UNKNOWN",
    "UNKNOWN",
    "UNKNOWN",
    "UNKNOWN"
};

const char *PIDEV_mfgNames[8]=
{
    "Sony",
    "Egoman",
    "Embest",
    "Sony Japan",
    "Embest",
    "Stadium",
    "UNKNOWN",
    "UNKNOWN"
};



//  Pin maps:
//  Physical pin on the board to the gpio pin on the CPU:
//  There's 40 pins, but counting starts at 1:
//  From: https://projects.raspberrypi.org/en/projects/rpi-gpio-pins
const int PIDEV_physToGpio[41]=
{
    -1,         //  Pin '0' does not exist
    -1, -1,     //  3V3 5V  (pin 1, 2)
     2, -1,     //  GPIO2 5V
     3, -1,     //  GPIO3 GND
     4, 14,     //  GPIO4 GPIO14
    -1, 15,     //  GND GPIO15
    17, 18,     //  GPIO17 GPIO18
    27, -1,     //  GPIO27 GND
    22, 23,     //  GPIO22 GPIO23
    -1, 24,     //  3V3 GPIO24
    10, -1,     //  GPIO10 GND
     9, 25,     //  GPIO9 GPIO25
    11,  8,     //  GPIO11 GPIO8
    -1,  7,     //  GND GPIO7
     0,  1,     //  DNC DNC -- EEPROM pins
     5, -1,     //  GPIO5 GND
     6, 12,     //  GPIO6 GPIO12
    13, -1,     //  GPIO13 GND
    19, 16,     //  GPIO19 GPIO16
    26, 20,     //  GPIO26 GPIO20
    -1, 21      //  GND GPIO21
};

//  This is the style of the old model B layout:
const int PIDEV_physToGpioOld[27]=
{
    -1,         //  Pin '0' does not exist
    -1, -1,     //  3V3 5V  (pin 1, 2)
     0, -1,     //  GPIO0 5V
     1, -1,     //  GPIO1 GND
     4, 14,     //  GPIO4 GPIO14
    -1, 15,     //  GND GPIO15
    17, 18,     //  GPIO17 GPIO18
    21, -1,     //  GPIO21 GND
    22, 23,     //  GPIO22 GPIO23
    -1, 24,     //  3V3 GPIO24
    10, -1,     //  GPIO10 GND
     9, 25,     //  GPIO9 GPIO25
    11,  8,     //  GPIO11 GPIO8
    -1,  7      //  GND GPIO7
};






//  Debug to print a few details about the hardware:
int PIDEV_fprintHardware(FILE *f, PIDEV_HW *pi)
{
    if ((*pi).rev==0)
        fprintf(f, "Not a Raspberry Pi\n");
    else
        fprintf(f, "Raspberry Pi rev %x model %s %imb arch:%s/r%i mfg:%s\n", 
            (*pi).rev, PIDEV_modelNames[(*pi).model], (256<<(*pi).ram),
            PIDEV_archNames[(*pi).arch], (*pi).brev, PIDEV_mfgNames[(*pi).mfg]);
    return(0);
}




//  Memory unmap the ranges:
int PIDEV_closeHardware(PIDEV_HW *pi)
{
    if ((*pi).simulation==1)
    {
        if ((*pi).gpio!=NULL) free((*pi).gpio);
        if ((*pi).clock!=NULL) free((*pi).clock);
        if ((*pi).pwm!=NULL) free((*pi).pwm);
        if ((*pi).spi!=NULL) free((*pi).spi);
    }
    else
    {
        if ((*pi).gpio!=MAP_FAILED) munmap((*pi).gpio, PIDEV_GPIO_LEN);
        if ((*pi).clock!=MAP_FAILED) munmap((*pi).clock, PIDEV_CLOCK_LEN);
        if ((*pi).pwm!=MAP_FAILED) munmap((*pi).pwm, PIDEV_PWM_LEN);
        if ((*pi).spi!=MAP_FAILED) munmap((*pi).spi, PIDEV_SPI_LEN);
    }
    return(0);
}



//  This code figures out the hardware we have and where the control
//  registers are.  Inspiration by the pigpio and wiringpi libraries.
//  Based on the 32-bit revision number from /proc/cpuinfo
//
//  Bit:
//      0-3   revision of the specific hardware board
//     4-11   The actual model (Zero vs. PI3B+ etc)
//    12-15   Processor architecture (BCM2835 vs. BCM2711)
//    16-19   Manufacturer (Sony, Egoman, Embest, etc.)
//    20-22   Ram size (256mb-4gb)
//       23   Old vs. new style revision coding
//    24-25   Warranty?
//
//  If bit 23 is NOT set, then the revision numbers are a simple list:
//      02-03:   B/1, 256
//      04-06:   B/1.2, 256
//      07-09:   A/1.2, 256
//      0d-0f:   B/1.2, 512
//      10-19:   B+/1.2, 512
//      11-1a:   Compute/1.1, 512
//      12-1b:   A+/1.1, 256/512
//       
//
int PIDEV_setupHardware(PIDEV_HW *pi)
{
    FILE *file=NULL;
    char buf[256];
    uint32_t rev=0;
    int gpioBaseAddr=0;
    int fd=-1;

        //  Init 'pi'
    (*pi).gpio=MAP_FAILED;
    (*pi).clock=MAP_FAILED;
    (*pi).pwm=MAP_FAILED;
    (*pi).spi=MAP_FAILED;

        //
        //  Step one is determine the revision
        //
    if ((*pi).simulation==1)
    {
        //  Pretend to be a 3B+
        rev=0xa020d3;
    }
    else
    {
        file=fopen("/proc/cpuinfo", "r");
        if (file==NULL) return(-1);
        while(fgets(buf, 256, file)!=NULL)
        {
            //  The word "Revision" shows up at the start of the line:
            if (strncmp(buf, "Revision", 8)==0)
            {
                //  Parse the hex value:
                int p=8;
                int l=strlen(buf);
                while (p<l)
                {
                    if (buf[p]>='0' && buf[p]<='9') rev=(rev<<4)+(buf[p]-'0');
                    else if (buf[p]>='a' && buf[p]<='f') rev=(rev<<4)+(buf[p]-'a'+10);
                    else if (buf[p]>='A' && buf[p]<='F') rev=(rev<<4)+(buf[p]-'A'+10);
                    else if (buf[p]=='\n') p=l;
                    p+=1;
                }
            }
        }
        fclose(file);
    }

        //
        //  Parse the revision: bit 23 determines the schema:
        //
    (*pi).rev=rev;
    if (rev==0) return(-2);
    if (rev&0x800000)
    {
        //  
        //  New style:
        //
        (*pi).ram=  (rev&0x700000)>>20;
        (*pi).mfg=  (rev&0x0f0000)>>16;
        (*pi).arch= (rev&0x00f000)>>12;
        (*pi).model=(rev&0x000ff0)>>4;
        (*pi).brev= (rev&0x00000f);
    }
    else
    {
        //
        //  Old style:
        //  NOTE: UNTESTED
        //
        rev&=0xff;
        (*pi).arch=PIDEV_ARCH_2835;  //  All BCM2835
        (*pi).ram=0;                 //  Most RAM=256mb
        (*pi).mfg=0;                 //  Ignored
        if (rev>=0x2 && rev<=0x6)
        {
            //  Model B
            (*pi).model=1;
        }
        else if (rev>=0x7 && rev<=0x9)
        {
            //  Model A
            (*pi).model=0;
        }
        else if (rev>=0xd && rev<=0xf)
        {
            //  Model B/512
            (*pi).model=1;
            (*pi).ram=1;
        }
        else if (rev>=0x10 && rev<=0x19)
        {
            //  Model B+/512
            (*pi).model=3;
            (*pi).ram=1;
        }
        else if (rev>=0x11 && rev<=0x1a)
        {
            //  Compute Module/512
            (*pi).model=6;
            (*pi).ram=1;
        }
        else if (rev>=0x12 && rev<=0x1b)
        {
            //  Model A+/256/512
            (*pi).model=2;
            if (rev==0x15)
                (*pi).ram=1;
        }
    }

    //  
    //  Configure:
    //
    switch ((*pi).arch)
    {
        case PIDEV_ARCH_2835:
            (*pi).gpioBaseAddr=PIDEV_GPIO_BASE_2835;
            break;

            //  Note: some 2711s have the same 0x20...00 base address as the 2835
        case PIDEV_ARCH_2711:
            (*pi).gpioBaseAddr=PIDEV_GPIO_BASE_2711;
            break;

        case PIDEV_ARCH_2836:
        case PIDEV_ARCH_2837:
        default:
            (*pi).gpioBaseAddr=PIDEV_GPIO_BASE_2836;
            break;
    }

    //  Size of a block:
    (*pi).pageSize=sysconf(_SC_PAGESIZE);
    if ((*pi).pageSize<=0)
        (*pi).pageSize=4096;    //  According to the books it is 4k


    //  Mapping the ranges:
    if ((*pi).simulation==1)
    {
        (*pi).gpio=malloc(PIDEV_GPIO_LEN);
        (*pi).clock=malloc(PIDEV_CLOCK_LEN);
        (*pi).pwm=malloc(PIDEV_PWM_LEN);
        (*pi).spi=malloc(PIDEV_SPI_LEN);
    }
    else
    {
        //  Try to open "all-mem"
        fd=open(PIDEV_ALLMEM, O_RDWR|O_SYNC);
        if (fd<0)
            return(-1);
        //  Need the offset into mem:
        gpioBaseAddr=(*pi).gpioBaseAddr;

        //
        //  Note:  it is possible to open /dev/gpiomem which in many cases
        //  does not need 'root' permission, but it only gives access to the
        //  gpio registers, and not the clocks, pwm, or others...
        //
        (*pi).gpio=mmap(NULL, PIDEV_GPIO_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpioBaseAddr+0x0200000);
        (*pi).clock=mmap(NULL, PIDEV_CLOCK_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpioBaseAddr+0x0101000);
        (*pi).pwm=mmap(NULL, PIDEV_PWM_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpioBaseAddr+0x020C000);
        (*pi).spi=mmap(NULL, PIDEV_SPI_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, fd, gpioBaseAddr+0x0204000);

        close(fd);
    }

    if ((*pi).gpio==MAP_FAILED || (*pi).clock==MAP_FAILED || (*pi).pwm==MAP_FAILED || (*pi).spi==MAP_FAILED)
    {
        PIDEV_closeHardware(pi);
        return(-3);
    }

    return(0);
}


//
//  Set the GPIO mode for a pin.
//  The first 6 words of 32 bits set the mode for the pin.
//
int PIDEV_setGpioMode(PIDEV_HW *pi, unsigned int pin, uint8_t mode)
{
    unsigned int r=pin/10;      //  First 6 regs of gpiomem are the modes
    unsigned int s=(pin%10)*3;  //  Shift into the word

    if (pin>PIDEV_MAXPIN) return(-1);

    (*pi).gpio[r]=((*pi).gpio[r] & ~(0x7<<s)) | ((mode&0x7)<<s);

    return(0);
}

//  Read a pin:
//  The level registers are words 13 and 14 into gpiomem
int PIDEV_readGpio(PIDEV_HW *pi, unsigned int pin)
{
    unsigned int r=pin>>5;      //  Becomes either 0 or 1, add 13
    unsigned int s=pin&0x1f;

    if (pin>PIDEV_MAXPIN) return(-1);

    if ((*pi).gpio[r+13]&(0x1<<s))
        return(1);
    return(0);
}

//  Write a pin:
//  The set/clr is different registers banks.  
//  Set is banks 7+8, and clear is banks 10+11
int PIDEV_writeGpio(PIDEV_HW *pi, unsigned int pin, uint8_t state)
{
    unsigned int r=(pin>>5)+7;    //  Becomes 0 or 1, add 7 or 10
    unsigned int s=pin&0x1f;

    if (pin>PIDEV_MAXPIN) return(-1);

    if (state==0)
        r+=3;

    (*pi).gpio[r]=0x1<<s;
    return(0);
}


//
//  Ran into significant issues with "nanosleep", the fastest I
//  could get it to go was 7.4Khz on/off (effective 68 microseconds)
//  So anything under 100 microseconds, use a loop...
//  Using a loop, I still don't get much faster than 156Khz
//
int PIDEV_nanosleep(struct timespec *tsWait, struct timespec *tsRem)
{
    long long tus=(*tsWait).tv_sec*1000000+(*tsWait).tv_nsec/1000;

    //  100us accuracy TH
    if (tus<100)
    {
        //  The accuracy here is in milliseconds:
        struct timeval tsStart, tsNow;
        long long dt=0;

        gettimeofday(&tsStart, NULL);
        while (dt<tus)
        {
            //  At least give up the CPU before we check the clock:
            sched_yield();
            gettimeofday(&tsNow, NULL);
            dt=(tsNow.tv_sec-tsStart.tv_sec)*1000000+(tsNow.tv_usec-tsStart.tv_usec);
        }

        //  No time remaining.
        if (tsRem)
        {
            (*tsRem).tv_sec=0;
            (*tsRem).tv_nsec=0;
        }
    }
    else
        return(nanosleep(tsWait, tsRem));
    return(0);
}



//
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
int PIDEV_initClock(PIDEV_HW *pi, int clock, uint64_t freq)
{
    int ctrl, div;
    uint32_t idiv=0;
    uint32_t fdiv=0;
    uint32_t mash=0;
    struct timespec tsWait;

        //  
        //  Select the clock registers:
        //
    switch(clock)
    {
        case PIDEV_CLK0:
            ctrl=PIDEV_CLK0_CTRL;
            div=PIDEV_CLK0_DIV;
            break;
        case PIDEV_CLK1:
            ctrl=PIDEV_CLK1_CTRL;
            div=PIDEV_CLK1_DIV;
            break;
        case PIDEV_CLK2:
            ctrl=PIDEV_CLK2_CTRL;
            div=PIDEV_CLK2_DIV;
            break;
        case PIDEV_CLKPCM:
            ctrl=PIDEV_CLKPCM_CTRL;
            div=PIDEV_CLKPCM_DIV;
            break;
        case PIDEV_CLKPWM:
            ctrl=PIDEV_CLKPWM_CTRL;
            div=PIDEV_CLKPWM_DIV;
            break;
        default:
            return(1);
            break;
    }

        //
        //  Compute idiv and fdiv (there's no mash unless fdiev!=0)
        //
    if (freq>0)
    {
        //  Determine the frequency of the oscillator
        uint64_t oscFreq=PIDEV_BCM2835_OSC_FREQ;
        if ((*pi).arch==PIDEV_ARCH_2711)
            oscFreq=PIDEV_BCM2711_OSC_FREQ;
        
        //  These are both 12-bit numbers:
        idiv=(uint32_t)(oscFreq/freq);
        fdiv=(uint32_t)((double)((oscFreq%freq)<<12)/(double)oscFreq);
        idiv&=0xfff;
        fdiv&=0xfff;
        if (fdiv)
            mash=0x1<<9;
    }

        //
        //  Stop the clock
        //
    if ((*pi).clock[ctrl] & (PIDEV_CLK_CTRL_BUSY|PIDEV_CLK_CTRL_ENABLE))
    {
        struct timeval tsStart, tsNow;
        long long dt=0;

//fprintf(stderr, "  stopping clock 0\n");
        //(*pi).clock[PIDEV_CLK0_CTRL] = PIDEV_CLK_PASSWD | 0;
        (*pi).clock[ctrl] = ((*pi).clock[ctrl] & (~PIDEV_CLK_CTRL_ENABLE)) | PIDEV_CLK_PASSWD;


//fprintf(stderr, "  waiting for NOT busy\n");
            //  Wait at most 1/100th of a second for the clock to no longer
            //  be busy.  It should go much quicker (min freq is 4688Hz)
            //  Experimentation at slowest clock showeed about 217us max
        gettimeofday(&tsStart, NULL);
        while(((*pi).clock[ctrl] & PIDEV_CLK_CTRL_BUSY) && dt<10000)
        {
            //sched_yield();
            gettimeofday(&tsNow, NULL);
            dt=(tsNow.tv_sec-tsStart.tv_sec)*1000000+(tsNow.tv_usec-tsStart.tv_usec);
            /* nothing else */;
        }
        //  Did the busy flag clear?
        if ((*pi).clock[ctrl] & PIDEV_CLK_CTRL_BUSY)
        {
//            fprintf(stderr, "Clock still busy: %lli us\n", dt);
            return(1);
        }
//        else
//            fprintf(stderr, "Clock cleared in %lli us\n", dt);
    }

        //
        //  Configure and start:
        //
    if (freq>0)
    {
            //  Set the divider, and clock modes:
//fprintf(stderr, "Set divider %i / %i\n", idiv, fdiv);
        (*pi).clock[div]=PIDEV_CLK_PASSWD | (idiv<<12) | fdiv;
        //  Wait a bit:
        tsWait.tv_sec=0;
        tsWait.tv_nsec=10000;  //  Wait time of 10us
        PIDEV_nanosleep(&tsWait, NULL);

            //  Start the clock
//fprintf(stderr, "Start clock 0\n");
        (*pi).clock[ctrl]=PIDEV_CLK_PASSWD | mash | PIDEV_CLK_CTRL_OSC;
        //  Wait a bit:
        tsWait.tv_sec=0;
        tsWait.tv_nsec=10000;  //  Wait time of 10us
        PIDEV_nanosleep(&tsWait, NULL);

            //  The manual states to set 'enable' separately,
            //  fails to mention how long to wait:
        (*pi).clock[ctrl]=PIDEV_CLK_PASSWD | mash | PIDEV_CLK_CTRL_OSC| PIDEV_CLK_CTRL_ENABLE;
    }
    return(0);
}

int PIDEV_stopClock(PIDEV_HW *pi, int clock)
{
    return(PIDEV_initClock(pi, clock, 0));
}



//
//  Set hardware PWM
//
int PIDEV_initPwm(PIDEV_HW *pi, uint64_t freq, uint32_t range)
{
    struct timespec tsWait;
    //uint32_t pwmState=0;
    int rc;

    //  Stop PWM first.
    //pwmState=(*pi).pwm[PIDEV_PWM_CTRL];     //  Save the state to restore.
    (*pi).pwm[PIDEV_PWM_CTRL]=0;

    //  Set clock to 10khz
    rc=PIDEV_initClock(pi, PIDEV_CLKPWM, freq);
    if (rc!=0) 
        return(rc);


    //  This long wait seems necessary after initialization of
    //  the clock:
    tsWait.tv_sec=0;
    tsWait.tv_nsec=100000;
    PIDEV_nanosleep(&tsWait, NULL);

    //  Enable both PWM modules and set balanced mode (default)
    //  (*pi).pwm[PIDEV_PWM_CTRL]=pwmState;
    (*pi).pwm[PIDEV_PWM_CTRL] = PIDEV_PWM_CTRL_ENABLE | (PIDEV_PWM_CTRL_ENABLE<<8);

    //  Set the PWM range number (default 1024)
    //  This is the number of bits that will be pushed out each time.
    //  Again for both channels:
    (*pi).pwm[PIDEV_PWM_RANGE0]=range;
    (*pi).pwm[PIDEV_PWM_RANGE1]=range;

    //  Finally set an empty PWM duty cycle for both registeres:
    (*pi).pwm[PIDEV_PWM_DATA0]=0;
    (*pi).pwm[PIDEV_PWM_DATA1]=0;

        //  Wait, this is necessary, and setting PWM data before
        //  this wait expires seems to fail to be picked up.
    tsWait.tv_sec=0;
    tsWait.tv_nsec=100000;
    PIDEV_nanosleep(&tsWait, NULL);

    return(0);
}


//
//  Figues out which pin maps to which data register
//  and sets the data, checking range:
//
int PIDEV_setPwmData(PIDEV_HW *pi, int pin, uint32_t data)
{
    int alt=0;
    int dreg=0;

    //  Which channel?
    if (pin==12 || pin==18 || pin==40 || pin==52)
        dreg=PIDEV_PWM_DATA0;
    else if (pin==13 || pin==19 || pin==41 || pin==45 || pin==53)
        dreg=PIDEV_PWM_DATA1;
    else
        return(1);

    //  Alt func:
    if (pin==12 || pin==13 || pin==40 || pin==41 || pin==45)
        alt=PIDEV_ALT0;
    else if (pin==18 || pin==19)
        alt=PIDEV_ALT5;
    else if (pin==52 || pin==53)
        alt=PIDEV_ALT1;
    else
        return(1);

    //  Set:
    (*pi).pwm[dreg]=data;
    PIDEV_setGpioMode(pi, pin, alt);

    return(0);
}





