/**
 *  @file config.c
 *  @author Cristian Cristea
 *  @date October 15, 2023
 *
 *  @brief Source file for the Config module
 **/


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                  Includes                                  //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#include "config.h"

#include "uart.h"
#include "utils.h"

#include <util/delay.h>
#include <avr/io.h>

#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                             Public definitions                             //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

INLINE void SetClockFrequencyWithPrescaler(uint8_t const frequency, uint8_t const prescaler)
{
    SetClockFrequency(frequency);

    // Enable the prescaler and set it to the specified value
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bm | prescaler);

    return;
}

INLINE void SetClockFrequency(uint8_t const frequency)
{
    // Enable external crystal oscillator
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm);

    // Set OSCHF as the main clock
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCHF_gc);

    // Set OSCHF clock to the specified frequency and enable auto-tune
    _PROTECTED_WRITE(CLKCTRL.OSCHFCTRLA, frequency | CLKCTRL_AUTOTUNE_bm);

    return;
}

INLINE void TightLoopContents(void)
{
    return;
}

INLINE void PrintForLogging(char const * const message)
{
#if defined __AVR128DA48__
    uart1.Print(message);
#elif defined __AVR64DD32__
    uart0.Print(message);
#else
    #error "Invalid device!"
#endif

    return;
}


////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//                                   Fuses                                    //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

#if defined __AVR128DA48__
    FUSES = {
        .BODCFG   = ACTIVE_DISABLE_gc | LVL_BODLEVEL0_gc | SAMPFREQ_128Hz_gc | SLEEP_DISABLE_gc,
        .BOOTSIZE = 0x00,
        .CODESIZE = 0x00,
        .OSCCFG   = CLKSEL_OSCHF_gc,
        .SYSCFG0  = CRCSEL_CRC16_gc | CRCSRC_NOCRC_gc | RSTPINCFG_GPIO_gc,
        .SYSCFG1  = SUT_0MS_gc,
        .WDTCFG   = PERIOD_OFF_gc | WINDOW_OFF_gc,
    };
#elif defined __AVR64DD32__
    FUSES = {
        .BODCFG   = ACTIVE_DISABLE_gc | LVL_BODLEVEL0_gc | SAMPFREQ_128Hz_gc | SLEEP_DISABLE_gc,
        .BOOTSIZE = 0x00,
        .CODESIZE = 0x00,
        .OSCCFG   = CLKSEL_OSCHF_gc,
        .SYSCFG0  = CRCSEL_CRC16_gc | CRCSRC_NOCRC_gc | RSTPINCFG_GPIO_gc | UPDIPINCFG_UPDI_gc,
        .SYSCFG1  = MVSYSCFG_DUAL_gc | SUT_0MS_gc,
        .WDTCFG   = PERIOD_OFF_gc | WINDOW_OFF_gc,
    };
#else
    #error "Invalid device!"
#endif