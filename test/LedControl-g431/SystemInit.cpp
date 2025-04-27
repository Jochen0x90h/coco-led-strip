#include "config.hpp"
#include <coco/debug.hpp>
#include <coco/platform/platform.hpp>


// called from startup code to setup clock and flash before static constructors and main()
// pass -Wl,--undefined=SystemInit to gcc in addition to -Wl,--gc-sections to prevent the function from being garbage collected
extern "C" {
void SystemInit() {
    /*
        Use HSE and set system clock to 160MHz and peripheral clocks to 20MHz/40MHz
        HSI is 16MHz
        HSE is not connected

        Reference manual Table 9
        Wait states  Range 1 boost  Range 1 normal  Range 2
        0WS           34MHz          30MHz           12MHz
        1WS           68MHz          60MHz           24MHz
        2WS          102MHz          90MHz           26MHz
        3WS          136MHz         120MHz
        4WS          170MHz         150MHz
    */

    // enable clock of PWR
    RCC->APB1ENR1 = RCC->APB1ENR1 | RCC_APB1ENR1_PWREN;

    // configure flash
    FLASH->ACR = FLASH_ACR_LATENCY_4WS // 4 wait states
        | FLASH_ACR_ICEN // instruction cache enable
        | FLASH_ACR_DCEN // data cache enable
        | FLASH_ACR_PRFTEN // prefetch enable
        | FLASH_ACR_DBG_SWEN; // debug enable

    // configure PLL: 16MHz / 1 * 20 = 320MHz / 2 = 160MHz
    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI // source is internal oscillator HSI16
        | ((1 - 1) << RCC_PLLCFGR_PLLM_Pos) // PLL M divisor
        | (20 << RCC_PLLCFGR_PLLN_Pos) // PLL N multiplier
        | RCC_PLLCFGR_PLLREN // enable PLL R output
        | RCC_PLLCFGR_PLLPEN // enable PLL P output (for ADC)
        | (7 << RCC_PLLCFGR_PLLPDIV_Pos); // ADC clock: 320MHz / 7 = 45.7MHz

    // enable internal oscillator and PLL
    RCC->CR = 0x63 // bits 0-7 must be kept at reset value
        | RCC_CR_HSION // enable internal oscillator
        | RCC_CR_PLLON; // enable PLL

    // set boost mode
    PWR->CR5 = 0;

    // wait until PLL is ready
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    // use PLL, set APB1 and APB2 prescaler
    uint32_t CFGR = RCC_CFGR_SW_PLL // use PLL
        | RCC_CFGR_PPRE1_DIV4 // APB1_CLOCK = AHB_CLOCK / 4
        | RCC_CFGR_PPRE2_DIV4; // APB2_CLOCK = AHB_CLOCK / 4

    // switch to medium speed clock, wait for at least 1us according to reference manual 7.2.7, then switch to high speed clock
    RCC->CFGR = CFGR | RCC_CFGR_HPRE_DIV2; // medium speed: AHB_CLOCK = SYS_CLOCK / 2
    for (int i = 0; i < 170; ++i)
        __NOP();

    // switch to high speed clock
    RCC->CFGR = CFGR | RCC_CFGR_HPRE_DIV1; // high speed: AHB_CLOCK = SYS_CLOCK

    // set clock source of ADC to PLL "P"
    RCC->CCIPR = RCC->CCIPR | RCC_CCIPR_ADC12SEL_0;

    // enable FPU, depends on compiler flags
#if (__FPU_USED == 1)
    SCB->CPACR = SCB->CPACR | (3UL << 20) | (3UL << 22);
    __DSB();
    __ISB();
#else
    #warning "FPU is not used"
#endif

    // enable clocks of GPIO and SYSCFG
    RCC->AHB2ENR = RCC->AHB2ENR
        | RCC_AHB2ENR_GPIOAEN
        | RCC_AHB2ENR_GPIOBEN
        | RCC_AHB2ENR_GPIOCEN
        //| RCC_AHB2ENR_GPIODEN
        //| RCC_AHB2ENR_GPIOEEN
        | RCC_AHB2ENR_GPIOFEN;
    RCC->APB2ENR = RCC->APB2ENR | RCC_APB2ENR_SYSCFGEN;

    coco::debug::init();
}
}
