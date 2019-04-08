#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "SysTickInts.h"
//#include "PLL.h"

unsigned long SW1;
unsigned long SW2;
unsigned long Out;

void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void PortE_Init(void);
void PortF_Init(void);
void PWM_Init(void);
void ADC_Init(void);

void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);

//void SysTick_Init(void);
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay);

volatile signed int countD = 1;
volatile signed int countF = 1;
volatile signed long CMPA_Value, CMPB_Value;
volatile signed long speed = 10000;

/**
 * main.c
 */
int main(void)
{
    disable_interrupts();
    SysTick_Init(16000);        // Initialize SysTick timer
    PortA_Init();               // Initialize Port A
    PortB_Init();               // Initialize Port B
    PortD_Init();               // Initialize Port D
    PortE_Init();               // Initialize Port E
    PortF_Init();               // Initialize Port F
    PWM_Init();                 // Initialize PWM in PE4, 5
    //ADC_Init();                 // Initialize ADC in PE3
    enable_interrupts();
    while(1){
        wait_for_interrupts();
    }

}

void PortB_Init(void) {
    disable_interrupts();
    SYSCTL_RCGC2_R |= 0x2;          // Enable clock on PORTB
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    GPIO_PORTB_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port B
    GPIO_PORTB_CR_R = 0x01;          // Allow changes to PB0
    GPIO_PORTB_DIR_R &= ~0x01;       // Set PB0 as input
    GPIO_PORTB_AFSEL_R &= ~0x01;     // Disable alternate function on PB0
    GPIO_PORTB_AMSEL_R &= ~0x01;     // Disable analog on PB0
    GPIO_PORTB_PCTL_R &= 0x00000000;// Use PB0 as GPIO
    GPIO_PORTB_PUR_R |= 0x01;        // Set PB0 as pull up
    GPIO_PORTB_DEN_R |= 0x01;        // Set PB0 as digital I/O

    // Interrupt Initialize
    GPIO_PORTB_IM_R &= ~0x01;        // Mask PB0
    GPIO_PORTB_IS_R &= ~0x01;        // Edge sensitive
    GPIO_PORTB_IBE_R &= ~0x01;       // Single edge
    GPIO_PORTB_IEV_R &= ~0x01;       // Falling edge
    GPIO_PORTB_ICR_R = 0x01;         // Clear flags
    GPIO_PORTB_IM_R |= 0x01;         // Unmask PB0
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFF0FFF) | 0x0000A000;   // Set Priority 5
    NVIC_EN0_R |= 0x2;         // Enable interrupt 1
    enable_interrupts();
}


void PortF_Init(void) {
    SYSCTL_RCGC2_R |= 0x20;          // Enable clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x20) == 0) {};
    GPIO_PORTF_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port F
    GPIO_PORTF_CR_R = 0x1F;          // Allow changes to PF0-4
    GPIO_PORTF_AMSEL_R &= 0x00;      // Disable analog on PF
    GPIO_PORTF_PCTL_R &= 0x00000000; // Use PF0-4 as GPIO
    GPIO_PORTF_DIR_R &= ~0x11;        // Set PF0, 4 as input; PF1-3 as output
    GPIO_PORTF_AFSEL_R &= 0x00;      // Disable alternate function on PF
    GPIO_PORTF_PUR_R |= 0x11;        // Enable pull-up on PF0, 4
    GPIO_PORTF_DEN_R |= 0x1F;        // Enable digital I/O on PF0-4

    // Interrupt Initialize
    GPIO_PORTF_IM_R &= ~0x10;        // Mask PF4
    GPIO_PORTF_IS_R &= ~0x10;        // Edge P
    GPIO_PORTF_IBE_R &= ~0x10;       // Single edge
    GPIO_PORTF_IEV_R &= ~0x10;       // Falling edge
    GPIO_PORTF_ICR_R = 0x10;         // Clear flags
    GPIO_PORTF_IM_R |= 0x10;         // Unmask PF4
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000;    // Set Priority 5
    NVIC_EN0_R = 0x40000000;         // Enable interrupt 30
}



void PortD_Init(void) {
    disable_interrupts();
    SYSCTL_RCGC2_R |= 0x08;          // Enable clock on PORTD
    while ((SYSCTL_PRGPIO_R & 0x08) == 0) {};
    GPIO_PORTD_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port F
    GPIO_PORTD_CR_R = 0x80;          // Allow changes to PF0-4
    GPIO_PORTD_DIR_R &= ~0x80;       // Set PD7 as input
    GPIO_PORTD_AFSEL_R &= ~0x80;     // Disable alternate function on PD7
    GPIO_PORTD_AMSEL_R &= ~0x80;     // Disable analog on PD7
    GPIO_PORTD_PCTL_R &= 0x00000000;// Use PD7 as GPIO
    GPIO_PORTD_PUR_R |= 0x80;        // Set PD7 as pull up
    GPIO_PORTD_DEN_R |= 0x80;        // Set PD7 as digital I/O

    // Interrupt Initialize
    GPIO_PORTD_IM_R &= ~0x80;        // Mask PD7
    GPIO_PORTD_IS_R &= ~0x80;        // Edge sensitive
    GPIO_PORTD_IBE_R &= ~0x80;       // Single edge
    GPIO_PORTD_IEV_R &= ~0x80;       // Falling edge
    GPIO_PORTD_ICR_R = 0x80;         // Clear flags
    GPIO_PORTD_IM_R |= 0x80;         // Unmask PD7
    NVIC_PRI0_R = (NVIC_PRI0_R & 0x0FFFFFFF) | 0xC0000000;   // Set priority 6
    NVIC_EN0_R |= 0x8;         // Enable interrupt 4
    enable_interrupts();
}

void PortA_Init(void) {
    disable_interrupts();
    SYSCTL_RCGC2_R |= 0x01;          // Enable clock on PORTA
    while ((SYSCTL_PRGPIO_R & 0x01) == 0) {};
    GPIO_PORTA_LOCK_R = 0x4C4F434B;  // Unlock GPIO Port a
    GPIO_PORTA_CR_R = 0x40;          // Allow changes to PA6
    GPIO_PORTA_DIR_R &= ~0x40;       // Set PA6 as input
    GPIO_PORTA_AFSEL_R &= ~0x40;     // Disable alternate function on PA6
    GPIO_PORTA_AMSEL_R &= ~0x40;     // Disable analog on PA6
    GPIO_PORTA_PCTL_R &= 0x00000000;// Use PA6 as GPIO
    GPIO_PORTA_PUR_R |= 0x40;        // Set PA6 as pull up
    GPIO_PORTA_DEN_R |= 0x40;        // Set PA6 as digital I/O

    // Interrupt Initialize
    GPIO_PORTA_IM_R &= ~0x40;        // Mask PA6
    GPIO_PORTA_IS_R &= ~0x40;        // Edge sensitive
    GPIO_PORTA_IBE_R &= ~0x40;       // Single edge
    GPIO_PORTA_IEV_R &= ~0x40;       // Falling edge
    GPIO_PORTA_ICR_R = 0x40;         // Clear flags
    GPIO_PORTA_IM_R |= 0x40;         // Unmask PA6
    NVIC_PRI0_R = (NVIC_PRI0_R & 0xFFFFFF0F) | 0x000000E0;   // Set priority 8
    NVIC_EN0_R |= 0x1;         // Enable interrupt 4
    enable_interrupts();
}


void PortE_Init(void){
    disable_interrupts();
    // Initialize Port E
    SYSCTL_RCGC2_R |= 0x10;          // Activate clock for PortE
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}; // Wait until PortE is ready
    GPIO_PORTE_AFSEL_R |= 0x30;      // Enable alt function on PE4
    GPIO_PORTE_PCTL_R &= 0x00FF0000;  // Clear PE4 GPIOPCTL PMCx fields
    GPIO_PORTE_PCTL_R |= 0x00550000; // Configure PE4 as PWM Module 1
    GPIO_PORTE_AMSEL_R &= ~0x30;     // Disable analog functionality on PE4
    GPIO_PORTE_DEN_R |= 0x30;        // Enable digital I/O on PE4
    enable_interrupts();
}

void PWM_Init(void) {
    disable_interrupts();
    SYSCTL_RCGCPWM_R |= 0x02;        // Enable PWM1 clock
    while ((SYSCTL_PRPWM_R & 0x02) == 0) {}; // Wait until PortE is ready
    SYSCTL_RCC_R |= 0x00100000;      // Configure PWM clock divider as the source for PWM clock
    SYSCTL_RCC_R &= ~0x000E0000;     // Clear PWMDIV field
    SYSCTL_RCC_R |= 0x00060000;      // Set divider (0x3) /16: SysClock = 16MHz, PWM clock = 1MHz
    PWM1_1_CTL_R = 0;                // Disable PWM while initializing; also configure Count-Down mode
    PWM1_1_GENA_R = 0x08C;           // Drives pwmA HIGH when counter matches value in PWM1LOAD
                                     // Drive pwmA LOW when counter matches comparator A
    PWM1_1_GENB_R = 0x80C;           // Drives pwmB HIGH when counter matches value in PWM1LOAD
                                     // Drive pwmB LOW when counter matches comparator B
    PWM1_1_LOAD_R = 10001 -1;        // Since target period is 100Hz, there are 10,000 clock ticks per period
    PWM1_1_CMPA_R = 10000 -1;        // Set 0% duty cycle to PE4
    PWM1_1_CMPB_R = 10000 -1;        // Set 0% duty cycle to PE5
    PWM1_1_CTL_R |= 0x01;            // start the timers in PWM generator 1 by enabling the PWM clock
    PWM1_ENABLE_R |= 0x0C;           // Enable M1PWM2 and M1PWM3
    enable_interrupts();
}

//void ADC_Init(void){
//    disable_interrupts();
//    SYSCTL_RCGCADC_R |= 0x1;         // Enable ADC0 clock
//    while ((SYSCTL_PRADC_R & 0x01) == 0) {}; // wait until ADC0 is ready
//    SYSCTL_RCGC2_R |= 0x10;          // 2.1) activate clock for PortE
//    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}; // wait until PortE is ready
//    GPIO_PORTE_AFSEL_R |= 0x08;      // Enable alternative function on PE3
//    GPIO_PORTE_DEN_R |= ~0x08;       // Enable digital I/O on PE3
//    GPIO_PORTE_AMSEL_R |= 0x08;      // Disable analog functionality on PE3
//    GPIO_PORTE_ADCCTL |= 0x08;       // Enable pin to trigger ADC
//
//    // Sample Sequencer Configuration
//    ADC0_ACTSS_R &= ~0x01;
//
//    enable_interrupts();
//}

/* Disable interrupts by setting the I bit in the PRIMASK system register */
void disable_interrupts(void) {
    __asm("    CPSID  I\n"
          "    BX     LR");
}


/* Enable interrupts by clearing the I bit in the PRIMASK system register */
void enable_interrupts(void) {
    __asm("    CPSIE  I\n"
          "    BX     LR");
}


/* Enter low-power mode while waiting for interrupts */
void wait_for_interrupts(void) {
    __asm("    WFI\n"
          "    BX     LR");
}

void SysTick_Handler(void){

}

void PortA_Handler(void){
    GPIO_PORTA_ICR_R = 0x40;      // acknowledge flag6
    SysTick_Wait10ms(5);          // debounce the switch: delay 10ms and then recheck the switch status
    if ((GPIO_PORTA_DATA_R & 0x40) == 0) {
        PWM1_1_CMPA_R = 10000-1;
        PWM1_1_CMPB_R = 10000-1;
        countD = 1;
        countF = 1;
    }
}

void PortB_Handler(void){
    GPIO_PORTB_ICR_R = 0x01;      // acknowledge flag0
    SysTick_Wait10ms(5);          // debounce the switch: delay 10ms and then recheck the switch status
    if ((GPIO_PORTB_DATA_R & 0x01) == 0) {
        speed-= 250;
        if (speed< 7500) speed= 9000; // reload to 10000 if it's less than 0
        CMPA_Value = abs(speed- 1); // update comparatorA value
        CMPB_Value = abs(speed- 1); // update comparatorB value
        if (countD == 2 & countF == 1){
            PWM1_1_CMPA_R = 10000-1;
            PWM1_1_CMPB_R = CMPB_Value;
        }
        if (countD == 1 & countF == 2){
            PWM1_1_CMPB_R = 10000-1;
            PWM1_1_CMPA_R = CMPA_Value;
        }
    }
}

void PortD_Handler(void){
    GPIO_PORTD_ICR_R = 0x80;      // acknowledge flag7
    SysTick_Wait10ms(5);          // debounce the switch: delay 10ms and then recheck the switch status
    if ((GPIO_PORTD_DATA_R & 0x80) == 0) {
        if (countD == 1){
            PWM1_1_CMPA_R = 10000-1;
            PWM1_1_CMPB_R = CMPB_Value;
            countD = 2;
            countF = 1;
        }
        else if (countD == 2){
            PWM1_1_CMPA_R = 10000-1;
            PWM1_1_CMPB_R = 10000-1;
            countD = 1;
            countF = 1;
        }
   }
}

void PortF_Handler(void){
    GPIO_PORTF_ICR_R = 0x11;      // acknowledge flag4
    SysTick_Wait10ms(5);          // debounce the switch: delay 10ms and then recheck the switch status
    if ((GPIO_PORTF_DATA_R & 0x10) == 0) {
        if (countF == 1){
            PWM1_1_CMPB_R = 10000-1;
            PWM1_1_CMPA_R = CMPA_Value;
            countD = 1;
            countF = 2;
        }
        else if (countF == 2){
            PWM1_1_CMPA_R = 10000-1;
            PWM1_1_CMPB_R = 10000-1;
            countD = 1;
            countF = 1;
        }
    }
}

// The delay parameter is in units of the 16 MHz core clock. (62.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}

// 160000*62.5ns equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(160000);  // wait 10ms
  }
}

