#include "ULTRASONIC.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>
void Timer0_init(int period )
{
	SYSCTL_RCGCTIMER_R|=0x00000001;
	GPIO_initPin(PORTB,PIN6,DIGITAL,PERIPHERAL);
	GPIO_initPin(PORTB,PIN7,DIGITAL,PERIPHERAL);
	GPIO_PORTB_PCTL_R&=0x77FFFFFF;
	TIMER0_CTL_R=0x00;
	TIMER0_CFG_R=0x00;
	TIMER0_TAMR_R=0x02;
	TIMER0_TAILR_R=period-1;
	TIMER0_TAPR_R=0x00;
	TIMER0_CTL_R=0x01;
}