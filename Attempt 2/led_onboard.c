#include <MKL25Z4.h>
#include "led_onboard.h"

void led_init() {
	//power to port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//Make 2 pins GPIO
	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);
	
	//Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS)|MASK(GREEN_LED_POS);
	
	//Turn off LEDs
	PTB->PSOR |= MASK(RED_LED_POS)|MASK(GREEN_LED_POS);	
}

void ledwake() {
	//Turn off Red LED, tun on Green LED
	PTB->PSOR |= MASK(RED_LED_POS);
	PTB->PCOR |= MASK(GREEN_LED_POS);
	
	//PTC->PCOR |= MASK(BLUE_LED_POS);
}

void ledsleep() {
	//Turn off Green LED, tun on Red LED
	PTB->PSOR |= MASK(GREEN_LED_POS);
	PTB->PCOR |= MASK(RED_LED_POS);
	
	//PTC->PSOR |= MASK(BLUE_LED_POS);
}
