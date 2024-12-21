#ifndef LED_ONBOARD_H
#define LED_ONBOARD_H

//Freedom KL25Z LEDs
#define RED_LED_POS		(18)
#define GREEN_LED_POS	(19)
#define BLUE_LED_POS (1) //PTD1

#define MASK(n)		(1UL<<n)

//Declare functions
void led_init(void);
void ledwake(void);
void ledsleep(void);


#endif
