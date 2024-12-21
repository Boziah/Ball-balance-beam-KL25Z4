#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h> 

#include "pid.h"

#define ADC_SAMPLES 10 // Number of samples for averaging
#define ADC_CHANNEL 4

//Declare variables
extern uint16_t adc_samples[ADC_SAMPLES];
extern uint8_t adc_index;
extern int count;

//Declare functions
void initADC(void);
uint16_t readADC(void);
void initPWM(void);
void boundary_test(void);
void setPWMDutyCycle(float duty_cycle);
float calculatePID(float);
void setupComparator(void);
float scaleDutyCycle(PIDController *pid, float out);

#endif
