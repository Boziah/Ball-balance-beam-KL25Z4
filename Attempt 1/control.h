#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h> 

#define ADC_SAMPLES 25 // Number of samples for averaging
#define ADC_CHANNEL 4

//Declare variables
extern float setpoint;
extern float last_error;
extern float integral;
extern float duty_cycle;
extern uint16_t stability;
extern int count;
extern uint16_t adc_samples[ADC_SAMPLES];
extern uint8_t adc_index;
extern float position;
extern float control_signal;

//Declare functions
void initADC(void);
uint16_t readADC(void);
void initPWM(void);
void boundary_test(void);
void setPWMDutyCycle(float duty_cycle);
float calculatePID(float);
void setupComparator(void);
float scaleDutyCycle(float control_signal);

#endif
