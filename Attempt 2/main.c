#include <stdio.h>


#include "MKL25Z4.h"
#include "control.h"
#include "led_onboard.h"

#include "pid.h"

/* Controller parameters */
#define PID_KP  2.8f	//5.0f
#define PID_KI  0.08f	//6.0f
#define PID_KD  1.0f	//0.2f

#define PID_TAU 0.5f		//derivative filter time constant

#define PID_LIM_MIN -12.0f		//controller output limits (also affect the scaling for PWM range)
#define PID_LIM_MAX  12.0f

#define PID_LIM_MIN_INT PID_LIM_MIN/PID_KI		//integral accumulator limits	
#define PID_LIM_MAX_INT  PID_LIM_MAX/PID_KI

#define SAMPLE_TIME_S 0.0001f

#define DUTY_MIN 4.3f                  // Minimum duty cycle (%)
#define DUTY_MAX 8.3f                  // Maximum duty cycle (%)

//ADC averaging array
uint16_t adc_samples[ADC_SAMPLES]; // Array to store ADC samples
uint8_t adc_index = 0;             // Current index in the array	

void initPWM(void) {
  // Enable clock for TPM0 and PORTE
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

  // Configure PTE30 as TPM0_CH3 (PWM output)
  PORTE->PCR[30] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[30] |= PORT_PCR_MUX(3);

  // Set TPM0 clock source
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); //MCGFLLCLK (48MHz)

  // Configure TPM0
  TPM0->SC = TPM_SC_PS(6);          //6=64
  TPM0->MOD = 6600;	//Should be: 7499=20ms?
	
  // Configure TPM0_CH3 for edge-aligned PWM
  TPM0->CONTROLS[3].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
  TPM0->CONTROLS[3].CnV = 0;        // Initialize duty cycle to 0

  // Start TPM0
  TPM0->SC |= TPM_SC_CMOD(1);
}

void initADC(void) {
  // Enable clock for ADC0 and PORTE
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

  // Configure PTE29 as ADC input
  PORTE->PCR[29] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[29] |= PORT_PCR_MUX(0);

  // Configure ADC0
  ADC0->CFG1 = 	ADC_CFG1_ADLPC(0)		|
								ADC_CFG1_ADIV(0) 		|   // Clock divide by 2
								ADC_CFG1_ADLSMP(1)	|		
								ADC_CFG1_MODE(1) 		|   // 12-bit resolution (for 500mm beam, ~122um resolution)
								ADC_CFG1_ADICLK(0); 		// Long sample time

  // Configure MUXSEL for ADC0_SE4b (PTE29)
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
	
	    uint16_t initial_value = ADC0->R[0]; // Read the first ADC value
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_samples[i] = initial_value; // Fill the buffer with the initial value
    }
}
uint16_t readADC(void) {
  ADC0->SC1[0] = ADC_SC1_ADCH(ADC_CHANNEL); // Start conversion on CH4
  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));   // Wait for conversion
  uint16_t new_sample = ADC0->R[0];             // Read ADC result
	
	// Store the sample in the circular buffer
    adc_samples[adc_index] = new_sample;
    adc_index = (adc_index + 1) % ADC_SAMPLES; // Increment and wrap around

    // Calculate the average of the samples
    uint32_t sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += adc_samples[i];
    }

    return (uint16_t)(sum / ADC_SAMPLES); // Return the averaged value
}

float scaleDutyCycle(PIDController *pid, float out) {
	// Clamp PID output to the expected range
    if (out > PID_LIM_MAX) out = PID_LIM_MAX;
    if (out < PID_LIM_MIN) out = PID_LIM_MIN;

    // Map the PID output to the duty cycle range
     return DUTY_MIN + ((out - PID_LIM_MIN) / (PID_LIM_MAX - PID_LIM_MIN)) * (DUTY_MAX - DUTY_MIN);
}

void setPWMDutyCycle(float duty_cycle) {
  if (duty_cycle > DUTY_MAX) duty_cycle = DUTY_MAX;
  if (duty_cycle < DUTY_MIN) duty_cycle = DUTY_MIN;

  // Calculate CnV for the specified duty cycle
  TPM0->CONTROLS[3].CnV = (uint16_t)((duty_cycle / 100.0f) * (TPM0->MOD + 1));
}
int main()	{
	
  /* Initialise PID controller */
  PIDController pid = { PID_KP, PID_KI, PID_KD,
                        PID_TAU,
                        PID_LIM_MIN, PID_LIM_MAX,
												PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                        SAMPLE_TIME_S };

  PIDController_Init(&pid);
													
	led_init();
	initPWM();
	initADC();
												
	static int count = 0;

  /* Simulate response using test system */
  float setpoint = 1.51f;

  while(1) {
		
		// Read ball position voltage
		float position = (uint16_t)readADC() * 3.3f / 4096.0f; // Convert ADC value to voltage- 3.3V, 12-bit
		if (position > (setpoint*0.95f) & position < (setpoint*1.05f)) {	//Ball-center LED test (1.65 center)
		ledwake();
		} else {
			ledsleep();
		}
		
		count++;
		
		if (position > 2.82f) { position = 2.82f; }	//clamp position range to remove outliers
		if (position < 0.20f) { position = 0.20f; }

    /* Compute new control signal */
    PIDController_Update(&pid, setpoint, position);
				
		// Scale control signal to duty cycle range
		float duty_cycle = scaleDutyCycle(&pid, pid.out);
		
		//Set PWM duty cycle for servo control
		setPWMDutyCycle(duty_cycle);

		// Add a delay for stability
		//for (volatile int i = 0; i < 500; i++);
    }
		
}