#include "MKL25Z4.h"
#include "led_onboard.h"
#include "control.h"

// Define constants               
#define V_MIN 0.78f   // Minimum ball position voltage
#define V_MAX 2.92f		// Maximum ball position voltage

#define PID_KP 0.8f                // Proportional gain
#define PID_KI 0.2f                  // Integral gain
#define PID_KD 4.0f                 // Derivative gain

#define I_MAX 10.0f
#define I_MIN	-10.0f
#define	C_MIN -100.0f
#define C_MAX 100.0f
#define D_MIN 4.3f                  // Minimum duty cycle (%)
#define D_MAX 8.2f                  // Maximum duty cycle (%)

// PID variables
float setpoint = 1.6f;      // Desired ball position (volts). Volatile if this value may be changed continuously in a hardware register (e.g. ADC)
float last_error = 0.0f;    // Previous error
float integral = 0.0f;			//integral accumulator
float duty_cycle;
float position = 0.0;
float control_signal = 0.0;

//ADC averaging array
uint16_t adc_samples[ADC_SAMPLES]; // Array to store samples
uint8_t adc_index = 0;             // Current index in the array	

//uint16_t stability = 0;



void initADC(void) {
  // Enable clock for ADC0 and PORTE
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;

  // Configure PTE29 as ADC input
  PORTE->PCR[29] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[29] |= PORT_PCR_MUX(0);

  // Configure ADC0
  ADC0->CFG1 = 0b10110100;
								//ADC_CFG1_ADIV(1) |    // Clock divide by 2
               //ADC_CFG1_MODE(1) |    // 12-bit resolution (for 500mm beam, ~122um resolution)
               //ADC_CFG1_ADLSMP(1); // Long sample time

  // Configure MUXSEL for ADC0_SE4b (PTE29)
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK;
	
	    uint16_t initial_value = ADC0->R[0]; // Read the first ADC value
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adc_samples[i] = initial_value; // Fill the buffer with the initial value
    }
}

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
  TPM0->SC = TPM_SC_PS(7);          //7=128
  TPM0->MOD = 3300;	//Should be: 7499=20ms?
	
  // Configure TPM0_CH3 for edge-aligned PWM
  TPM0->CONTROLS[3].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
  TPM0->CONTROLS[3].CnV = 0;        // Initialize duty cycle to 0

  // Start TPM0
  TPM0->SC |= TPM_SC_CMOD(1);
}


void boundary_test(void) {
	duty_cycle = D_MAX;								//manually set DC
	setPWMDutyCycle(duty_cycle);
	for (volatile int i = 0; i < 3500000; i++);
	duty_cycle = D_MIN;
	setPWMDutyCycle(duty_cycle);
	for (volatile int i = 0; i < 3500000; i++);
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

float calculatePID(float position) {
	
	float error = setpoint - position;
  integral += error;                       // Accumulate error
  
	if (integral > I_MAX) integral = I_MAX; // Clamp integral
  if (integral < I_MIN) integral = I_MIN;
	
	float derivative = error - last_error;  // Change in error
  last_error = error;                     // Update last error
	
  // Compute PID output
  return (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivative);
}


float scaleDutyCycle(float control_signal) {
	// Map control signal to duty cycle range
	// Clamp PID output to the valid range
    if (control_signal > C_MAX) control_signal = C_MAX;
    if (control_signal < C_MIN) control_signal = C_MIN;

    // Adjust mapping for negative control_signal
    if (control_signal < 0) {
        // Map negative control_signal to lower half of the duty cycle range
        return D_MIN + ((control_signal + 5.0f) / 5.0f) * ((D_MAX - D_MIN) / 2.0f);
    } else {
        // Map positive control_signal to upper half of the duty cycle range
        return D_MIN + ((control_signal) / 5.0f) * ((D_MAX - D_MIN) / 2.0f) + ((D_MAX - D_MIN) / 2.0f);
    }
}

void setPWMDutyCycle(float duty_cycle) {
  if (duty_cycle > D_MAX) duty_cycle = D_MAX;
  if (duty_cycle < D_MIN) duty_cycle = D_MIN;

  // Calculate CnV for the specified duty cycle
  TPM0->CONTROLS[3].CnV = (uint16_t)((duty_cycle / 100.0f) * (TPM0->MOD + 1));
}

int main(void) {

  // Initialize peripherals
  initPWM();
	led_init();
	//boundary_test();
	initADC();
	
  while (1) {
			
//		//Motor test
//		position = (float)readADC() * 3.3 / 4096; // Convert ADC to volts (Voltage/12-bit) 1.88 target (range 2.92V to 0.78V)
//			
//		if (position > 1.65) {	//ADC input LED test
//			ledwake();
//		} else {
//			ledsleep();
//		}
//		
//		uint8_t DCy = ((position/3.3)*100);
//		if (DCy < D_MIN) DCy = D_MIN;
//		if (DCy > D_MAX) DCy = D_MAX;
//		setPWMDutyCycle(DCy);
//		for (volatile int i = 0; i < 10000; i++);
			
			
			
		// Read ball position voltage
    position = (float)readADC() * 3.3f / 4096.0f; // Convert ADC value to voltage
		if (position > (setpoint*0.95f) & position < (setpoint*1.05f)) {	//Ball-center LED test (1.65 center)
			ledwake();
		} else {
			ledsleep();
		}

    // Calculate PID control signal
    control_signal = calculatePID(position);

    // Scale control signal to duty cycle range
    duty_cycle = scaleDutyCycle(control_signal);
		
		//Set PWM duty cycle for servo control
    setPWMDutyCycle(duty_cycle);
				
				
	}
}
