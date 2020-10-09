#include "MKL25Z4.h"
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

// frequencies for C,   D,   E,   F,   G,   A,   B respectively
int frequency[] = {262, 294, 330, 349, 392, 440, 494};

// LEDs
#define PIN_RLED	A1
#define PIN_GLED1	A5
#define PIN_GLED2	A4
#define PIN_GLED3	A3
#define PIN_GLED4	A2
#define PIN_GLED5	A0
#define PIN_GLED6	2
#define PIN_GLED7	4
#define PIN_GLED8	11

// Motor
#define PIN_MOTORFRP	10	// PWM
#define PIN_MOTORFRN	12
#define PIN_MOTORFLP	5	// PWM
#define PIN_MOTORFLN	13
#define PIN_MOTORBRP	9	// PWM
#define PIN_MOTORBRN	8
#define PIN_MOTORBLP	6	// PWM
#define PIN_MOTORBLN	7

// Buzzer
#define PIN_AUDIO   3

#define RED_LED 18   // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1   // PortD Pin 1
#define SWITCH 6    // PortD Pin 6
#define MASK(x) (1 << (x))
#define PIN_NUM 3 

enum color_t{RED, GREEN, BLUE};

volatile int counter = 0;



void initLed(void)
{
	// Enable Clock to PORTB and PORTD
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
		
	// Configure MUX settings to make all 3 pins GPIO
	// PCR is pin control register
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
		
	// Set Data Direction Registers for PortB and PortD
	// Configures the individual pins for input or output
	PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
	PTD->PDDR |= MASK(BLUE_LED);
}

// Setting bits to 1 turns the LED off, active low
void turnAllLedOff() {
	GPIOB->PDOR |= MASK(RED_LED) | MASK(GREEN_LED);
	GPIOD->PDOR |= MASK(BLUE_LED);
}

// Shows colour passed in on LED
void showSingleLed(enum color_t col) {
	switch (col) {
	case RED:
		GPIOB->PDOR &= ~MASK(RED_LED);
		break;
	case GREEN:
		GPIOB->PDOR &= ~MASK(GREEN_LED);
		break;
	case BLUE:
		GPIOD->PDOR &= ~MASK(BLUE_LED);
		break;
	}
}

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
const osThreadAttr_t thread_attr = {
	.priority = osPriorityNormal1
};
void app_main (void *argument) {
 
  // ...
  for (;;) {
		showSingleLed(RED);
		osDelay(1000);
		turnAllLedOff();
		osDelay(1000);
	}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initLed();
	turnAllLedOff();
  // ...
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
