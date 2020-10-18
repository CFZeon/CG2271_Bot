#include "MKL25Z4.h"
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

// frequencies for C,   D,   E,   F,   G,   A,   B respectively
int frequency[] = {262, 294, 330, 349, 392, 440, 494};
//								E5   F5   B4   F5   G5   B5   A5   G5   F5   E5   F5   B4
int rickrollStart[] = {659, 698, 494, 698, 784, 988, 880, 784, 698, 659, 698, 494};
int rickrollChorus[] = {};

// LEDs
#define PIN_RLED	5  //portC pin16
	
#define PIN_GLED1	3  //portC pin3
#define PIN_GLED2	4  //portC pin4
#define PIN_GLED3	5  //portC pin5
#define PIN_GLED4	6  //portC pin6
#define PIN_GLED5	10 //portC pin10
#define PIN_GLED6	11 //portC pin11
#define PIN_GLED7	12 //portC pin12
#define PIN_GLED8	13 //portC pin13
#define GREEN_LED_MASK 0x00003C78
	
static int gLedPos[] = {3,4,5,6,10,11,12,13};
volatile uint8_t ledCounter = 0;

// Motors
//	F front B back C clockwise CC counter clockwise
#define PIN_MOTOR_LEFT_FC	 3 //PTB3 
#define PIN_MOTOR_LEFT_FCC 2 //PTB2
#define PIN_MOTOR_LEFT_BC	 1 //PTB1
#define PIN_MOTOR_LEFT_BCC 0 //PTB0
	
#define PIN_MOTOR_RIGHT_FC	5//PTD5	 
#define PIN_MOTOR_RIGHT_FCC	0//PTD0
#define PIN_MOTOR_RIGHT_BC	2//PTD2
#define PIN_MOTOR_RIGHT_BCC	3//PTD3

// Buzzer
#define PIN_AUDIO   3

#define RED_LED 18   // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1   // PortD Pin 1
#define SWITCH 6    // PortD Pin 6
#define MASK(x) (1 << (x))
#define PIN_NUM 3 

#define Q_SIZE (32)

// 
// CODE CHUNK FOR QUEUE HERE
// struct for queue
typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head; //points to oldest data element
	unsigned int Tail; //points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;

//declaration of tx queue and rx queue
Q_T tx_q, rx_q;

enum color_t{RED, GREEN, BLUE};

// ISR for UART
void UART2_IRQHandler() {
	
}

void Q_Init(Q_T *q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty (Q_T * q) {
	return q->Size == 0;
}

int Q_Full(Q_T * q) {
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, unsigned char d) {
	if (!Q_Full(q)) {
	q->Data[q->Tail++] = d;
	q->Tail %= Q_SIZE;
	q->Size++;
	return 1; // success
	} else
	return 0; // failure
}

unsigned char Q_Dequeue(Q_T * q) {
	// Must check to see if queue is empty before dequeueing
	unsigned char t=0;
	if (!Q_Empty(q)) {
	t = q->Data[q->Head];
	q->Data[q->Head++] = 0; // to simplify debugging
	q->Head %= Q_SIZE;
	q->Size--;
	}
	return t;
}
//
// CODE CHUNK FOR QUEUE ENDS HERE
//
	
void initLed(void)
{
	// Enable Clock to PORTC
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);
		
	//Configure pins to GPIO
	PORTC->PCR[PIN_RLED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PIN_RLED] |= PORT_PCR_MUX(1);
	//Init green led
	for(int i = 0; i < 8; i++){
		PORTC->PCR[gLedPos[i]] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[gLedPos[i]] |= PORT_PCR_MUX(1);
	}

		
	// Set Data Direction Registers for PortC
	for(int i = 0; i < 8; i++){
		PTC->PDDR |= MASK(gLedPos[i]);
	}
	PTC->PDDR |= MASK(PIN_RLED);
}
//Flash red LED on and off 500ms or 250ms as determined
void tLEDRED(int delayTime){
	while(1){
		PTC->PSOR |= MASK(PIN_RLED);
		osDelay(delayTime);
		PTC->PCOR |= MASK(delayTime);
		osDelay(delayTime);
	}
}

//Flash green LED on and off 500ms or 250ms as determined
void flashGreen(int delayTime){
	while(1){
		PTC->PSOR |= GREEN_LED_MASK;
		osDelay(delayTime);
		PTC->PCOR |= GREEN_LED_MASK;
		osDelay(delayTime);
	}
}

//Flash green 1 by 1
void alternatingGreen(int delayTime){
	while(1){
		PTC->PCOR |= GREEN_LED_MASK;
		PTC->PSOR |= gLedPos[ledCounter++];
		osDelay(delayTime);
		if(ledCounter >=7){
			ledCounter =0;
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
		//turnAllLedOff();
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
