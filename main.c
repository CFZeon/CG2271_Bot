#include "MKL25Z4.h"
#include "stdbool.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

// frequencies for C,   D,   E,   F,   G,   A,   B respectively
int frequency[] ={ 262, 294, 330, 349, 392, 440, 494 };
//								     E5   F5   B4   F5   G5   B5   A5   G5   F5   E5   F5   B4
int rickrollStart[] ={ 659, 698, 494, 698, 784, 988, 880, 784, 698, 659, 698, 494 };
//											nev  er   gon  na   give you  up   nev  er   gon  na   let  you  do   wn
int rickrollChorus[] ={ 233, 261, 277, 293, 349, 349, 311, 207, 233, 261, 207, 311, 311, 277, 233 };

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

static int gLedPos[] ={ 3, 4, 5, 6, 10, 11, 12, 13 };
volatile uint8_t ledCounter = 0;

// Motors
//	F front B back C clockwise CC counter clockwise
/*#define PIN_MOTOR_LEFT_FC	 1 //PTC1 : TPM0_CH0
#define PIN_MOTOR_LEFT_FCC 2 //PTC2 : TPM0_CH1
#define PIN_MOTOR_LEFT_BC	 29 //PTE29 : TPM0_CH2
#define PIN_MOTOR_LEFT_BCC 30 //PTE30 : TPM0_CH3

#define PIN_MOTOR_RIGHT_FC	31//PTE31 : TPM0_CH4
#define PIN_MOTOR_RIGHT_FCC	5//PTD5  :TPM0_CH5
#define PIN_MOTOR_RIGHT_BC	2//PTB2 : TPM2_CH0
#define PIN_MOTOR_RIGHT_BCC	3//PTB3 : TPM2_CH1
*/
#define PIN_MOTOR_RIGHT_FC	 1 //PTC1 : TPM0_CH0 alt4
#define PIN_MOTOR_RIGHT_FCC 2 //PTC2 : TPM0_CH1 alt4
#define PIN_MOTOR_RIGHT_BC	 29 //PTE29 : TPM0_CH2 alt3
#define PIN_MOTOR_RIGHT_BCC 30 //PTE30 : TPM0_CH3 alt 3

#define PIN_MOTOR_LEFT_FC	31//PTE31 : TPM0_CH4 alt 3
#define PIN_MOTOR_LEFT_FCC	5//PTD5  :TPM0_CH5 alt4
#define PIN_MOTOR_LEFT_BC	2//PTB2 : TPM2_CH0 alt 3
#define PIN_MOTOR_LEFT_BCC	3//PTB3 : TPM2_CH1 alt 3

#define MOD_VALUE 10000

// Buzzer
#define PIN_AUDIO 20 // PTE20 (TPM1_CH0 ALT3)
#define FREQ_MOD(x) (375000 / x)


// ??
#define RED_LED 18   // PortB Pin 18
#define GREEN_LED 19 // PortB Pin 19
#define BLUE_LED 1   // PortD Pin 1
#define SWITCH 6    // PortD Pin 6
#define MASK(x) (1 << (x))
#define PIN_NUM 3 


// UART
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

volatile int currentCommand = 0;
bool isMoving;

#define Q_SIZE (32)

//Delay function from lecture USE OSdelay() FOR RTOS!!
static void delay(volatile uint32_t nof) {
	while (nof != 0) {
		__asm("NOP");
		nof--;
	}
}

enum color_t {	
RED, GREEN, BLUE
};

//////////////////////////
// CODE CHUNK FOR AUDIO //
//////////////////////////

void initPWMBuzzer() {
	// changed to TPM1_CH0 (PTE20)

	// supplies power to the port
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// 3 here sets the pin to timer mode page 163
	PORTE->PCR[PIN_AUDIO] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PIN_AUDIO] |= PORT_PCR_MUX(3);

	// this bit controls the clock
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	// this selects the clock source for the TPM counter clock
	// TPMSRC is bits 24/25
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// set modulo value 48000000 / 128 = 375000 / 7500 = 50 Hz
	TPM1->MOD = 7500;

	// Edge-Aligned PWM
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

/////////////////////////////////
// END OF CODE CHUNK FOR AUDIO //
/////////////////////////////////

// ////////////////////////////
// CODE CHUNK FOR QUEUE HERE //
// struct for queue ///////////
/*
typedef struct {
	uint8_t Data[Q_SIZE];
	unsigned int Head; //points to oldest data element
	unsigned int Tail; //points to next free space
	unsigned int Size; // quantity of elements in queue
} Q_T;

//declaration of tx queue and rx queue
Q_T tx_q, rx_q;
*/
/*
// ISR for UART
int Q_Empty(Q_T * q) {
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
	}
	else
		return 0; // failure
}

void Q_Init(Q_T *q) {
	unsigned int i;
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
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
*/
void initUART2(uint32_t baudrate)
{
	uint32_t divisor, bus_clock;

	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PORTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PORTE23] |= PORT_PCR_MUX(4);

	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	bus_clock = (DEFAULT_SYSTEM_CLOCK)/2;
	divisor = bus_clock / (BAUD_RATE * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);

	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;

	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= UART_C2_TIE_MASK |
		UART_C2_RIE_MASK;
	UART2->C2 |= UART_C2_RIE_MASK;
	//Q_Init(&tx_q);
	//Q_Init(&rx_q);
}

void UART2_IRQHandler() {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	// for transmit
	if (UART2->S1 & UART_S1_TDRE_MASK) {
		// can send another character
		/*if (!Q_Empty(&tx_q)) {
			UART2->D = Q_Dequeue(&tx_q);
		}
		else {
			// queue is empty so disable tx
			UART2->C2 &= ~UART_C2_TIE_MASK;
		}*/
	}
	// for receiver
	if (UART2->S1 & UART_S1_RDRF_MASK) {
		// assigns integer to current command
		currentCommand =  UART2->D;
		/*
		// received a character
		if (!Q_Full(&rx_q)) {
			Q_Enqueue(&rx_q, UART2->D);
		}
		else {
			// error - queue full. (while 1 is a bad fix)
			while (1)
				;
		}
		*/
	}
	if (UART2->S1 & (UART_S1_OR_MASK |
		UART_S1_NF_MASK |
		UART_S1_FE_MASK |
		UART_S1_PF_MASK)) {
		// TO IMPLEMENT error handling
		// clear the flag
	}
}

////////////////////////////////////
// CODE CHUNK FOR QUEUE ENDS HERE //
////////////////////////////////////

void initLed(void)
{
	// Enable Clock to PORTC
	SIM->SCGC5 |= (SIM_SCGC5_PORTC_MASK);

	//Configure pins to GPIO
	PORTC->PCR[PIN_RLED] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PIN_RLED] |= PORT_PCR_MUX(1);
	//Init green led
	for (int i = 0; i < 8; i++) {
		PORTC->PCR[gLedPos[i]] &= ~PORT_PCR_MUX_MASK;
		PORTC->PCR[gLedPos[i]] |= PORT_PCR_MUX(1);
	}


	// Set Data Direction Registers for PortC
	PTC->PDDR |= GREEN_LED_MASK;
	PTC->PDDR |= MASK(PIN_RLED);
}

void initMotors() {
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;

	//Use TPM0 CH0
	PORTC->PCR[PIN_MOTOR_RIGHT_FC] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PIN_MOTOR_RIGHT_FC] |= PORT_PCR_MUX(4);
	//Use TPM0 CH1
	PORTC->PCR[PIN_MOTOR_RIGHT_FCC] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[PIN_MOTOR_RIGHT_FCC] |= PORT_PCR_MUX(4);
	//Use TPM0 CH2
	PORTE->PCR[PIN_MOTOR_RIGHT_BC] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PIN_MOTOR_RIGHT_BC] |= PORT_PCR_MUX(3);
	//Use TPM0 CH3
	PORTE->PCR[PIN_MOTOR_RIGHT_BCC] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PIN_MOTOR_RIGHT_BCC] |= PORT_PCR_MUX(3);
	//Use TPM0 CH4
	PORTE->PCR[PIN_MOTOR_LEFT_FC] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[PIN_MOTOR_LEFT_FC] |= PORT_PCR_MUX(3);
	//Use TMP0 CH5
	PORTD->PCR[PIN_MOTOR_LEFT_FCC] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_MOTOR_LEFT_FCC] |= PORT_PCR_MUX(4);
	//Use TMP2 CH0
	PORTB->PCR[PIN_MOTOR_LEFT_BC] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PIN_MOTOR_LEFT_BC] |= PORT_PCR_MUX(3);
	//Use TPM2 CH1
	PORTB->PCR[PIN_MOTOR_LEFT_BCC] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[PIN_MOTOR_LEFT_BCC] |= PORT_PCR_MUX(3);

	//Enable clock for tpm0 and tpm 2
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// this selects the clock source for the TPM counter clock
	// TPMSRC are bits 24/25
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	//cnv 1000 corresponds to 10 percent
	TPM0->MOD = 10000;
	TPM2->MOD = 10000;

	//Prescalar : divide by 128
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;

	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM2->SC &= ~TPM_SC_CPWMS_MASK;


	// Enable PWM on TPM1 Channel 0 -> PTB0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//Initialise all all CNV values to 0. Functions to move will edit this value
	TPM2_C0V = 0;
	TPM2_C1V = 0;
	TPM0_C0V = 0;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;
}
int calcCnv(int dutyCycle) {
	return ((dutyCycle / 100) * 10000);
}

void moveRightFrontCounterClockwise(int cnvValue) {
	TPM0_C0V = cnvValue;
}

void moveRightFrontClockwise(int cnvValue) {
	TPM0_C1V = cnvValue;
}

void moveRightBackCounterClockwise(int cnvValue) {
	TPM0_C2V = cnvValue;
}

void moveRightBackClockwise(int cnvValue) {
	TPM0_C3V = cnvValue;
}

void moveLeftFrontClockwise(int cnvValue) {
	TPM0_C4V = cnvValue;
}

void moveLeftFrontCounterClockwise(int cnvValue) {
	TPM0_C5V = cnvValue;
}

void moveLeftBackClockwise(int cnvValue) {
	TPM2_C0V = cnvValue;
}

void moveLeftBackCounterClockwise(int cnvValue) {
	TPM2_C1V = cnvValue;
}

void stopMotors() {
	TPM2_C0V = 0;
	TPM2_C1V = 0;
	TPM0_C0V = 0;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
	TPM0_C4V = 0;
	TPM0_C5V = 0;
}

//Flash red LED on and off 500ms or 250ms as determined
void tLEDRED(int delayTime) {
	while (1) {
		PTC->PSOR |= MASK(PIN_RLED);
		osDelay(delayTime);
		PTC->PCOR |= MASK(delayTime);
		osDelay(delayTime);
	}
}

//Flash green LED on and off 500ms or 250ms as determined
void flashGreen(int delayTime) {
	while (1) {
		PTC->PSOR |= GREEN_LED_MASK;
		osDelay(delayTime);
		// doesnt need to turn off when it's stationary
		//PTC->PCOR |= GREEN_LED_MASK;
		osDelay(delayTime);
	}
}

//Flash green 1 by 1
void alternatingGreen(int delayTime) {
	while (1) {
		PTC->PCOR |= GREEN_LED_MASK;
		PTC->PSOR |= gLedPos[ledCounter++];
		osDelay(delayTime);
		if (ledCounter >=7) {
			ledCounter =0;
		}
	}
}


void tBrain(void *argument) {
	int hold;
	for (;;)
	{
		// to check if command changed
		if (currentCommand != hold || currentCommand != 0) 
		{
			hold = currentCommand;
		}
		//to check if is moving
		if (hold >= 1 || hold <= 8) 
		{
			isMoving = true;
		}
	}
}

void tLEDGreen(void *argument) {
	for (;;)
	{
		if (isMoving == false) {
			flashGreen(0);
			tLEDRED(250);
		}
		else if (isMoving == true) {
			alternatingGreen(100);
			tLEDRED(500);
		}
	}
}

void tLEDRed(void *argument) {
	for (;;)
	{
		if (isMoving == false) {
			tLEDRED(250);
		}
		else if (isMoving == true) {
			tLEDRED(500);
		}
	}
}

void tAudio(void *argument) {
	for (;;)
	{
		bool isStart = false;
		while (1) {
			if (currentCommand == 'c' || isStart == false) {
				for (int i=0; i<12; i++) {
					// mod determines period
					TPM1->MOD = FREQ_MOD(rickrollStart[i]);
					// C0V is the one that determines duty cycle (toggle the thing down)
					// so this being half, halves the duty cycle
					TPM1_C0V = (FREQ_MOD(rickrollStart[i])) / 2;
					delay(900000);
				}
				isStart = true;
			}
			else if (currentCommand == 'c') {
				for (int i=0; i<15; i++) {
					// mod determines period
					TPM1->MOD = FREQ_MOD(rickrollChorus[i]);
					// C0V is the one that determines duty cycle (toggle the thing down)
					TPM1_C0V = (FREQ_MOD(rickrollChorus[i])) / 2;
					delay(900000);
				}
			}
			else if (currentCommand == 'f') {
				for (int i=0; i<12; i++) {
					// mod determines period
					TPM1->MOD = FREQ_MOD(rickrollStart[i]);
					// C0V is the one that determines duty cycle
					TPM1_C0V = (FREQ_MOD(rickrollStart[i])) / 2;
					delay(900000);
				}
			}
		}
	}
}


void tMotorControl() {
	for (;;)
	{
		if (isMoving == false) {
			stopMotors();
		}
		else if (isMoving == true) {
		// do action
		}
	}
}

/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 ---------------------------------------------------------------------------*/

 /*----------------------------------------------------------------------------
  * Application main thread
  ---------------------------------------------------------------------------*/
/*const osThreadAttr_t thread_attr ={
	.priority = osPriorityNormal1
};*/
void app_main(void *argument) {

	// ...
	for (;;) {
	}
}

int main(void) {

	// System Initialization
	SystemCoreClockUpdate();
	
	//init everything
	initLed();
	initUART2(9600);
	initMotors();
	initPWMBuzzer();
	
	// put everything into threads
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, NULL);			// self explanatory
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLEDGreen, NULL, NULL);			// self explanatory
	osThreadNew(tLEDRed, NULL, NULL);			// self explanatory
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osThreadNew(tAudio, NULL, NULL);    // Create application main thread
  osKernelStart();
// Start thread execution
	for (;;) {

		while (1) {
			//moveLeftFrontClockwise(2500);
			//moveLeftFrontCounterClockwise(2500);
			//moveLeftBackClockwise(2500);
			//moveLeftBackCounterClockwise(2500);
			//moveRightFrontClockwise(2500);
			//moveRightFrontCounterClockwise(2500);
			//moveRightBackClockwise(2500);
			//stopMotors();
		}
	}
}


