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
#define PIN_RLED	16  //portC pin16

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
//C clockwise CC counter clockwise
#define PIN_MOTOR_RIGHT_C	 0 //PTD0 : TPM0_CH0 ALT4
#define PIN_MOTOR_RIGHT_CC 	 5 //PTD5 : TPM0_CH5 ALT4
#define PIN_MOTOR_LEFT_C	 2 //PTD2 : TPM0_CH2 ALT4
#define PIN_MOTOR_LEFT_CC	 3 //PTD3 : TPM0_CH3 ALT4

#define MOD_VALUE 10000

// Buzzer
#define PIN_AUDIO 20 // PTE20 (TPM1_CH0 ALT3)
#define FREQ_MOD(x) (375000 / x)
#define TEST_AUDIO 31 // PTE31 tpm0 ch4 alt3
#define TEST_AUDIO2 0


// Legacy code
//#define RED_LED 18   // PortB Pin 18
//#define GREEN_LED 19 // PortB Pin 19
//#define BLUE_LED 1   // PortD Pin 1
//#define SWITCH 6    // PortD Pin 6
#define MASK(x) (1 << (x))
//#define PIN_NUM 3 


// UART
#define BAUD_RATE 9600
#define UART_TX_PORTE22 22
#define UART_RX_PORTE23 23
#define UART2_INT_PRIO 128

volatile uint8_t rxData = 0;
volatile uint8_t direction = 0;

osThreadId_t brainFlag;
osThreadId_t ledFlag;
osThreadId_t audioStartFlag;
osThreadId_t audioFlag;
osThreadId_t motorFlag;


// Delay function
static void delay(volatile uint32_t nof) {
  while(nof!=0) {
    __asm("NOP");
    nof--;
  }
}

/////////////////////
// TEST CODE CHUNK //
/////////////////////

void initMotor() {
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	
	//Use TPM0 CH0
	PORTD->PCR[PIN_MOTOR_RIGHT_C] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_MOTOR_RIGHT_C] |= PORT_PCR_MUX(4);
	//Use TPM0 CH5
	PORTD->PCR[PIN_MOTOR_RIGHT_CC] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_MOTOR_RIGHT_CC] |= PORT_PCR_MUX(4);
	//Use TPM0 CH2
	PORTD->PCR[PIN_MOTOR_LEFT_C] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_MOTOR_LEFT_C] |= PORT_PCR_MUX(4);
	//Use TPM0 CH3
	PORTD->PCR[PIN_MOTOR_LEFT_CC] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[PIN_MOTOR_LEFT_CC] |= PORT_PCR_MUX(4);

	//Enable clock for tpm0 
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// this selects the clock source for the TPM counter clock
	// TPMSRC are bits 24/25
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	//cnv 1000 corresponds to 10 percent
	TPM0->MOD = 10000;

	//Prescalar : divide by 128
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));
	TPM0->SC &= ~TPM_SC_CPWMS_MASK;

	// Enable PWM on TPM1 Channel 0 -> PTD0
	TPM0_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C5SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C5SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C2SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C2SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	TPM0_C3SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C3SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));

	//Initialise all all CNV values to 0. Functions to move will edit this value
	TPM0_C0V = 0;
	TPM0_C5V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
}

void initBuzzer2() {
	// changed to PTE31 TPM0_CH4 alt 3

	// supplies power to the port
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// 3 here sets the pin to timer mode page 163
	PORTE->PCR[TEST_AUDIO] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[TEST_AUDIO] |= PORT_PCR_MUX(3);

	// this bit controls the clock
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

	// this selects the clock source for the TPM counter clock
	// TPMSRC is bits 24/25
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// set modulo value
	TPM0->MOD = 10000;

	// Edge-Aligned PWM
	TPM0->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM0->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM0->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM0_C4SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM0_C4SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void initBuzzer3() {
	// changed to PTB0 tpm1 ch0 alt3

	// supplies power to the port
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	// 3 here sets the pin to timer mode page 163
	PORTB->PCR[TEST_AUDIO2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[TEST_AUDIO2] |= PORT_PCR_MUX(3);

	// this bit controls the clock
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

	// this selects the clock source for the TPM counter clock
	// TPMSRC is bits 24/25
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	// set modulo value
	TPM1->MOD = 10000;

	// Edge-Aligned PWM
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= (TPM_SC_CMOD(1)) | (TPM_SC_PS(7));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}


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

//////////////////////////////////////
// CODE CHUNK FOR UART STARTS HERE  //
//////////////////////////////////////
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
	UART2->C2 |= UART_C2_RIE_MASK;
}

void UART2_IRQHandler() {
	NVIC_ClearPendingIRQ(UART2_IRQn);
	// for transmit
	if(UART2->S1 & UART_S1_TDRE_MASK) {
	}	
	if(UART2->S1 & UART_S1_RDRF_MASK) {
		rxData = UART2->D;
		// prompts tBrain to change variables
		osEventFlagsSet(brainFlag, 0x0001);
	}
}

////////////////////////////////////
// CODE CHUNK FOR UART ENDS HERE  //
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

int calcCnv(int dutyCycle) {
	return ((dutyCycle / 100) * 10000);
}

void moveRightCounterClockwise(int cnvValue) {
	TPM0_C0V = cnvValue;
}

void moveRightClockwise(int cnvValue) {
	TPM0_C5V = cnvValue;
}

void moveLeftCounterClockwise(int cnvValue) {
	TPM0_C2V = cnvValue;
}

void moveLeftClockwise(int cnvValue) {
	TPM0_C3V = cnvValue;
}


void stopMotors() {

	TPM0_C0V = 0;
	TPM0_C5V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
}

//////////////////////
// CODE FOR THREADS //
//////////////////////

void tLED(void *argument) {
	// for the initial connection sequence, set ledFlag to 0x0001
	int ledCounter = 0;
	int storedCounter = 0;
	
  osEventFlagsWait(ledFlag, 0x0001, osFlagsWaitAny, osWaitForever);
	osEventFlagsClear(ledFlag, 0x0001);
	PTC->PSOR |= GREEN_LED_MASK;
	osDelay(250);
	PTC->PCOR |= GREEN_LED_MASK;
	osDelay(500);
	PTC->PSOR |= GREEN_LED_MASK;
	osDelay(250);
	PTC->PCOR |= GREEN_LED_MASK;
	for(;;) {
		if (direction != 0x0A) {
			storedCounter = direction;
		}
		if (storedCounter == 9 || storedCounter == 0) {
			PTC->PTOR |= MASK(PIN_RLED); //toggles red led
			PTC->PSOR |= GREEN_LED_MASK;
			osDelay(250);
		} else {
				PTC->PTOR |= MASK(PIN_RLED); //toggles green led
			PTC->PCOR |= GREEN_LED_MASK;
			PTC->PSOR |= MASK(gLedPos[ledCounter++]);
			osDelay(500);
			if (ledCounter >= 9) {
				ledCounter = 0;
			}			
		}
	}
}

void tBrain(void *argument) {
	//decodes messages received from serial comms
	uint8_t status; //current state of machine
	for (;;)
	{
		osEventFlagsWait(brainFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		osEventFlagsClear(brainFlag, 0x0001);
		
		if (rxData == 10) {
			osEventFlagsSet(audioFlag, 0x0001);
			osEventFlagsSet(ledFlag, 0x0001);
		}
		//to check if is moving
		if (rxData >= 1 || rxData <= 9) 
		{
			if (rxData >= 1 || rxData <= 8) { //moving
				direction = rxData;
			} else { // not moving
				direction = 9;
			}
			//set flag to prompt motor update
			osEventFlagsSet(motorFlag, 0x0001);
		}
	}
}

void tAudio(void *argument) {
	osEventFlagsWait(audioFlag, 0x0001, osFlagsWaitAny, osWaitForever);
	osEventFlagsClear(audioFlag, 0x0001);
	for (int i=0; i<12; i++) { // connected tone sequence
		// mod determines period
		TPM0->MOD = FREQ_MOD(rickrollStart[i]);
		TPM0_C4V = (FREQ_MOD(rickrollStart[i])) / 2;
		osDelay(100);
	}
	
	for (;;)
	{
		for (int i=0; i<15; i++) {
			// mod determines period
			TPM0->MOD = FREQ_MOD(rickrollChorus[i]);
			// C0V is the one that determines duty cycle (toggle the thing down)
			TPM0_C4V = (FREQ_MOD(rickrollChorus[i])) / 2;
			osDelay(200);
		}
		for (int i=0; i<12; i++) {
			// mod determines period
			TPM0->MOD = FREQ_MOD(rickrollStart[i]);
			// C0V is the one that determines duty cycle
			TPM0_C4V = (FREQ_MOD(rickrollStart[i])) / 2;
			osDelay(200);
		}
	}
}


void tMotorControl() {
	for (;;)
	{		
		// only updates when direction is updated by tBrain
		osEventFlagsWait(motorFlag, 0x0001, osFlagsWaitAny, osWaitForever);
		osEventFlagsClear(motorFlag, 0x0001);
		if (direction != 0) {
			if (direction == 1){ //forward
				moveRightClockwise(1000);
				moveLeftClockwise(1000);
				
			} else if (direction == 2){ //backward
				moveRightCounterClockwise(1000);
				moveLeftCounterClockwise(1000);
				
			} else if (direction == 3){ //turn left
				moveRightCounterClockwise(1000);
				moveLeftClockwise(1000);
				
			} else if (direction == 4){ //turn right
				moveRightClockwise(1000);
				moveLeftCounterClockwise(1000);
				
			} else if (direction == 5){ //forward left
				moveRightClockwise(1000);
				moveLeftClockwise(2000);
				
			} else if (direction == 6){ //forward right
				moveRightClockwise(2000);
				moveLeftClockwise(1000);
				
			} else if (direction == 7){ //reverse left
				moveRightCounterClockwise(2000);
				moveLeftCounterClockwise(1000);
				
			} else if (direction == 8){ //reverse right
				moveRightCounterClockwise(1000);
				moveLeftCounterClockwise(2000);
				
			}
		}
		else {
			stopMotors();
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

int main(void) {

	// System Initialization
	SystemCoreClockUpdate();
	
	//init system components
	initLed();
	initUART2(9600);
	initMotor();
	initBuzzer2();
	
	// initialize flags
	ledFlag = osEventFlagsNew(NULL);
	brainFlag = osEventFlagsNew(NULL);
	audioFlag = osEventFlagsNew(NULL);
	motorFlag = osEventFlagsNew(NULL);
	
	// put everything into threads
	osKernelInitialize();                  // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, NULL);		
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, NULL);		
	osThreadNew(tAudio, NULL, NULL);
  //osThreadId_t audioThread = osThreadNew(tAudio, NULL, NULL);
	//osThreadSetPriority(audioThread, osPriorityAboveNormal);
  osKernelStart();                       // Start thread execution
	for (;;) {
	}
	//int ledCounter = 0;
	while (1) {
		//moveLeftClockwise(2500);
		//moveLeftCounterClockwise(2500);
		//moveRightClockwise(2500);
		//moveRightCounterClockwise(2500);
		//stopMotors();
		/*PTC->PCOR |= GREEN_LED_MASK;
		delay(1000000);
		PTC->PSOR |= MASK(gLedPos[2]);
		delay(1000000);
		PTC->PSOR |= MASK(gLedPos[3]);
		delay(1000000);
		PTC->PSOR |= MASK(gLedPos[4]);
		delay(1000000);
		PTC->PSOR |= MASK(gLedPos[5]);
		delay(1000000);
		ledCounter = ledCounter + 1;
			PTC->PSOR |= MASK(PIN_RLED); //toggles red led
		
			
			if (ledCounter >= 9) {
				ledCounter = 0;
	}*/
	}
}