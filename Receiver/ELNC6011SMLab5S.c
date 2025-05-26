/*Use of AI / Cognitive Assistance Software is not allowed in any evaluation, assessment or exercise.*/
/*=============================================================================
	File Name:	ELNC6011SMLab5S.c  
	Author:		Siddhant .K. Mahindrakar
	Date:		07/04/2025
	Modified:	None
	� Fanshawe College, 2025

	Description: 	A C-program that on PIC18f45K22 microcontroller takes the analog input from the sensors connected to the input port and samples this input every second.
					After collecting 10 samples in an array, then at every next sample the average of these samples is calculated. Push buttons are connectted to add user operations 
					to the system. Upon pressing a button, system performs some action. The system performs control action on the outputs upon feedback from the sensor channels and hardcoded set limits.
					Heater, Cooler, Fan, Lights and Sprinkler are the outputs connected to the uC. A stepper Motor is connected to PORTB to control the vent damper position. The status of the system like 
					outputs/SP limits/sensor PV/damper sp and curr sp are transfered to UART1 of the uC. A Communication sentence is generated to convey the data (process data) over the UART protocol 
					when the inc/dec button is pressed. This sentence is transmitted on UART for 5sec and remains unchanged in this interval. If again a inc/dec button is pressed the state of the system 
					is stored and transmitted for next 5sec interval. 
					This message/sentence is encrypted with data which has a vital information of this system and are comma separated values. The structure of the sentence is; 
					StartsymbolBasesentence, Controller add, Node add, Channel, Mode, New value, ChecksumEndcharacter\r. A chceksum is a unique value created by Ex-oring the characters in string to be 
					transmitted. It is packed and then transmitted with the sentence, to decode the sentence recieved is a valid recieve. This sentence is used to convey that a limit is changed on the 
					system and it belongs to which channel and what limit with the new value. 
					Every time the INC/DEC button is pressed the generated communication sentence is sent to a client controller connected on USART2 of the host(this)controller.    
=============================================================================*/

/* Preprocessor ===============================================================
   Hardware Configuration Bits ==============================================*/
#pragma config FOSC		= INTIO67
#pragma config PLLCFG	= OFF
#pragma config PRICLKEN = ON
#pragma config FCMEN	= OFF
#pragma config IESO		= OFF
#pragma config PWRTEN	= OFF 
#pragma config BOREN	= ON
#pragma config BORV		= 285 
#pragma config WDTEN	= OFF
#pragma config PBADEN	= OFF
#pragma config LVP		= OFF
#pragma config MCLRE	= EXTMCLR

// Libraries ==================================================================
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <delays.h>

// Constants  =================================================================
#define TRUE	1	
#define FALSE	0	

#define T0FLAG INTCONbits.TMR0IF
#define RC1READ RCREG1
#define DCVAL 100
#define PBMASK	0xF0
#define MODE	0xE0	
#define CHANSEL	0xD0
#define INCR	0xB0
#define DEC	0x70
#define NOPRESS	0xF0
#define LD4LIGHTINGOP LATBbits.LATB3
#define LD3COOLER LATBbits.LATB2
#define LD2HEATER LATBbits.LATB1
#define LD1FAN LATBbits.LATB0
#define DELAYCOUNT 50
#define BUFSIZE 35
#define SAMPSIZE 10
#define SENCOUNT 3
#define ONESEC 10
#define FIVESEC 5

#define STEPSIZE 3
#define STEPPERMASK 0x0F
#define PATTERNCOUNT 4
#define STEPPERPORT LATB

#define ADCRES 0.0048
#define SLOPE1 0.01
#define INCPT1 0.4
#define SLOPE2 0.05
#define INCPT2 0
#define SLOPE3 0.000345833
#define INCPT3 0

#define LIGHTING LATCbits.LATC0
#define COOLER LATCbits.LATC1
#define HEATER LATCbits.LATC2
#define FAN LATCbits.LATC3
#define SPKLR LATCbits.LATC4 

#define TEMP 0
#define HUMD 1
#define CO2 2
#define CONTROLLER 1
#define MYADDY 977


//isr function (interrupt service routine) declaration
void isr(void); 

//assigning high priority vector to the interruptVector
#pragma code interruptVector = 0x0008 
void interruptVector(void)
{
	_asm
		GOTO isr //asm instruction to strore the code in one byte memory
	_endasm
}
#pragma code

// Global Variables  ==========================================================

//typedefing datatypes
typedef int sensor_t;
typedef char flag_t;


//structure for holding individual sensor data 
typedef struct
{
	sensor_t samples[SAMPSIZE];
	sensor_t avg;
	sensor_t upperLimit;
	sensor_t lowerLimit;
	sensor_t insert;
	flag_t avgReady;
}sensorCh_t;

//structure for timing variables
typedef struct
{
	char count;
	char second;
	char sec5;
	char timin5;
}limits_t;

//structure for holding state of the stepper motor
typedef struct
{
	char patCount;
	char currentPat;
	int currPos;
	int setPos;
	char isMoving;	
}stepper_t;

//structure for holding the state of each pushbutton object
typedef struct
{
	char mode;
	char chanSel;
	int pbState;
	int lastState;	
}pbs_t;

//structure for communication objects
typedef struct 
{
	flag_t commRcvd;
	flag_t commValid;
	flag_t commParsed;
	flag_t commExec;
	flag_t limitChngd;
	char buf[BUFSIZE];
	char senRdy;
	int insert;	
}serial_t;

//defining struct objects
sensorCh_t sensors[SENCOUNT];
limits_t limit = {0,0}; 
pbs_t pbs;
stepper_t vent;
serial_t rcvr;
serial_t trmt;

//vars
char pattern[PATTERNCOUNT] = {0x01,0x02,0x04,0x08};
char buf[BUFSIZE];

// Functions  =================================================================

/*>>> setOSC4MHz: ===========================================================
Author:		Siddhant Mahindrakar
Date:		02/08/2024
Modified:	None
Desc:		Sets the oscillator frequency to 4Mhz unless the frequency is stable.
Input: 		None 
Returns:	None 
 ============================================================================*/
void setOSC4MHz(void)
{
	OSCCON = 0x52;
	while(!OSCCONbits.HFIOFS);
} // setOSC4MHz::


/*>>> configureIOport:===========================================================
Author:		Siddhant Mahindrakar
Date:		02/08/2024
Modified:	21/01/2025
Desc:		Initialises the state of operation for the I/O ports. Each HEX number against particular configuration register defines the mode of operation of the particular I/O port.
Input: 		None 
Returns:	None 
 ============================================================================*/
void configureIOport(void)
{
	ANSELA = 0x07;//input port configured for digital operation 0000 0111 = 07
	LATA = 0x00; // output latched to GND at initialization
	TRISA = 0xFF; // port configured as digital input
	
	ANSELB = 0x00;//input port configured for digital operation
	LATB = 0x00; // output latched to GND at initialization
	TRISB = 0xF0; // lower nibble of the port is initialised to output and higher nibble is initialised as input 1111 0000 = F0
	
	ANSELC = 0x00;//input port configured for digital operation
	LATC = 0x00; // output latched to GND at initialization
	TRISC = 0xE0; // lower nibble of the port is initialised to output and higher nibble is initialised as input 1110 0000 = E0 
	
	ANSELD = 0x00;//input port configured for digital operation
	LATD = 0x00; // output latched to GND at initialization
	TRISD = 0xFF; // lower nibble of the port is initialised to output and higher nibble is initialised as input
	
	ANSELE = 0x00;//input port configured for digital operation
	LATE = 0x00; // output latched to GND at initialization
	TRISE = 0xFF; // port configured as digital input
	
} // configureIOport::

/*>>> resetTMR0: ===========================================================
Author:		Siddhant Mahindrakar
Date:		02/08/2024
Modified:	None
Desc:		When called this functions resets the timer. It clears the timer interrupt in the INTCON register and load the timer with a false start value represented by the hex number.
Input: 		None 
Returns:	None 
 ============================================================================*/
void resetTMR0(void)
{
	T0FLAG = FALSE; //clears TMR0 interrupt flag bit
	TMR0H = 0x3C; //loading preset start value for timer0 100ms rollover = 3CB0
	TMR0L = 0xB0; //loading preset start value for timer0
} // resetTMR0::

/*>>> configTMR0: ===========================================================
Author:		Siddhant Mahindrakar
Date:		02/08/2024
Modified:	None
Desc:		When called this function configures the Timer 0 module for the operation. 
Input: 		None 
Returns:	None 
 ============================================================================*/
void configTMR0(void)
{
	T0CON = 0x90; //timer on, 16bit operation, internal clock source, prescaller assigned, PS value 2 Lower byte = 0000
	resetTMR0(); //resets timer and loads it with preset false start value
} // configTMR0::

/*>>> serialP1Config: ===========================================================
Author:		Siddhant Mahindrakar
Date:		19/07/2024
Modified:	None
Desc:		Configures the serial UART port for the operation
Input: 		None
Returns:	None
 ============================================================================*/
void serialP1Config(void)
{
	SPBRG1 = 25; //value for baud rate generator for 9600 baud rate
	BAUDCON1 = 0x40; //BRG16 = 0
	RCSTA1 = 0x90; //SPEN = 1, CREN = 1
	TXSTA1 = 0x26; //TXEN = 1, SYNC = 0, BRGH = 1
} // eo serialP1Config::

/*>>> serialP2Config: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/04/2025
Modified:	None
Desc:		Configures the serial UART port2 for the operation
Input: 		None
Returns:	None
 ============================================================================*/
void serialP2Config(void)
{
	SPBRG2 = 12; //value for baud rate generator for 19.2K baud rate
	BAUDCON2 = 0x40; //BRG16 = 0
	RCSTA2 = 0x90; //SPEN = 1, CREN = 1
	TXSTA2 = 0x26; //TXEN = 1, SYNC = 0, BRGH = 1
} // eo serialP2Config::

/*>>> configADCON: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/06/2024
Modified:	None
Desc:		Initialises the state of operation for the ADC module. Each HEX number against particular configuration register defines the mode of operation of the ADC module.
Input: 		None 
Returns:	None 
 ============================================================================*/
void configADCON(void)
{
	ADCON0 = 0x01; //enables ADC 
	ADCON1 = 0x00; // +ve ref voltage set to VDD and -ve ref voltage set to VSS of the microcontroller
	ADCON2 = 0xA9; //right justified, 12TAD,Fosc/8
} // configADCON::

/*>>> getADCSample: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/06/2024
Modified:	None
Desc:		When called this function starts the ADC sampling of an analog input that is passed to it as an argument. Its returns the sampled input. It returns the result of ADC sampling of type Int.
Input: 		type char: Respective analog channel of microcontroller to be sampled is passed as an argument to the function. 
Returns:	type int: Since, the result of ADC sampling is returned as a combination of ADRESH and ADRESL as a 16bit result of type int.
 ============================================================================*/
int getADCSample(char chan)
{
	ADCON0bits.CHS = chan; //configures the analog channel for sampling channel chan is configured to the analog channel 0 for sampling.
	ADCON0bits.GO = TRUE; //start sampling the analog input
	while(ADCON0bits.GO); //checks the condition untill ADC sampling is in progress
	return ADRES; //returns the ADC result as a 16bit comination of ADRESH and ADRESL as typr int
} // getADCSample ::

/*>>> configIntr: ===========================================================
Author:		Siddhant Mahindrakar
Date:		19/07/2024
Modified:	None
Desc:		Configures the interrupt module for opeartion
Input: 		None
Returns:	None
 ============================================================================*/
void configIntr(void)
{
	INTCON = 0xE0; //enables all interrupts(hardware, global, TMR0)
	RCONbits.IPEN = FALSE; //disables interrupt priority
} // eo configIntr::

/*>>> systemINI: ===========================================================
Author:		Siddhant Mahindrakar
Date:		21/01/2025
Modified:	None
Desc:		This is a function when called, calls all the configuration functions at once..
Input: 		None 
Returns:	None 
 ============================================================================*/
void systemINI(void)
{
	setOSC4MHz(); // sets the oscillator frequency
	configureIOport(); //initializes I/O port
	configTMR0(); // initializes timer0
	serialP1Config(); // initializes serial port for the operation
	serialP2Config(); // initializes serial port2 for the operation
	configIntr(); // initializes interrupt module for the operation
	configADCON(); //configures and initializes the ADC module for operation
} // systemINI::

/*>>> initChannel: ===========================================================
Author:		Siddhant Mahindrakar
Date:		27/01/2025
Modified:	None
Desc:		Initializes all the members of the object, defined of type sensorCh_t.
Input: 		sensorCh_t *sensorPtr = requires a ptr of type sensorCh_t to be passed to the function. Through the ptr to the struct we
			access the members of the struct object   
Returns:	None
 ============================================================================*/
void initChannel(sensorCh_t *sensorPtr,int highLimit,int lowLimit)
{
	int index = 0;
	for (index = 0; index<SAMPSIZE ;index++)
	{
		sensorPtr->samples[index] = 0;
	}
	sensorPtr->avg =0;
	sensorPtr->upperLimit = highLimit;
	sensorPtr->lowerLimit = lowLimit ;
	sensorPtr->insert =0;
	sensorPtr->avgReady =0;	
} // eo initChannel::

/*>>> initStepper: ===========================================================
Author:		Siddhant Mahindrakar
Date:		05/03/2025
Modified:	None
Desc:		Initializes all the members of the object, defined of type stepper_t .
Input: 		stepper_t *stepPtr = requires a ptr of type stepper_t to be passed to the function. Through the ptr to the struct we
			access the members of the struct object   
Returns:	None
 ============================================================================*/
void initStepper(stepper_t  *stepPtr)
{
	stepPtr->patCount = 0;
	stepPtr->currentPat = pattern[stepPtr->patCount];
	stepPtr->currPos = 0;
	stepPtr->setPos = 0;
	stepPtr->isMoving = FALSE;	
} // eo initStepper::

/*>>> initPBS: ===========================================================
Author:		Siddhant Mahindrakar
Date:		05/03/2025
Modified:	None
Desc:		Initializes all the members of the object, defined of type pbs_t. 
Input: 		pbs_t *pbsPtr = requires a ptr of type pbs_t to be passed to the function. Through the ptr to the struct we
			access the members of the struct object   
Returns:	None
 ============================================================================*/
void initPBS(pbs_t* pbsPtr)
{
	pbsPtr->mode =FALSE;
	pbsPtr->chanSel = 0;
	pbsPtr->pbState = 0;
	pbsPtr->lastState = 0;	/// what do you mean PBMASK value
} // eo initPBS::

/*>>> changeMode: ===========================================================
Author:		Siddhant Mahindrakar
Date:		05/03/2025
Modified:	None
Desc:		Performs an operation of changing mode when it is called. 
Input: 		None   
Returns:	None
 ============================================================================*/
void changeMode(void)
{
	pbs.mode =!pbs.mode;
} // eo changeMode::

/*>>> changeChan: ===========================================================
Author:		Siddhant Mahindrakar
Date:		27/01/2025
Modified:	None
Desc:		Performs an operation of changing the sensor channel(0-2) when it is called
Input: 		None
Returns:	None
 ============================================================================*/
void changeChan(void)
{
	pbs.chanSel++;
	if(pbs.chanSel>=SENCOUNT)
	{
		pbs.chanSel=0;
	}
} // eo changeChan::

/*>>> incrementLimit: ===========================================================
Author:		Siddhant Mahindrakar
Date:		05/03/2025
Modified:	None
Desc:		When it is called, performs an operation of changing(increasing)the desired(up-low) limits   
			upon the state of mode selected by the user 
Input: 		None
Returns:	None
 ============================================================================*/
void incrementLimit(void)
{
	if(pbs.mode)
	{
		sensors[pbs.chanSel].upperLimit++;
	}
	else
	{
		sensors[pbs.chanSel].lowerLimit++;
	}
} // eo incrementLimit::

/*>>> decrementLimit: ===========================================================
Author:		Siddhant Mahindrakar
Date:		05/03/2025
Modified:	None
Desc:		When it is called, performs an operation of changing(decreasing)the desired(up-low) limits 
			upon the state of mode selected by the user
Input: 		None   
Returns:	None
 ============================================================================*/
void decrementLimit(void)
{
	if(pbs.mode)
	{
		sensors[pbs.chanSel].upperLimit--;
	}
	else
	{
		sensors[pbs.chanSel].lowerLimit--;
	}
} // eo decrementLimit::

/*>>> calCheckSum: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/04/2025
Modified:	None
Desc:		Generates a unique value by ex-oring all the characters of the string.
Input: 		char* ptr = A pointer to the array that holds the string of which a checksum is to be generated 
Returns:	char = Returns the calculated checksum.
 ============================================================================*/
char calCheckSum(char* ptr)
{
	char checkSum = 0;
	while(*ptr)
	{
		checkSum ^= *ptr;
		ptr++; 
	}
	return checkSum; 										
} // eo calCheckSum::

/*>>> displayData: ===========================================================
Author:		Siddhant Mahindrakar
Date:		27/01/2025
Modified:	None
Desc:		Display function that prints series of print statements
Input: 		None   
Returns:	None
 ============================================================================*/
void displayData(void)
{
	
	printf("\033[2J\033[HSensor system : 1191977\n");
	printf("\n\r");
	printf("Channel: %d", pbs.chanSel);
	if(pbs.mode)
	{
		printf("\tMode: High");
	}
	else
	{
		printf("\tMode: Low");
	}
	printf("\n\n\r");
	printf("Temp=%dC\tHumd=%d%%\tCO2=%2dppm", sensors[0].avg, sensors[1].avg, sensors[2].avg); 
	printf("\n\r");
	printf("HL=%2dC\t\tHL=%2d%%\t\tHL=%2dppm\t", sensors[0].upperLimit,sensors[1].upperLimit,sensors[2].upperLimit);
	printf("\n\r");
	printf("LL=%2dC\t\tLL=%2d%%\t\tLL=%2dppm", sensors[0].lowerLimit,sensors[1].lowerLimit,sensors[2].lowerLimit);
	printf("\n\n\r");
	if(HEATER)
	{
		printf("Heater: High");
	}
	else
	{
		printf("Heater: Low");
	}
	if(COOLER)
	{
		printf("\tCooler: High");
	}
	else
	{
		printf("\tCooler: Low");
	}
	if(FAN)
	{
		printf("\tFAN: High");
	}
	else
	{
		printf("\tFAN: Low");
	}
	printf("\n\r");
	
	if(LIGHTING)
	{
		printf("Lighting: High");
	}
	else
	{
		printf("Lighting: Low");
	}
	if(SPKLR)
	{
		printf("\tSprinkler: High");
	}
	else
	{
		printf("\tSprinkler: Low");
	}
	printf("\n\n\r");
	printf("Vent");
	printf("\n\n\r");
	printf("Set position=%d\t\tCurr position=%d", vent.setPos, vent.currPos);
	printf("\n\r");
	printf("Data pattern=0x%x", vent.currentPat);
	printf("\n\n\r");

	//transmit the sentence for every limit changed constantly for 5sec
	if(rcvr.limitChngd &&  (limit.timin5 <= FIVESEC))
	{
		printf("%s",buf);
    }
} // eo displayData::


/*>>> isr: ===========================================================
Author:		Siddhant Mahindrakar
Date:		19/07/2024
Modified:	None
Desc:		This is the interrupt service routine function definition at the start of the program when the code goes through the vector table
			the high interrupt vectors are executed. This high address is assigned to isr function. Therfore this ISR function executes and makes changes to the 
			operation as written by the coder.
Input: 		None 
Returns:	None 
 ============================================================================*/
void isr(void) //isr function definition
{
	static char msec100 = 0;
	static char fisecs = 0;
	if (T0FLAG) 
	{ 
		resetTMR0();
		//timming instance for 100msec
		msec100++;
		limit.count = TRUE;	

		//timming instance for one sec
		if(msec100 >= ONESEC)
		{
			limit.second = TRUE;
			msec100 = FALSE;
		}		
		
		//timming instance for one 5sec interval initiated on inc/dec button pressed
		if(rcvr.limitChngd == TRUE && limit.second == TRUE)
		{		
			if(limit.timin5 <= FIVESEC)
			{
				limit.timin5++; 
			}
		}
		
		//after 5sec clear flags
		if (limit.timin5 == FIVESEC)
		{
			limit.timin5 = 0; 
			limit.sec5 = TRUE;
			rcvr.limitChngd = FALSE;
		}
	}

	INTCON |= 0xE0; //always on global, peripheral and timer0 interrupts
} //eo isr::

/*=== MAIN: FUNCTION ==========================================================
 ============================================================================*/
void main( void )
{
	int index = 0;
	int chID = 0;
	 
	
	//initialise all configured system peripherals
	systemINI();
	
	
	//initialise struct object and its members
	initChannel(&sensors[0], 35,15);
	initChannel(&sensors[1], 65,35);
	initChannel(&sensors[2], 1400,650);
	initStepper(&vent);
	
	while(TRUE)
	{
		//reads pressed push button
		pbs.pbState = PORTA & PBMASK;
		
		//checks if button is still pressed or new pressed button is same as previous
		if(pbs.pbState != NOPRESS && pbs.pbState != pbs.lastState)
		{
			pbs.lastState = pbs.pbState;
			switch (pbs.pbState)
			{
				//when Mode button is pressed it changes the mode of the system
				case MODE:
					changeMode();
					break;	
					
				//when Channel select button is pressed it changes the sensor channel of the system	
				case CHANSEL:
					changeChan();
					break;	
					
				//when Increement button is pressed it increements the high/low limits of the system		
				case INCR:
					incrementLimit();
					//when button pressed start 5sec interval and set the flag
					limit.timin5 = 0;
					rcvr.limitChngd = TRUE;
					trmt.senRdy = TRUE;
					break;
				
				//when Decrement button is pressed it decreements the high/low limits of the system			
				case DEC:
					decrementLimit();
					//when button pressed start 5sec interval and set the flag
					limit.timin5 = 0;
					rcvr.limitChngd = TRUE;
					trmt.senRdy = TRUE;
					break;
				default:
					break; 
			}
		}
		else if(pbs.pbState == NOPRESS)
		{
			pbs.lastState = 0;	
		}
					

		//at one msec interval do the following
		if(limit.count == TRUE)
		{
			//clear the flag
			limit.count = FALSE;
			
			//check every 100msec whether curr vent pos is equal to set pos	
			if(vent.currPos != vent.setPos)
			{
				//rotate clockwise if curr is less than set pos
				if(vent.setPos > vent.currPos)
				{
					vent.patCount++;
					if(vent.patCount >= PATTERNCOUNT)
					{
						vent.patCount = 0;
					}
					vent.currentPat = pattern[vent.patCount];
					
					//advance motor movement forward acording to defined stepsize deg/step 3 in this case
					vent.currPos += STEPSIZE;
					STEPPERPORT = STEPPERMASK & vent.currentPat;
				}
				
				// rotate counter-clockwise if curr pos is less than set pos
				if(vent.setPos < vent.currPos)
				{
					vent.patCount--;
					if(vent.patCount < 0)
					{
						vent.patCount = PATTERNCOUNT - 1;
					}
					vent.currentPat = pattern[vent.patCount];
					
					//advance motor movement reverse acording to defined stepsize deg/step 3 in this case
					vent.currPos -= STEPSIZE;
					STEPPERPORT = STEPPERMASK & vent.currentPat;
				}
			}
		}
		
		//at onesec interval do the following
		if(limit.second == TRUE)
		{
			//clear the flag
			limit.second = FALSE;
			
			//take samples from input sensors and populate the samples array every one sec
			for(chID = 0; chID<SENCOUNT; chID++)
			{
				sensors[chID].samples[sensors[chID].insert] = getADCSample(chID);
				sensors[chID].insert++;
				
				//after collection of 10 samples bump the flag to indicate average is ready
				if(sensors[chID].insert>=SAMPSIZE)
				{
					sensors[chID].insert = 0;
					sensors[chID].avgReady = TRUE;
				}
				
				//after collection of 10 samples calculate the average for every next sample
				if(sensors[chID].avgReady)
				{
					float val = 0;
					long sum =0;
					for(index =0;index<SAMPSIZE;index++)
					{
						sum += sensors[chID].samples[index];
					}
					sensors[chID].avg = sum/SAMPSIZE;
					
					// Convert to a voltage (y)
					val = sensors[chID].avg * ADCRES; 
					switch( chID )
					{
						//scaling analog input1
						case TEMP:
							val -= INCPT1; // (y-b)
							sensors[chID].avg = val / SLOPE1;
							
							//control action for outputs dependent on sensor0
							if (sensors[chID].avg > sensors[chID].upperLimit)
							{
								COOLER = 1;
								HEATER = 0;
								FAN = 1;
								vent.setPos = 90;
								
							}
							else if (sensors[chID].avg < sensors[chID].lowerLimit)
							{
								COOLER = 0;
								HEATER = 1;
								FAN = 1;
								vent.setPos = 6;
								
							}	
							else 
							{
								COOLER = 0;
								HEATER = 0;
								FAN = 0;
							}
							break;
						case HUMD:
							//scaling analog input2
							sensors[chID].avg = val / SLOPE2;
							
							//control action for outputs dependent on sensor1
							if (sensors[chID].avg > sensors[chID].upperLimit)
							{
								SPKLR = 0;
								vent.setPos = 66;						
							}
							else if (sensors[chID].avg < sensors[chID].lowerLimit)
							{
								SPKLR = 1;
								vent.setPos = 12;
							}	
					        else 
							{
								SPKLR = 0;
							}	
							break;
						case CO2:
							//scaling analog input3
							sensors[chID].avg = val / SLOPE3;
							
							//control action for outputs dependent on sensor2
							if(sensors[chID].avg > sensors[chID].upperLimit)
							{
								FAN = 1;
								vent.setPos = 9;
							}
							else if(sensors[chID].avg < sensors[chID].lowerLimit)
							{
								FAN = 1;
								vent.setPos = 90;
							}
							break;
						default:
							break;
					}// eo switch for linear eqn	
				}//eo if avgRdy
			}//eo for
			
			//at one 5sec interval do the following 
			//generate and save locally the sentence for every limit changed for 5sec.
			if(rcvr.limitChngd && (limit.timin5 <= FIVESEC) )
			{
				//do this for high limit
				if(pbs.mode)
				{
					//generate and save locally the sentence
					sprintf(buf,"$CONLIM,%d,%d,%d,%d,%d",CONTROLLER,MYADDY,pbs.chanSel,pbs.mode,sensors[pbs.chanSel].upperLimit);
					//attach the checksum and store locally
					sprintf(buf,"%s,%d\r",buf,calCheckSum(buf));
					//when incr/dec button is pressed send info to client connected on USART2
					if(trmt.senRdy == TRUE)
					{
						trmt.senRdy = FALSE;
						//send the info stored in buf to the client connected on USART2
						puts2USART(buf);
					}
				}
				
				//do this for low limit
				else
				{
					//generate and save locally the sentence
					sprintf(buf,"$CONLIM,%d,%d,%d,%d,%d",CONTROLLER,MYADDY,pbs.chanSel,pbs.mode,sensors[pbs.chanSel].lowerLimit);
					//attach the checksum and store locally
					sprintf(buf,"%s,%d\r",buf,calCheckSum(buf));
					//when incr/dec button is pressed send info to client connected on USART2
					if(trmt.senRdy == TRUE)
					{
						trmt.senRdy = FALSE;
						//send the info stored in buf to the client connected on USART2
						puts2USART(buf);
					}
				}
			}//eo 5 sec interval
			
			
			//display data for every second
			displayData();		
		}//eo 1 sec time interval 
	}//eo while
} // eo main::
