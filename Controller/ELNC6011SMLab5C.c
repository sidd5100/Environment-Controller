/*Use of AI / Cognitive Assistance Software is not allowed in any evaluation, assessment or exercise.*/
/*=============================================================================
	File Name:	ELNC6011SMLab5C.c  
	Author:		Siddhant .K. Mahindrakar
	Date:		07/04/2025
	Modified:	None
	� Fanshawe College, 2025

	Description:	A c-program for pic1845fk22 uC that receives and processes a string from another pic1845fk22 uC connected on USART2.
					It recieves the string byte per byte and stroes inside an array. After the string is fully received it checks whether the received string
					is exactly what we are expecting. Once validated the string is broken into tokens of useful info needed to process the information inside
					the string and then it is used/processed further as needed.
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
#include <string.h>

// Constants  =================================================================
#define TRUE	1	
#define FALSE	0	

#define RC2READ RCREG2
#define RC2FLAG PIR3bits.RC2IF

#define BUFSIZE 35
#define SAMPSIZE 10
#define SENCOUNT 3
#define ONESEC 10
#define FIVESEC 5

#define TEMP 0
#define HUMD 1
#define CO2 2
#define SENSORADDY 977
#define MYADDY 1
#define TOKENSIZE 10


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
	sensor_t avg;
	sensor_t upperLim;
	sensor_t lowerLim;
}sensorCh_t;

//structure for communication objects
typedef struct 
{
	char buf[BUFSIZE];
	char senRdy;
	int insert;	
}serial_t;

//defining struct objects
sensorCh_t sensors[SENCOUNT];
serial_t rcvr;
char* tokens[TOKENSIZE];
char conlim[] = {"CONLIM"};


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
Modified:	07/04/2025
Desc:		Initialises the state of operation for the I/O ports. Each HEX number against particular configuration register defines the mode of operation of the particular I/O port.
Input: 		None 
Returns:	None 
 ============================================================================*/
void configureIOport(void)
{
	ANSELA = 0x00;//input port configured for digital operation 
	LATA = 0x00; // output latched to GND at initialization
	TRISA = 0xFF; // port configured as digital input
	
	ANSELB = 0x00;//input port configured for digital operation
	LATB = 0x00; // output latched to GND at initialization
	TRISB = 0xFF; // port configured as digital input
	
	ANSELC = 0x00;//input port configured for digital operation
	LATC = 0x00; // output latched to GND at initialization
	TRISC = 0xFF; // port configured as digital input
	
	ANSELD = 0x00;//input port configured for digital operation
	LATD = 0x00; // output latched to GND at initialization
	TRISD = 0xFF; // port configured as digital input
	
	ANSELE = 0x00;//input port configured for digital operation
	LATE = 0x00; // output latched to GND at initialization
	TRISE = 0xFF; // port configured as digital input
	
} // configureIOport::

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
	INTCON = 0xC0; //enables all interrupts(hardware, global)
	RCONbits.IPEN = FALSE; //disables interrupt priority
	PIE3bits.RC2IE = TRUE; // enables the EUSART2 receive interrupt
} // eo configIntr::

/*>>> systemINI: ===========================================================
Author:		Siddhant Mahindrakar
Date:		21/01/2025
Modified:	07/04/2025
Desc:		This is a function when called, calls all the configuration functions at once..
Input: 		None 
Returns:	None 
 ============================================================================*/
void systemINI(void)
{
	setOSC4MHz(); // sets the oscillator frequency
	configureIOport(); //initializes I/O port
	serialP1Config(); // initializes serial port for the operation
	serialP2Config(); // initializes serial port2 for the operation
	configIntr(); // initializes interrupt module for the operation
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
	sensorPtr->avg =0;
	sensorPtr->upperLim = highLimit;
	sensorPtr->lowerLim = lowLimit ;
} // eo initChannel::

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

/*>>> valSentence: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/04/2025
Modified:	None
Desc:		This function evaluates the sentence by extracting the checksum value attached to it and comparing by it with the newly generated checksum 
			value at the receiver end. If these value matches resembling the sentence is valid, the function returns TRUE. If not valid returns FALSE.
Input: 		char* ptr: A pointer to the array holding the string to be validated 
Returns:	char: A 1 byte data value either TRUE(if valid) or FALSE(if not valid) (1/0) depending upon the result of validation.
 ============================================================================*/
char valSentence(char* ptr)
{
	char newCS = 0; char rcvdCS = 0;
	int count = 0;
	char csRdy = FALSE;
	count = strlen(ptr);
	
	while(csRdy != TRUE)
	{
		//find for , just before the checksum, make it zero and retrieve the recieved checksum value
		if(*(ptr + count) == ',')
		{
			*(ptr + count) = 0;
			rcvdCS = atoi(ptr+count+1);
			csRdy = TRUE;
		}
		count--;
	}
	newCS = calCheckSum(ptr);
	if(newCS == rcvdCS)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}										
} // eo valSentence::

/*>>> parseStr: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/04/2025
Modified:	None
Desc:		A function which breaks the received string into useful information pieces(tokens) and stores inside an array.
			This array is a pointer array which stores start address of the tokens from the string received in receive buf array.  
Input: 		char* ptr: A pointer to the string out of which useful tokens of information needs to be sorted.
Returns:	None
 ============================================================================*/
void parseStr(char* ptr)
{
	char tokenCounter = 0;
	while(*ptr)
	{
		//find for start character and comma separators
		if(*ptr == '$' || *ptr == ',')
		{
			*ptr = 0;
			//store the address of the next location to start and ',' character 
			tokens[tokenCounter] = ptr+1;
			tokenCounter++;
		}
		ptr++;
	}											
} // eo parseStr::

/*>>> execStr: ===========================================================
Author:		Siddhant Mahindrakar
Date:		07/04/2025
Modified:	None
Desc:		This function performs an operation of assigning the received new limit values from the host controller over USART2 
Input: 		None
Returns:	None
 ============================================================================*/
void execStr(void)
{
	if(!strcmp(conlim,tokens[0]))
	{
		//if mode is true then received new value is increment value
		if(atoi(tokens[4]))
		{
			sensors[atoi(tokens[3])].upperLim = atoi(tokens[5]);
		}
		//else mode is false then received new value is deccrement value
		else
		{
			sensors[atoi(tokens[3])].lowerLim = atoi(tokens[5]);
		} 
	}											
} // eo execStr::

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
	
	printf("\033[2J\033[HController : %s\n", SENSORADDY);
	printf("\n\n\r");
	printf("Temp=%dC\tHumd=%d%%\tCO2=%2dppm", sensors[0].avg, sensors[1].avg, sensors[2].avg); 
	printf("\n\r");
	printf("HL=%2dC\t\tHL=%2d%%\t\tHL=%2dppm\t", sensors[0].upperLim,sensors[1].upperLim,sensors[2].upperLim);
	printf("\n\r");
	printf("LL=%2dC\t\tLL=%2d%%\t\tLL=%2dppm", sensors[0].lowerLim,sensors[1].lowerLim,sensors[2].lowerLim);
	
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
	if(RC2FLAG)
	{
		char hold = 0;
		hold = RC2READ;
		if(hold == '$')
		{
			rcvr.insert = 0;
		}
		if(hold == '\r')
		{
			rcvr.senRdy = TRUE;
			hold = 0;
		}
		rcvr.buf[rcvr.insert] = hold;
		rcvr.insert++;
	}

	INTCON |= 0xC0; //always on global and peripheral interrupts
} //eo isr::

/*=== MAIN: FUNCTION ==========================================================
 ============================================================================*/
void main( void )
{
	int index = 0;
	 
	//initialise all configured system peripherals
	systemINI();
	
	//initialise struct object and its members
	initChannel(&sensors[0], 35,15);
	initChannel(&sensors[1], 65,35);
	initChannel(&sensors[2], 1400,650);
	
	
	while(TRUE)
	{
		//if complete sentence is received do the following
		if(rcvr.senRdy)
		{
			rcvr.senRdy = FALSE;
			//if the sentence is valid do the following
			if(valSentence(rcvr.buf))
			{
				//parse the string and create tokens
				parseStr(rcvr.buf);
				//perform desired operation using tokens (display on USART1 in this case)
				execStr();
				displayData();
			}
		}	
	}//eo while
} // eo main::
