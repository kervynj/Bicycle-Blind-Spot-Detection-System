//-------------------------------------------------------------------------------------//
//	ENGR 359 Term Project															   //
//	Bicycle Blind Spot Detection System using Ultrasonic sensor and Tiva C Launchpad   //
//																					   //
//	Written By:	Mitchell Johnston													   //
//				Trevor Gordon														   //
//				Jonathon May														   //
//																					   //
//	March 15/2016																	   //
//-------------------------------------------------------------------------------------//
#include <stdint.h>
#include <stdbool.h>
#include "stdlib.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.c"
#include <string.h>

const double temp = 1.0/80.0;

const double G_slope = -77.0/150.0;//-78.0/75.0;
const double R_slope = 77.0/150.0;//78.0/75.0;
const double G_intercept = 157;//183;
const double R_intercept = 0.49; //-103

volatile uint8_t echowait = 0;
volatile uint32_t pulse = 0;
uint32_t Period, DutyCycle;
int32_t Duty_Red,Duty_Green;

int i;
int j;
int const sample_size = 8;

uint32_t count = 0;
uint32_t pulse_history[sample_size];
uint32_t dist_avg =0;

void PINconfig();
void SonicIntHandler();
void InitConsole(void);


int main(void) {

	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	Period = SysCtlClockGet()/1000000;	// 80e6/1000000 = 80 clock cycles per Period
	DutyCycle = 0.5*Period;

	PINconfig();
	InitConsole();

	//Configure Timer for tracking echo pulse
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	SysCtlDelay(3);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
	TimerEnable(TIMER2_BASE,TIMER_A);


	while(1)
	{
		//Checks if a pulse read is in progress
		 if(echowait != 1)
		 {
		 //Does the required pulse of 10uS
		 GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
		 SysCtlDelay(266);
		 GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);

		  /*
		 This makes the code wait for a reading to finish
		 You can omit this part if you want the code to be non-blocking but
		 reading is only ready when echowait=0.
		 */
		 while(echowait != 0);
		 {
			 count += 1;


			 //UARTprintf("%2d \n" , (dist_avg));

			 //Converts the counter value to cm
			 pulse =(uint32_t)(temp * pulse);
			 pulse = pulse / 58;
			 //UARTprintf("%2d \n" , (pulse));


			 // Store Value in pulse history array based on FIFO ordering
			 for (j = 1; j < (sample_size+1);j++)
			 {
				 if (count %j ==0)
					 pulse_history[(j-1)] = pulse;
			 }

			 // Sum Values in pulse history
			 for (i =0; i <sample_size; i++)
			 {
				 dist_avg += pulse_history[i];
			 }

			 // Calculate Average by dividing sum by number of data points in array
			 dist_avg = (uint32_t)dist_avg/sample_size;

			 UARTprintf("%2d \n" , (dist_avg));

			 // Check distance conditions to specify LED settings
			 // If object detected between 1 and 1.75 meters, illuminate Yellow LED with intensity proportional to distance
			 if (dist_avg < 300 && dist_avg >= 150)
			 {
				 Duty_Red = R_slope*pulse + R_intercept;
				 Duty_Green = G_slope*pulse + G_intercept;

				 //UARTprintf("%2d \n" , (Duty_Red));

				 TimerMatchSet(TIMER0_BASE, TIMER_A, Duty_Red);
				 TimerMatchSet(TIMER0_BASE, TIMER_B, Duty_Green);
				 SysCtlDelay(80); // Delay for one period

			 }
			 // If Hazard is 1.5 meters, illuminate red LED
			 else if (dist_avg >0 && dist_avg < 150)
			 {

				 TimerMatchSet(TIMER0_BASE, TIMER_A, 1);
				 TimerMatchSet(TIMER0_BASE, TIMER_B, Period-2);
				 SysCtlDelay(80); // Delay for one period

			 }
			 else if (dist_avg >300)
			 {

				 TimerMatchSet(TIMER0_BASE, TIMER_A, Period-2);
				 TimerMatchSet(TIMER0_BASE, TIMER_B, 1);
				 SysCtlDelay(80); // Delay for one period
			 }
		 }

		SysCtlDelay(400000);

		}
	}

	return;
}

void PINconfig()
{
	// Configure Port A pin 3 as Trigger output
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(3);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

	// Configure Port A pin 2 as Echo input
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

	//Enable portA pin 2 for interrupt
	GPIOIntEnable(GPIO_PORTA_BASE,GPIO_PIN_2);
	// Initialize interupt to occur on Pin 2 corresponding to both rising and falling edges
	GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_2,GPIO_BOTH_EDGES);
	// Create Prototype to inform processor of function to call when interrupt detected
	GPIOIntRegister(GPIO_PORTA_BASE,SonicIntHandler);

	// Configure RED LED on PB4 as Timer Base 1, Timer A (T1CCP0)
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	 SysCtlDelay(3);
	 GPIOPinConfigure(GPIO_PB4_T1CCP0);
	 GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_4);

	 SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	 SysCtlDelay(3);
	 TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM);
	 TimerLoadSet(TIMER1_BASE, TIMER_A, Period -1);
	 TimerMatchSet(TIMER1_BASE, TIMER_A, Period*0.5);

	 TimerEnable(TIMER1_BASE, TIMER_A);

	 // Configure GREEN LED on PB7 as Timer as Timer Base 0, Timer B (T0CCP1)
	 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	 SysCtlDelay(3);
	 GPIOPinConfigure(GPIO_PB7_T0CCP1);
	 GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_7);

	 SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	 SysCtlDelay(3);
	 TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
	 TimerLoadSet(TIMER0_BASE, TIMER_B, Period -1);
	 TimerMatchSet(TIMER0_BASE, TIMER_B, 1);

	 TimerEnable(TIMER0_BASE, TIMER_B);
}


void SonicIntHandler()
{
	//clear interrupt flag
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);

	// Read echo to determine if interrupt occured due to rising or falling edge
	if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2)
	{
		// Set Clock to 0
		HWREG(TIMER2_BASE + TIMER_O_TAV) = 0;
		// Begin Timer
		TimerEnable(TIMER2_BASE, TIMER_A);
		echowait = 1;
	}

	else
	{
		// Falling Edge Detected - Stop and Record pulse time
		pulse = TimerValueGet(TIMER2_BASE,TIMER_A);
		TimerDisable(TIMER2_BASE,TIMER_A);
		// Set Echowait to 0
		echowait=0;
	}
}

//			 if (count %3 ==0)
//			 {
//				 pulse_history[2] = pulse;
//			 }
//			 else if(count %2 == 0)
//			 {
//				 pulse_history[1] = pulse;
//			 }
//			 else if(count %1 == 0)
//			 {
//				 pulse_history[0] = pulse;
//			 }


void InitConsole(void)
{
 //
 // Enable GPIO port A which is used for UART0 pins.
 // TODO: change this to whichever GPIO port you are using.
 //
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
 SysCtlDelay(3);
 //
 // Configure the pin muxing for UART0 functions on port A0 and A1.
 // This step is not necessary if your part does not support pin muxing.
 // TODO: change this to select the port/pin you are using.
 //
 GPIOPinConfigure(GPIO_PA0_U0RX);
 GPIOPinConfigure(GPIO_PA1_U0TX);
 //
 // Enable UART0 so that we can configure the clock.
 //
 SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
 //
 // Use the internal 16MHz oscillator as the UART clock source.
 //
 UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
 //
 // Select the alternate (UART) function for these pins.
 // TODO: change this to select the port/pin you are using.
 //
 GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
 //
 // Initialize the UART for console I/O.
 //
 UARTStdioConfig(0, 460800, 16000000);
}
