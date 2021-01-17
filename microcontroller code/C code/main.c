//Liam Strijckers 400179278
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

uint16_t	dev=0x52;

int status=0;

volatile int IntCount;

//device in interrupt mode (GPIO1 pin signal)
#define isInterrupt 1 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

void PortM_Init(void){
	//Use PortNM as an input sign for the motor to turn on
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				// activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	// allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0b00000000;        								// make PM0 input
  GPIO_PORTM_DEN_R |= 0b00000001;        								// enable digital I/O on PM0		
	return;
}
void PortF_Init(void){
	//Use PortF0 onboard LED	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;				// activate clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};	// allow time for clock to stabilize
	GPIO_PORTF_DIR_R |= 0b00000001;        								// make PF0 output (PF0 built-in LED)
  GPIO_PORTF_DEN_R |= 0b00000001;        								// enable digital I/O on PF0
	return;
}
void PortE_Init(void){
	//Use PortE for motor	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;				// activate clock for Port E
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){};	// allow time for clock to stabilize
	GPIO_PORTE_DIR_R |= 0b00011111;        								// make PE0 to PE3 an output
  GPIO_PORTE_DEN_R |= 0b00011111;        								// enable digital I/O on PE0 to PE3
	return;
}
void PortL_Init(void){
	//Use PortL on as output to off board led	
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;				// activate clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};	// allow time for clock to stabilize
	GPIO_PORTL_DIR_R |= 0b000001000;        								// make PL3 an output
  GPIO_PORTL_DEN_R |= 0b000001000;        								// enable digital I/O on PL3
	return;
}
void motor(int angle, int rate){
   //int motorturn[4]= {0b0011,0b0110,0b1100,0b1001}; //put all motor values in an array
	 int motorturn[4]={0b1100,0b0110,0b0011,0b1001};
   int num=0,point= 0; 
	 while(1){
		 if(num==4){//breaks out of while loop after 4 steps
			break;
		}if(num%angle==0){//checks if the num as meet the required turn angle
			GPIO_PORTF_DATA_R ^= 0b00000001;//turns on led
			SysTick_Wait10ms(1);
    }else{//else
			GPIO_PORTF_DATA_R=GPIO_PORTF_DATA_R&0b11111110;//keeps the led off
			}SysTick_Wait10ms(rate);
			GPIO_PORTE_DATA_R = motorturn[point]; //sends the output the 
			num=(num+1);//increatments the num
			point=num%4;//changes the index of the motorturn array
		}   
}
void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//capture values from VL53L1X for inspection
uint16_t debugArray[100];

int main(void) {
	//initialize
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
																				
	PortM_Init();
	PortE_Init();
	PortF_Init();
	PortL_Init();
	int round=0;
	while(1){
		if((GPIO_PORTM_DATA_R&0b00000001) == 0b0000000){//checks so if the button has been pressed
			GPIO_PORTL_DATA_R = 0b000000000;//or else checks the off board led on
			// hello world!
			//UART_printf("Program Begins\r\n");
			int mynumber = 1;
			sprintf(printf_buffer,"2DX4 Program Studio Code %d\r\n",mynumber);
			//UART_printf(printf_buffer);


		  /* Those basic I2C read functions can be used to check your own I2C functions */
			status = VL53L1_RdByte(dev, 0x010F, &byteData);					// This is the model ID.  Expected returned value is 0xEA
			myByteArray[i++] = byteData;

			status = VL53L1_RdByte(dev, 0x0110, &byteData);					// This is the module type.  Expected returned value is 0xCC
			myByteArray[i++] = byteData;
			
			status = VL53L1_RdWord(dev, 0x010F, &wordData);
			status = VL53L1X_GetSensorId(dev, &wordData);

			sprintf(printf_buffer,"Model_ID=0x%x , Module_Type=0x%x\r\n",myByteArray[0],myByteArray[1]);
			//UART_printf(printf_buffer);

			// Booting ToF chip
			while(sensorState==0){
				status = VL53L1X_BootState(dev, &sensorState);
				SysTick_Wait10ms(10);
			}
			FlashAllLEDs();
			
			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
			/* This function must to be called to initialize the sensor with the default setting  */
			status = VL53L1X_SensorInit(dev);
			Status_Check("SensorInit", status);

			status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
			Status_Check("StartRanging", status);
			GPIO_PORTL_DATA_R = 0b00000000;//turns off the off board led
			int temp=1;
			GPIO_PORTL_DATA_R = 0b000001000;//turns the off board led on
			while(temp){
				if((GPIO_PORTM_DATA_R&0b00000001) == 0b0000000){
					GPIO_PORTL_DATA_R = 0b00000000;//turns off the off board led
					SysTick_Wait10ms(50);
					for(int i=0;i<512;i++){
						motor(360,1);//calls the motor function to turn the stepper motor
							for(int i = 0; i < 1; i++) {
						  //while(1){ /* read and display data */
								if((GPIO_PORTM_DATA_R&0b00000001) == 0b0000000){//checks to see if the button has be push to stop the distance measurements
									GPIO_PORTL_DATA_R = 0b000001000;//turns the off board LED on
									SysTick_Wait10ms(100);
									while((GPIO_PORTM_DATA_R&0b00000001) == 0b0000001){//runs a wait until the button is pressed again
										SysTick_Wait10ms(1);
									}
								}
								GPIO_PORTL_DATA_R = 0b000000000;//turns the off board LED if it was on due to a button press
								while (dataReady == 0){//checks to see if the ToF sensor is ready
									status = VL53L1X_CheckForDataReady(dev, &dataReady);
											FlashLED3(1);
											VL53L1_WaitMs(dev, 5);
								}
								dataReady = 0;
								status = VL53L1X_GetRangeStatus(dev, &RangeStatus);//gets the distance measurements
								status = VL53L1X_GetDistance(dev, &Distance);
									FlashLED4(1);

									debugArray[i] = Distance;
								status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
									sprintf(printf_buffer,"%u\r\n", Distance);
									UART_printf(printf_buffer);//sends the data over UART to the PC
								
								SysTick_Wait10ms(1);
								round++;//used to break out of the loop once the the 360 degree rotation is done
							}
					}
			}
			else{SysTick_Wait10ms(1);
				if(round>=1){
					GPIO_PORTL_DATA_R = 0b000001000;//since the ToF sensor and motor are no longer running
					//the off board LED is turned on to signal the measurments for that plane are done
				}
			}
		}
	}
		else{
			GPIO_PORTL_DATA_R = 0b000001000;//else  the off board LED is on
			GPIO_PORTF_DATA_R = 0b00000000;//keeps the on board LED off
		}
	}
}


#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
    GPIO_PORTB_AFSEL_R |= 0x0C;           // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             // 4) enable open drain on PB3 only
    GPIO_PORTB_DEN_R |= 0x0C;             // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          // 7) disable analog functionality on PB2,3
// 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                                        // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        // 8) configure for 100 kbps clock
}


//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
    // configure PG0 as GPIO
		//GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
		GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
    return;
}

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}


