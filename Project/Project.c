
/*
*Minesweeper: eYRTC:#51
*Actual Implementation may wary depending on User and Environmental Conditions
*The bot needs chilled Budweiser from time to time
*/



//****************************This is basic Initialization*******************************************************
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning
void follow();
//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
unsigned char left_ir,front_ir,right_ir;
unsigned char b=0x64;
unsigned char c;
unsigned char r;
unsigned char l;

void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char left_black_line  = 0;
unsigned char Center_black_line  = 0;
unsigned char right_black_line  = 0;

//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	buzzer_pin_config();
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

void turn_on_ir_proxi_sensors (void) //turn on IR Proximity sensors
{
	PORTH = PORTH & 0xF7;
}

void turn_off_ir_proxi_sensors (void) //turn off IR Proximity sensors
{
	PORTH = PORTH | 0x08;
}

//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	reverse();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibble for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibble to 0
 PortARestore |= Direction; // adding lower nibble for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);
}

void stop (void)
{
  motion_set (0x00);
}


void right(void)
{
	motion_set(0x05);
}

void left(void)
{
	motion_set(0x0A);
}
void reverse(void)
{
	motion_set(0x09);
}	

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}

void leftturn(void){
	left();
	velocity(0,50);
}
void lcd_showvalue(unsigned char num)
{
	unsigned char H=0,T=0,O=0;
	H=num/100;
	T=(num - (H*100))/10;
	O=(num - (H*100) - (T*10));
	
	lcd_wr_char(H+48);
	lcd_wr_char(T+48);
	lcd_wr_char(O+48);
}

int pos=11;

//**************************************This is basic Line following implementation********************************
void follow()
{
			    DDRJ=0xff;
			    PORTJ=0x00;
				init_devices();
				int flag=0;
				int k=0;
	while(1)
	{

    	left_black_line  = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_black_line  = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_black_line  = ADC_Conversion(1);	//Getting data of Right WL Sensor
		
		    unsigned char b=80;
		    unsigned char c=Center_black_line;
		    unsigned char r=right_black_line;
		    unsigned char l=left_black_line;


		//int flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		if(c>=b && r>=b && l>=b)
		{
			stop();
			_delay_ms(1000); //do not remove			
			forward_mm(15);
			if(c<b)
			
			forward();
			//velocity(200,200);
			break;
		}			
	   if (l>=b&&c>=b&&r<b)
	    {
		  right_degrees(4);
		  flag=1;
		  forward();
		  velocity(200,200);
		  }
	   if (l>=b&&c<b&&r<b)
		  {
			 right_degrees(9);
			 flag=1;
			 forward();
			 velocity(200,200);
		  }		 
	  if(l<b&&c>=b&&r>=b)
		  { 
		    left_degrees(4);
			flag=2;
			forward();
			velocity(200,200);
	      }
	  if(l<b&&c>=b&&r<b)
		  {
		forward();
		velocity(200,200);
		  }
	if(l<b&&c<b&&r>=b)
	     {
		left_degrees(9);
		flag=2;
		forward();
		velocity(200,200);
	     }
		 if((r<b&&l<b&&c<b)||(r>=b&&c<b&&l>=b))
		 {
			 if(flag==1)
			 {
				 left_degrees(4);
				 forward();
				 velocity(200,200);
				 flag=0;
			 }
			 else if(flag==2)
			 {
				 right_degrees(4);
				 forward();
				 velocity(200,200);
				 flag=0;
			 }
			 else 
			 {
				
				if(k<8) 
				{
				left_degrees(5);
				k++;
				}			 
			 else
				 right_degrees(5);
			 }			 
		 }
		 			 
					
	}	  	  	

 }



int orie; //Keeps track of Firebird V's current orientation i.e North,East,South or West
int pointingnorth=1;
int pointingeast=2;
int pointingsouth=3;
int pointingwest=4;
int nnorth,neast,nwest,nsouth;
int traverse[50];	//Stack for holding Traverse node
int priority[50][2]; //Stack for holding neighboring nodes and it's priority
int obstacle[6];	//Stack for holding mine location
int trtop=-1;
int obtop=-1;
int prtop=-1;
int n;

//Function to travel in north direction
void north(unsigned int n)
{
	init_devices();
	if(n==pointingnorth)
	{
		
	   follow();
	}
	else if(n==pointingeast)
	{
		forward_mm(25);
		right_degrees(88);
		follow();
	}
	else if(n==pointingwest)
	{
		forward_mm(25);
		left_degrees(88);
		follow();
	}
	orie=pointingnorth;
}

//Function to travel in south direction
void south(unsigned int n)
{
	init_devices();
	if(n==pointingsouth)
	{
		follow();
	}
	else if(n==pointingeast)
	{
		forward_mm(25);
		left_degrees(88);
		follow();
	}
	else if(n==pointingwest)
	{
		forward_mm(25);
		right_degrees(88);
		follow();
	}
	orie=pointingsouth;
}

//Function to travel in west direction

void west(unsigned int n)
{
	init_devices();
	if(n==pointingwest)
	{
		follow();
	}
	else if(n==pointingnorth)
	{
		forward_mm(25);
		right_degrees(88);
		follow();
	}
	else if(n==pointingsouth)
	{
		
			forward_mm(25);
			left_degrees(88);
			follow();				
	}
	orie=pointingwest;
}

//Function to travel in east direction

void east(unsigned int n)
{
	init_devices();
	if(n==pointingeast)
	{
	   follow();
	}
	else if(n==pointingnorth)
	{
		forward_mm(25);
		left_degrees(88);
		follow();
	}
	else if(n==pointingsouth)
	{
		forward_mm(25);
		right_degrees(88);
		follow();
	}
	orie=pointingeast;
}

//function to get neighbours of a node
void neighbour(int x)
{
	if(x>=12&&x<=16)
	{
		nnorth=0;
		neast=x+1;
		nwest=x-1;
		nsouth=x+10;
	}
	else if(x>=72&&x<=76)
	{
		nnorth=x-10;
		neast=x+1;
		nwest=x-1;
		nsouth=0;
	}
	else if(x==21||x==31||x==41||x==51||x==61)
	{
		nnorth=x-10;
		neast=x+1;
		nwest=0;
		nsouth=x+10;
	}
	else if(x==27||x==37||x==47||x==57||x==67)
	{
		nnorth=x-10;
		neast=0;
		nwest=x-1;
		nsouth=x+10;
	}
	else if(x==11||x==17||x==71||x==77)
	{
		switch(x)
		{
			case 11:
			nnorth=0;nwest=0;neast=x+1;nsouth=x+10;
			break;
			case 17:
			nnorth=0;nwest=x-1;neast=0;nsouth=x+10;
			break;
			case 71:
			nnorth=x-10;nwest=0;neast=x+1;nsouth=0;
			break;
			case 77:
			nnorth=x-10;nwest=x-1;neast=0;nsouth=0;
			break;
			
		}
	}
	else
	{
		nnorth=x-10;
		neast=x+1;
		nwest=x-1;
		nsouth=x+10;
	}
}

//This function help in travelling from position 1 i.e pos1 to positon 2 i.e pos 2

func(int pos1,int pos2)
{
	if(pos1!=pos2)
	{
		int x1=pos1/10;
		int y1=pos1%10;
		int x2=pos2/10;
		int y2=pos2%10;
		
		if(x1>x2)
		while(x1!=x2)
		{
			north(orie);
			x1--;
		}
		if(x2>x1)
		while(x2!=x1)
		{
			south(orie);
			x1++;
		}
		if(y1>y2)
		while(y1!=y2)
		{
			west(orie);
			y1--;
		}
		if(y2>y1)
		while(y1!=y2)
		{
			east(orie);
			y1++;
		}
	}
	
}

//Helps in searching for a given value in priority stack
int peak(int y)
{
	int found=0;
	for(int i=0;i<=prtop;i++)
	{
		if(y==priority[i][0])
		{found=1;
		break;}
	}
	if(found)
	return 1;
	else return 0;
}
 
 
//Detects mine adjacent to current node
 
void mine_detection()
{
	init_devices();
	
    stop();
	_delay_ms(1000);
	left_ir=ADC_Conversion(4);
	front_ir=ADC_Conversion(6);
	right_ir=ADC_Conversion(8);
  if(orie==pointingeast)
  {
	  
	  if((left_ir<150)&&(nnorth!=0))
	  {
		  if(!peak(nnorth))
		  {
		  prtop++;
		  priority[prtop][0]=nnorth;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nnorth;
		  }			    
	  }	  
	  else if((front_ir<150)&&(neast!=0))
	  {

		  if(!peak(neast))
		  {		//delay
		  prtop++;
		  priority[prtop][0]=neast;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=neast;
		  }			  	  
	  }
	  else if((right_ir<153)&&(nsouth!=0))
	   {
		   if(!peak(nsouth))
		   {
		   prtop++;
		   priority[prtop][0]=nsouth;
		   priority[prtop][1]=0;
		   obstacle[++obtop]=nsouth;
		   }
		   
        }
	   }		
  else if(orie==pointingsouth)
  {
	  
	  if((left_ir<150)&&(neast!=0)&&(!peak(neast)))
	  {
		  prtop++;
		  priority[prtop][0]=neast;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=neast;
	  }
	  else if((front_ir<150)&&(nsouth!=0)&&(!peak(nsouth)))
	  {
		  prtop++;
		  priority[prtop][0]=nsouth;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nsouth;
	  }
	  else if((right_ir<153)&&(nwest!=0)&&(!peak(nwest)))
	  {
		  prtop++;
		  priority[prtop][0]=nwest;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nwest;
	  }
	  
  }
  else if(orie==pointingwest)
  {
	  
	  if((left_ir<150)&&(nsouth!=0)&&(!peak(nsouth)))
	  {
		  prtop++;
		  priority[prtop][0]=nsouth;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nsouth;
	  }
	  else if((front_ir<150)&&(nwest!=0)&&(!peak(nwest)))
	  {
		  prtop++;
		  priority[prtop][0]=nwest;
		  obstacle[++obtop]=nwest;
	  }
	  else if((right_ir<153)&&(nnorth!=0)&&(!peak(nnorth)))
	  {
		  prtop++;
		  priority[prtop][0]=nnorth;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nnorth;
	  }
	  
  }
  else if(orie==pointingnorth)
  {
	  
	  if((left_ir<150)&&(nwest!=0)&&(!peak(nwest)))
	  {
		  prtop++;
		  priority[prtop][0]=nwest;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nwest;
	  }
	  else if((front_ir<150)&&(nnorth!=0)&&(!peak(nnorth)))
	  {
		  prtop++;
		  priority[prtop][0]=nnorth;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=nnorth;
	  }
	  else if((right_ir<153)&&(neast!=0)&&(!peak(neast)))
	  {
		  prtop++;
		  priority[prtop][0]=neast;
		  priority[prtop][1]=0;
		  obstacle[++obtop]=neast;
	  }
	  
  }
}

//Reverses orientation on a 180 degree turn 


void oriechange()
{
	if(orie==pointingeast)
	orie=pointingwest;
	else if(orie==pointingwest)
	orie=pointingeast;
	else if(orie==pointingnorth)
	orie=pointingsouth;
	else if(orie==pointingsouth)
	orie=pointingnorth;
}
			
 
 int previous=0;
 int seast,swest,ssouth,snorth;
 
 //Checks if valid neighbors exist for current node
 int special()
 {
	 lcd_wr_command(0xc0);
	 lcd_showvalue((char)pos);
	 neighbour(pos);
	 for(int i=prtop;i>=0;i--)
	 {
		 if((priority[i][0]==neast)&&(priority[i][1]==1))
		 {
			seast=1;
			return i;
			break;
		 }			
		 else if((priority[i][0]==nnorth)&&(priority[i][1]==1))
		 {
			 snorth=1;
			 return i;
			 break;
		 }			 
		 else if((priority[i][0]==nsouth)&&(priority[i][1]==1))
		 {
		     ssouth=1;
			 return i;
			 break;
		 }			 
		 else if((priority[i][0]==nwest)&&(priority[i][1]==1))
		 {
		     swest=1;
			 return i;
			 break;
		 }
		 }		  
			 return 0;			 			 	 		 	 
 }
 
 
 //Function for making 180 degree turn depending on flag1 value
 int flag1=0;
 void fakebt()
 {
	 if(flag1==1)
	 {
	 forward_mm(15);
	 left_degrees(179);
	 oriechange();
	 follow();
	 }	 
 }	 
 
 //Function for checking value in traverse stack
 int trcheck(int x)
 {
	 for(int i=0;i<=trtop;i++)
	 if(traverse[i]==x)
	 {
	 return i;
	 break;
	 }
	 return 0; //beware
 }
 
 
 //Function for displaying mine position
 void display_mines()
 {
	 lcd_init();
	 int c=0;int c1=0;
	 for(int i=0;i<=obtop;i++)
	 {
		 int x=obstacle[i]/10;
		 int y=obstacle[i]%10;
		 if(c1<3)
		 {
			 lcd_cursor(1,++c);
			 lcd_wr_char(x+48);
			 lcd_cursor(1,++c);
			 lcd_wr_char(y+64);
			 lcd_cursor(1,++c);
			 lcd_wr_char(' ');
		 }
		 else
		 {
			 if(c1==3)
			 c=0;
			 lcd_cursor(2,++c);
			 lcd_wr_char(x+48);
			 lcd_cursor(2,++c);
			 lcd_wr_char(y+64);
			 lcd_cursor(2,++c);
			 lcd_wr_char(' ');
		 }
		 c1++;			 
	 }
	 for(int i=obtop;i>=0;i--)
	 {
		 buzzer_on();
		 _delay_ms(500);
		 buzzer_off();
		 _delay_ms(500);
	 }
	 
 }	 
 
 
 //Function for reaching final position (7G)
 int a[8][8]={0};
 void finish(){
	 lcd_init();
	 for(int i=1;i<8;i++)
		for(int j=1;j<8;j++)
			if(trcheck(i*10+j))
			a[i][j]=1;
	 	int position=traverse[trtop];
		 int pose1=position;
		 pos=pose1;
		 int pose2=0;
		 lcd_showvalue(position);
		 _delay_ms(1000);
		 int x=position/10;
		 int y=position%10;
	 while(x<7||y<7)
	 {
		 if(a[x][y+1]==1)
		 {
			 pose2=((x)*10)+y+1;
			 func(pose1,pose2);
			 pos=pose1=pose2;
			 a[x][y]=0;
			 y++;
		 }
		 else if(a[x+1][y]==1)
		 {
			 pose2=((x+1)*10)+y;
			 func(pose1,pose2);
			 pos=pose1=pose2;
			 a[x][y]=0;
			 x++;
		 }
		 else if(a[x][y-1]==1)
		 {
			 pose2=(x*10)+y-1;
			 func(pose1,pose2);
			 pos=pose1=pose2;
			 a[x][y]=0;
			 y--;
		 }
		 else if(a[x-1][y]==1)
		 {
			 pose2=((x-1)*10)+y;
			 func(pose1,pose2);
			 pos=pose1=pose2;
			 a[x][y]=0;
			 x--;
		 }
		 stop();
		 lcd_init();
		 lcd_cursor(1,1);
		 lcd_wr_char(x+48);
		 lcd_cursor(2,1);
		 lcd_wr_char(y+48);
		 forward();
		 velocity(200,200);
				 			 
	 }		 
 }
 
 
int main(void)
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	int pos1,pos2,error;
	orie=pointingeast;
	lcd_string("START");
	_delay_ms(1000);
	lcd_init();
	while(1)
	{
		if ((error==2)&&(previous==prtop)&&(special()==0))
		{
			fakebt();
			pos2=traverse[trcheck(pos)-1];
			if(flag1>1)
			func(pos,pos2);
			pos=pos2;
			error=1;
		}			
		    else if((previous==prtop)&&(special()>0)&&(error>=1))
		{	
			if(flag1>=1)
			flag1=0;
			neighbour(pos);
			if(seast==1)
			{
			pos2=neast;
			priority[special()][1]=0;
			func(pos,pos2);
			pos=pos2;
			seast=0;
			}			
			else if(ssouth==1)
			{
			pos2=nsouth;
			priority[special()][1]=0;
			func(pos,pos2);
			pos=pos2;		
			ssouth=0;
			}			
			else if(swest==1)
			{
			pos2=nwest;
			priority[special()][1]=0;
			func(pos,pos2);
			pos=pos2;
			swest=0;
			}			
			else if(snorth==1)
			{		
			pos2=nnorth;
			priority[special()][1]=0;
			func(pos,pos2);                                                                                                                       
			pos=pos2;
			snorth=0;
			}
			error=0;			
		}
		else 
		{			
		neighbour(pos);
		turn_on_ir_proxi_sensors();
		mine_detection();
		turn_off_ir_proxi_sensors();
		traverse[++trtop]=pos;
		if(flag1>=1)
		flag=0;
		switch(0)
		{
			case 0:
			if(nnorth!=0&&(!peak(nnorth)))
			{
				++prtop;
				priority[prtop][0]=nnorth;
				priority[prtop][1]=1;
			}
			case 1:
			if(nwest!=0&&(!peak(nwest)))
			{
				++prtop;
				priority[prtop][0]=nwest;
				priority[prtop][1]=1;
			}
			case 2:
			if(nsouth!=0&&(!peak(nsouth)))
			{
				++prtop;                                         
				priority[prtop][0]=nsouth;
				priority[prtop][1]=1;
			}
			case 3:
			if(neast!=0&&(!peak(neast)))
			{
				++prtop;
				priority[prtop][0]=neast;
				priority[prtop][1]=1;
			}
		}
		if (priority[prtop][1]==1)
		pos2=priority[prtop][0];
		else
		{
			error=2;
			flag1++;			
		}
		
		pos1=traverse[trtop];
		func(pos1,pos2);
		pos=pos2;
		priority[prtop][1]=0;
		previous=prtop;
		
	}
	if(trtop>=45)
	{
		stop();
		finish();
		stop();	
		display_mines();	
	    break;
		
	}	
  }				
}