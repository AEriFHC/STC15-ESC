#include<reg51.h>

#define DisableFilter
#define DisableEVLD
//Define Start or Start_Simple to active
#define Start

sbit CurrentSense = P3^0;
sbit Output_C     = P3^1;  //Output pin mode will not be open drain unless output needs to be floated
sbit PhaseChange  = P3^2;
sbit Input        = P3^3;  //Weak pull up mode, must be 1 in any time otherwise can't receive signal
sbit Output_A     = P3^4;  //High side MOSFET will turn on when the output is low in both push pull and open drain mode
sbit Output_B     = P3^5;  //Low side MOSFET wilL turn on when the output is high in push pull mode
sbit ELVD         = IE^6; 

sfr P3M0      = 0xB2;  // Push Pull: 1  Open Drain : 1   High Resistance : 0
sfr P3M1      = 0xB1;  //            0               1                     1
sfr AUXR      = 0x8E;  //B7:T0x12(0:12T,1:1T) T1x12 UART T2R T2_C/T(0:Timer) T2x12 EXTRAM SIST1; 0x80(T2 Disable) 0x90(T2 Enable) 0x94(T2 1T)
sfr AUXR2     = 0x8F;  //B6:EX4 0x00(INT4 Disable) 0x40(INT4 Enable)  interrupt16
sfr IE2       = 0xAF;  //B3:ET2 0x00(ET2 Disable)   0x04(ET2 Enable)  interrupt12
sfr TL2       = 0xD7;
sfr TH2       = 0xD6;  //Different from 8052
sfr IAP_DATA  = 0xC2;
sfr IAP_ADDRH = 0xC3;
sfr IAP_ADDRL = 0xC4;
sfr IAP_CMD   = 0xC5;
sfr IAP_TRIG  = 0xC6;
sfr IAP_CONTR = 0xC7;
sfr WDT_CONTR = 0xC1;  //Enable:0x23 Initiate:0x03 Clear: 0x13 Can't be shut down by software

volatile unsigned char Pin, PinMode, Phase = 0 ,I_C = 0, T_IC = 0, P3_Tem, T2_Int0_F;  //T_on:Period of time which output is high
//Pin,PinMode:pins will be operated in PWM  I_C:Record how many times current reach the limit(30A)
volatile unsigned int  T2_Int1_C, T2_PL_C, T_on = 65296, T_off = 65296, T_f = 0, D_Max = 4000, D_Min = 2000, DutyCount;
volatile unsigned int  T_on_c = 65296, T_off_c = 65296, T_f_c = 0;//For calculation
volatile unsigned int	T_PC1 = 65000, T_PC2 = 65000, T_PC3 = 65000, T_PCA = 65000, T_PCW = 10922;
/*  T_PC:Register's value in timer, indicate the lenth of time which one phase take */
volatile unsigned long D_A, DutyCycle;
volatile bit flag = 0, ERROR_Flag = 0, T2OF1 = 0, T2OF2 = 0, PWM_EN1 = 1, PWM_EN2 = 1, PWM_SQ1 = 0, PWM_SQ2 = 0, PWM_EN1_c = 1, PWM_EN2_c = 1;

unsigned char EEPROM_Read(unsigned int addr);
unsigned char EEPROM_Erase(unsigned int addr);
unsigned char EEPROM_Write(unsigned char dat, unsigned int addr);
void EEPROM_Disable();
void Sound(unsigned char Time_500mS, unsigned char Note);

void main()
{
	P3M0      = 0x32;             //xx11 0010
	P3M1      = 0x37;             //xx11 0111
	P3        = 0x3A;             //xx11 1010 
	TMOD      = 0x00;             //T0 Activate As 16 Bit Auto Load Timer
	TL0       = 0x00;
	TH0       = 0x00;             
	AUXR      = 0x80;             //T0 Speed:1T T2 Speed:12T
	TL2       = 0x00;
	TH2       = 0x00;             //32.7675mS
	WDT_CONTR = 0x07;             //Overflow time:4.1943S 
	IAP_CONTR = 0x00;
	IT0       = 0;                //INT0 Interrupt Mode
	IT1       = 0;
	EA        = 1;                //Enable all Interrupt
	ET0       = 1;
	TR0       = 0;
	AUXR2     = 0x40;             //INT4 Enable
	IE2       = 0x04;						  //ET2 Enable
	EX0       = 0;  
	EX1       = 0;                //Input signal
  Phase     = 0;               	//Initialization

	if(EEPROM_Read(0x0000) == 0x01 && EEPROM_Read(0x0003) == 0x02)
	{
		D_Max  = EEPROM_Read(0x0001)<<8;    //Maximum value of duty cycle, exact time can be calculated by multiply 0.5uS
		D_Max |= EEPROM_Read(0x0002);       //  32768 16384 8192 4096,2048 1024 512 256,128 64 32 16 ,8 4 2 1
		D_Min  = EEPROM_Read(0x0004)<<8;    //  4096                  256               16            1
		D_Min |= EEPROM_Read(0x0005);
	}
	if(D_Max <= 3600 || D_Max >= 4400 || D_Min <= 1600 || D_Min >= 2400)        
	{
		D_Max = 4000;                         //Default value, means 2mS
		D_Min = 2000;
		EEPROM_Erase(0x0000);
	}

	EX0 = 1;
	AUXR = 0x90;
	while(!T2OF2);   //Wait 33mS
	AUXR = 0x80;
	T2OF1 = 0;
	T2OF2 = 0;
	TL2 = 0;
	TH2 = 0;
	EX0 = 0;

	if(Phase <= 16 || Phase >= 90)          //If phase has changed during start-up process, means motor is rotating and following steps shouldn't be executed
	{
		Sound(1, 1);                          //C5 last for 0.5S
		Sound(1, 2);                          //D5
		Sound(1, 3);                          //E5
		
		AUXR = 0x90;
		EX1  = 1;
		for(Pin = 0; Pin <= 20 ; Pin++)   //Wait about 0.6S to decide whether to reset D_Max and D_Min
		{
			while(!T2OF2);
			T2OF2 = 0;
		}
		T2OF1 = 0;
		EX1  = 0;
		AUXR = 0x80;
		if(DutyCount >= 3600 && DutyCount <= 4400) //Input signal bigger than usual 
		{
			EEPROM_Erase(0x0000);
			EEPROM_Write(0x01, 0x0000);
			EEPROM_Write(DutyCount>>8, 0x0001);
			EEPROM_Write(DutyCount, 0x0002);
			if(ERROR_Flag == 1)
			{
				IAP_CONTR = 0x20;   //RESET
			}
			Sound(2, 1);        //D_Max set successfully
			
			WDT_CONTR = 0x17;
			AUXR = 0x90;
			EX1 = 1;
			for(Pin = 0; Pin <= 60 ; Pin++)   //Wait about 2.4S to recive D_Min value
			{
				while(!T2OF2);
				T2OF2 = 0;
			}	
			T2OF1 = 0;
			EX1 = 0;
			AUXR = 0x80;
			if(DutyCount <=2400 && DutyCount >= 1600)
			{
				EEPROM_Write(0x02, 0x0003);
				EEPROM_Write(DutyCount>>8, 0x0004);
				EEPROM_Write(DutyCount, 0x0005);
				if(ERROR_Flag == 1)
				{
					IAP_CONTR = 0x20;   
				}
				Sound(2, 2);   
			}
		}
	}    
  
	AUXR = 0x90;  				//T2 enable
	EX1 = 1;
	Phase = 0;
	flag = 0;
	D_A  = D_Max - D_Min;
	PinMode = 0x15;   		//CB xx01 0101
	Pin = 0x02;       		//C  xx00 0010
	WDT_CONTR = 0x23;     //Overflow time: 284.4mS
	ELVD = 0;
	
	while(1) 
	{
		//DutyCycle = 888;
		//DutyCycle = DutyCount > (D_Min + 20) ? (DutyCount - D_Min) : 0;
		if(DutyCount > (D_Min + 20))
		{
			if(DutyCycle + D_Min > DutyCount + 10)  //Lower than the threshold
			{
				DutyCycle -= 10;  //Limit the derivative
			}
			else if(DutyCycle + D_Min < DutyCount - 10)  //Higher than the threshold
			{
				DutyCycle += 10;
			}
		}
		else
		{
			DutyCycle = 0;
		}
		WDT_CONTR = 0x13;
		
		if(DutyCycle)     //Process will be started if D > 1%
		{
			if(!flag)       //And maintain >1% for more than 33mS;
			{
				TR0 = 0;
				flag = 1;
				for(P3_Tem = 0; P3_Tem < 20; P3_Tem++) // Wait 33*20mS
				{
					T2OF2 = 0;                
					while(!T2OF2);
					WDT_CONTR = 0x13;
					if(DutyCount < D_Min + 20)
					{
						flag = 0;
						break;
					}
				}
				#ifdef Start
				if(flag) //Start 
				{
					EX0 = 0;
					P3M1  = 0x07; //AB P3:XX54 3210
					Output_A = 1; //        BA   C
					Output_B = 0;
					Output_C = 1;
					Pin = TH2;
					while(TH2 - Pin < 40);
					for(P3_Tem = 0; P3_Tem < 12; P3_Tem++)
					{
						for(Phase = 0; Phase < 12; Phase++)
						{
							switch(Phase)
							{
								case  0:P3M1  = 0x07; //AB P3:XX54 3210
												Output_A = 1; //        BA   C
												Output_B = 0;
												Output_C = 1;
												break;
								case  1:Output_A = 0;
												Output_B = 0;
											  break;
								case  2:P3M1  = 0x25; //AC
												Output_A = 1;
												Output_B = 1;
												Output_C = 0;
											  break;
								case  3:Output_A = 0;
												Output_C = 0; 
											  break;	
								case  4:P3M1  = 0x15; //BC
												Output_A = 1;
												Output_B = 1;
												Output_C = 0; 
											  break;
								case  5:Output_C = 0;
												Output_B = 0;
											  break;
								case  6:P3M1  = 0x07; //BA
												Output_A = 0;
												Output_B = 1;
												Output_C = 1;
												break;												
								case  7:Output_A = 0;
												Output_B = 0;
											  break;				
								case  8:P3M1  = 0x25; //CA
												Output_A = 0;
												Output_B = 1;
												Output_C = 1;
											  break;				
								case  9:Output_A = 0;
												Output_C = 0;
											  break;				
								case 10:P3M1  = 0x15; //CB 
												Output_A = 1;
												Output_B = 0;
												Output_C = 1; 
											  break;				
								case 11:Output_C = 0;
												Output_B = 0;
											  break;
								default:IAP_CONTR |= 0x10;//Reset
							}
							Pin = TH2;
							while(TH2 - Pin < 30 - P3_Tem - Phase); //4*256*0.5 = 0.512mS
						}
						WDT_CONTR = 0x13;
					}
					P3M1 = 0x37;
					P3  |= 0x32;
					Phase = 0;
					Pin = 0x10;
					DutyCycle = 200;
					T_on = 65536 - 240;
					T_off = 65536 - 240;
					T_f = 63616;
					TR0 = 1;
					goto MAIN;
				}
				#elif defined Start_Simple
				DutyCycle = 2000;
				#endif
			}
			
			else
	  	{
				#ifdef Start
				MAIN:
				#endif
			flag = 0;
			
			if(2 * DutyCycle >= D_A)        //D >= 0.5
			{
				T_on_c = (12 * 100 * DutyCycle) / D_A;
				if(T_on_c < 12 * 96)
				{
					T_off_c = 12 * 100 - T_on_c;
					T_off_c = 65536 - T_off_c;
					PWM_EN1_c = 1;
				}
				else
				{
					T_off_c = 65535;
					PWM_EN1_c = 0;
				}
				T_f_c = 65535;
				PWM_EN2_c = 0;
				T_on_c = 65536 - T_on_c;
			}
			
			else if(5 * DutyCycle >= D_A)  //0.2 <= D <= 0.5 
			{
				T_on_c = (12 * 100 * DutyCycle) / D_A;
				T_off_c = T_on_c;
				PWM_EN1_c = 1;
				if((T_on_c + T_off_c) < 12 * 96)
				{
					T_f_c = 12 * 100 - 2 * T_on_c;
					T_f_c = 65536 - T_f_c;
					PWM_EN2_c = 1;
				}
				else
				{
					T_f_c = 65535;
					PWM_EN2_c = 0;
				}
				T_on_c = 65536 - T_on_c;
				T_off_c = 65536 - T_off_c;	
			}
			
			else                          //D < 0.2
			{
				T_on_c = 65536 - 12 * 20;     //Value directly load into timer 0's Register
				T_off_c = 65536 - 12 * 20;    //PFM: Minimum turn on time:10uS
				T_f_c = 65536 - (12 * 20 * D_A) / DutyCycle + 12 * 2 * 20;
				PWM_EN1_c = 1;
				PWM_EN2_c = 1;
			}
			TR0 = 0;//Update dutycycle
			P3M1 = 0x37;
			P3  |= 0x32;
			T_on = T_on_c;
			T_off = T_off_c;
			T_f = T_f_c;
			PWM_EN1 = PWM_EN1_c;
			PWM_EN2 = PWM_EN2_c;
			PWM_SQ1 = 0;
			PWM_SQ2 = 0;
			TL0  = T_on; 
			TH0  = T_on>>8;
			TR0 = 1;      //PWM on
			T2_PL_C  = TH2<<8;  //Record timer 2's Register value 
			T2_PL_C |= TL2;
			T2OF2 = 0;
			
			WDT_CONTR = 0x13;
			
			T_PC1  = T_PC2;
			T_PC2  = T_PC3;
			T_PC3  = T_PCA;
			T_PCA  = T_PC1 / 3;
			T_PCA += T_PC2 / 3;
			T_PCA += T_PC3 / 3;
			T_PCW  = T_PCA / 2;
		
			do                                   //30degree
			{
				T_PCA  = TH2<<8;
				T_PCA |= TL2;
				T_PCA -= T2_PL_C;                  //Length of this step
				if(T2OF2)
				{
					T_PCA--;
				}
			}
			while(T_PCA <= T_PCW);      
					
			
			if(Phase >= 6)                        //252 % 6 = 0
			{
				Phase = 0;
			}
			
			switch(Phase)           //Phase commutation
			{
				case 0: PinMode = 0x07;   //AB xx00 0111
								Pin = 0x10;       //A  xx01 0000
								break;			
					
				case 1: PinMode = 0x25;   //AC xx10 0101
								Pin = 0x10;       //A  xx01 0000
								break;
								
				case 2: PinMode = 0x15;   //BC xx01 0101
								Pin = 0x20;       //B  xx10 0000
								break;				
													
				case 3: PinMode = 0x07;   //BA xx00 0111
								Pin = 0x20;       //B  xx10 0000
								break;						
								
				case 4: PinMode = 0x25;   //CA xx10 0101
								Pin = 0x02;       //C  xx00 0010
								break;
								
				case 5: PinMode = 0x15;   //CB xx01 0101
								Pin = 0x02;       //C  xx00 0010
								break;
			}

			EX0 = 1;
			T_PC1 = 400 * D_A / DutyCycle; //0.2mS/phase
			T_PC1 = T_PC1 < 4000 ? T_PC1 : 4000; //Reuse T_PC1
			while(!flag)               
			{
				T_PCA  = TH2<<8;
				T_PCA |= TL2;
				T_PCA -= T2_PL_C;                  //Lenth of this step
				if(T2OF2)
				{
					T_PCA--;
				}
				if(T_PCA >= T_PC1 && !flag)                 //Enforce phase communication
				{
					EX0  = 0;
					flag = 1;
					Phase++;
				}
			}
			EX0 = 0;
			
			WDT_CONTR = 0x13;
		}
	}
		else
		{
			flag = 0;
			TR0  = 0;     //Turn PWM off
			EX0  = 0;
			P3M1 = 0x37;  //xx11 0111
			P3  |= 0x32;  //0011 0010
		}
		
		WDT_CONTR = 0x13;
	}
}

unsigned char EEPROM_Read(unsigned int addr)
{
	unsigned char dat;
	IAP_CONTR = 0x81;
	IAP_CMD = 1;
	IAP_ADDRL = addr;
	IAP_ADDRH = addr>>8;
	IAP_TRIG = 0x5A;
	IAP_TRIG = 0xA5;
	if(IAP_CONTR & 0x10) //Fail to operate
	{
		ERROR_Flag = 1;
	}
	dat = IAP_DATA;
	EEPROM_Disable();
	return dat;
}

unsigned char EEPROM_Erase(unsigned int addr)
{
	IAP_CONTR = 0x81;
	IAP_CMD = 3;
	IAP_ADDRL = addr;
	IAP_ADDRH = addr>>8;
	IAP_TRIG = 0x5A;
	IAP_TRIG = 0xA5;
	if(IAP_CONTR & 0x10)
	{
		ERROR_Flag = 1;
		EEPROM_Disable();
		return 0x00;
	}
	EEPROM_Disable();
	return 0xFF;
	
}

unsigned char EEPROM_Write(unsigned char dat,unsigned int addr)
{
	IAP_CONTR = 0x81;
	IAP_CMD = 2;
	IAP_ADDRL = addr;
	IAP_ADDRH = addr>>8;
	IAP_DATA = dat;
	IAP_TRIG = 0x5A;
	IAP_TRIG = 0xA5;
	if(IAP_CONTR & 0x10)
	{
		ERROR_Flag = 1;
		EEPROM_Disable();
		return 0x00;
	}
	EEPROM_Disable();
	return 0xFF;

}

void EEPROM_Disable()
{
	IAP_CONTR = 0x00;
	IAP_CMD = 0;
	IAP_ADDRL = 0;
	IAP_ADDRH = 0x80;
	IAP_TRIG = 0;
}

void Sound(unsigned char Time_500mS, unsigned char Note)
{
	unsigned int c, j, k;       
	P3M1 = 0x25;             //Pin:C A, push pull mode 
	
	switch(Note)
	{
		case 1:		k = 65536 - 1912;            //C5
							c = Time_500mS * 523/5;      //100mS
							break;
		
		case 2:		k = 65536 - 1702;            //D5
							c = Time_500mS * 587/5;
							break;
		
		case 3:		k = 65535 - 1516;            //E5
							c = Time_500mS * 659/5;
							break;
	}

	TH2 = k>>8;
	TL2 = k;
	AUXR = 0x90;  
	Output_B = 1;
	Output_C = 0;
	for(j = 0; j <= c ; j++)
	{
		while(!T2OF2);
		Output_B = !Output_B;
		T2OF1 = 0;
		T2OF2 = 0;
		Output_C = !Output_C;
	}
	
	AUXR = 0x80;
	TH2 = 0;
	TL2 = 0;
	P3M1 = 0x37;
	Output_B = 1;
	Output_C = 1;
}

void OutputPWM_Control() interrupt 1   //SQ: 00, 01, 10  ON OFF FLOAT 
{
	EX0  = 1;
	P3M1 = 0x37;
	P3  |= 0x32;               //Prevent two MOSFETs turn on at the same time
  if(!PWM_SQ1 && !PWM_SQ2)   //Step 0:Output high
	{
		if(PWM_EN1)              //To step 1
		{
			PWM_SQ1 = 1;
			PWM_SQ2 = 0;
			TL0  = T_off;
			TH0  = T_off>>8;
		}
		else
		{
			PWM_SQ1 = 0;
			PWM_SQ2 = 0;
			TL0  = T_on; 
			TH0  = T_on>>8;
		}
		P3M1 = PinMode;         //Two pin needs to be in push pull mode
		P3  &= ~Pin;            //Turn high side MOSFET on
	}                         //E.G. 0011 0010 &= ~0001 0000 means turn phase A's high side MOSFET on

  else if(PWM_SQ1 && !PWM_SQ2) //Step1:Output low
	{
		EX0  = 0;               	 //Disable zero-detection during this step
		if(PWM_EN2)
		{
			PWM_SQ1 = 0;
			PWM_SQ2 = 1;
			TL0  = T_f;
			TH0  = T_f>>8;
		}
		else
		{
			PWM_SQ1 = 0;
			PWM_SQ2 = 0;
			TL0  = T_on; 
			TH0  = T_on>>8;
		}
		
		P3M1 = PinMode;            //Turn low side MOSFET on
	}
	
	else if(!PWM_SQ1 && PWM_SQ2) //Step 2:Float
	{
		PWM_SQ1 = 0;
		PWM_SQ2 = 0;
		TL0  = T_on; 
		TH0  = T_on>>8;
		P3  |= 0x32;
	}

}

void PhaseAndInputSignal_Control() interrupt 12
{
	//if(T2OF1 == 1)        //Lose signal
	{
		if(DutyCycle && !flag)
		{
			DutyCount--;
		}
		else
		{
			TR0 = 0;
			WDT_CONTR = 0x0B;  
			T2OF1 = 0;
		}
	}
	T2OF1 = 1;
	T2OF2 = 1;
}

void Phase_Commutation() interrupt 0
{
	#ifndef DisableFilter
	T2_Int0_F -= TH2;
	if(T2_Int0_F <= 1024/256 && !flag) // 0.512mS/phase
	{
		Phase++;                                //AB(0) - AC - BC - BA - CA - CB(5) 
		flag = 1;
	}
	T2_Int0_F = TH2;                 //Software filter
	#else
	Phase++;
	flag = 1;
	#endif
}

void I_PeakLmit() interrupt 16
{
	ET0 = 0;
	P3_Tem = P3;
	T_IC = TL2;
	P3 |= 0x32;                           //Turn all push pull mode pin to low side on
	while(TL2 - T_IC < 100);              //50uS
	P3 = P3_Tem;
	I_C++;
	ET0 = 1;
}

void InputSignal() interrupt 2
{
	if(Input == 1)
	{
		T2_Int1_C  = TH2<<8;
		T2_Int1_C |= TL2;
		T2OF1 = 0;
	}
	else
	{
		DutyCount  = TH2<<8;
		DutyCount |= TL2;
		DutyCount -= T2_Int1_C;
		if(T2OF1)
		{
			DutyCount--;
		}
	}
}

void LowVoltage() interrupt 6 //Must not be empty otherwise somehow the output function can't work properly
{
	#ifndef DisableEVLD
	P3M1 = 0x37;
	P3  |= 0x32;
	#endif
}
