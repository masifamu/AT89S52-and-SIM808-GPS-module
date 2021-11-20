#include <REG052.H>                /* special function register declarations   */
#include <stdio.h>                /* prototype declarations for I/O functions */
#include <stdlib.h>
#include <string.h>

//defining macros
#define HIGH 		1
#define LOW 		0
#define TRUE 		1
#define FALSE 		0
#define START 		1
#define STOP 		0
#define YES 		1
#define NO  		0
#define KEY_STATUS_DETECT_THRESHOLD 0x10
//defining parameter for EEPROM
#define WPP 		P23
#define DATA        P21
#define CLK         P22
//parameters defining multiplexer
#define mul_A       P10
#define mul_B       P11
#define mul_C       P12
//defining the parameters for parallel adc
#define adc_port 	P0             //ADC Port
#define rd 		 	P25                  //Read signal 
#define wr 		 	P26                  //Write signal 
#define chip_s 	 	P24              //Chip Select 
#define intr 	 	P27                //INTR signal 
#define SIMCOM_GET_READY			"AT\r\n"													//exe_fun_id=
#define SIMCOM_SET_FULL_FUN_MODE 	"AT+CFUN=1\r\n"												//exe_fun_id=1 next=2
#define SIMCOM_SET_MIN_FUN_MODE		"AT+CFUN=0\r\n"												//exe_fun_id=42 next=43 will stuck here
#define GSM_SET_CON_TYPE 			"AT+SAPBR=3,1,\"contype\",\"GPRS\"\r\n"						//exe_fun_id=2 next=4
#define GSM_SET_APN 				"AT+SAPBR=3,1,\"APN\",\"airtelgprs.com\"\r\n"				//exe_fun_id=4 next-5
#define GSM_SET_CID_PARA			"AT+HTTPPARA=\"CID\",1\r\n"									//exe_fun_id=7,37
#define GSM_LBS_GET_LOC				"AT+CLBS=1,1\r\n"											//exe_fun_id=29 next=7
#define GSM_LBS_GET_CUST_ID			"AT+CLBSCFG=0,1\r\n"										//exe_fun_id=25 next=26
#define GSM_LBS_HIT_COUNT			"AT+CLBSCFG=0,2\r\n"										//exe_fun_id=26 next=28
#define GSM_LBS_REG_SERVER			"AT+CLBSCFG=1,3,\"lbs-simcom.com:3002\"\r\n"				//exe_fun_id=28 next=3
#define GSM_SET_URL					"AT+HTTPPARA=\"URL\",\"pdpl.in/live/data.php?string=S010"	//exe_fun_id=8 next=9 then 22,38 next=39 then 41 for reset array
#define GPS_LOC_STATUS				"AT+CGPSSTATUS?\r\n"										//exe_fun_id=30 next=19 or 29 or 17,44 next=19 or 29 or 17
#define GPS_TURN_POW_ON				"AT+CGPSPWR=1\r\n"											//exe_fun_id=16 next=44
#define GPS_HOT_RESET				"AT+CGPSRST=1\r\n"											//exe_fun_id=17 next=25
#define GPS_COLD_RESET				"AT+CGPSRST=0\r\n"											//exe_fun_id=17 next=25
#define GPS_GET_LOC					"AT+CGPSINF=0\r\n"											//exe_fun_id=19
#define GSM_TAKE_HTTPACTION			"AT+HTTPACTION=0\r\n"										//exe_fun_id=10,41 to 24 or 10 or 41 or 42
#define GSM_REQ_FOR_IP				"AT+SAPBR=1,1\r\n"											//exe_fun_id=5 next=6
#define GSM_REQ_CARR_IP				"AT+SAPBR=2,1\r\n"											//exe_fun_id=6 next=6 or 5 or 16
#define GSM_TERM_HTTP_CON			"AT+HTTPTERM\r\n"											//exe_fun_id=24
#define GSM_INIT_HTTP_CON			"AT+HTTPINIT\r\n"											//exe_fun_id=3 next=30,36 to 37 to 38 to 39 to 40 to 41
#define GSM_SET_CELL_INFO_MODE		"AT+CENG=1,1\r\n"											//exe_fun_id=33 next=34
#define GSM_GET_CELL_LOC			"AT+CENG?\r\n"												//exe_fun_id=34 next=36 or 35

//parameters defining the Micromonitoring chip function
sbit RESET =P1^3;
bit clk_count;
bit system_overloaded,key_status_checked,first_time_location_check,location_set,is_it_end;
unsigned char *ch_ptr;
unsigned char data_received[28];
unsigned char a1,b1,c1,d1,rec_char,rec_cnt1,rec_cnt2,rec_cnt3,rec_cnt4,rec_cnt5,rec_cnt6,sensed_current,battery_voltage,exe_fun_id;						//will hold the ID of function to be executed. can serve upto 128 functions.
unsigned int data2h1,data2l1,data2h,data2l,timer,overweight_count;
unsigned char write_byte,byte_temp_add,disp_data,data_byte,single_tower_byte_count,eeprom_add,stop_timer_count,key_status,loop_var,count_network,network_number;  //EEPROM variables

/***************************************************************************************************/
void delay_us(unsigned int us_count){  
    while(us_count!=0){
        us_count--;
    }
}
void delay_ms(unsigned int ms_count){
    while(ms_count!=0){
        delay_us(112);   //delay_us is called to generate 1ms delay
        ms_count--;
    }
}
void delay_sec(unsigned char sec_count){
    while(sec_count!=0){
        delay_ms(1000);  //delay_ms is called to generate 1sec delay
        sec_count--;
    }
} 
/************************************************************************************************/
void drive_clk(void){
	clk_count = 1;
	clk_count = 0;
	clk_count = 1;
	clk_count = 0;
	CLK = 1;
	clk_count = 1;
	clk_count = 0;
	clk_count = 1;
	clk_count = 0;
	CLK =0;
	clk_count = 1;
	clk_count = 0;
	clk_count = 1;
	clk_count = 0;
} 
/**********************************************************************************/
void wait1(void){
}
/************************************************************************************/
void wait_ee(void){
	unsigned char delay;
	for(delay = 0;delay <250;delay++);
}  
/************************************************************************************************/
void read_no(unsigned char temp){
	byte_temp_add = temp;
	data_byte=0;
	disp_data=0xA0;
	
	DATA = 1;											     //high to low on data when clk is high is start condition
	CLK = 1;
	wait1();
	DATA = 0; 
	wait1();
	CLK =0;

	DATA = 1;						 						//dummy cycle
	drive_clk();
	DATA = 0;
	drive_clk();
	DATA = 1;
	drive_clk();
	DATA = 0;
	drive_clk();

	if((disp_data&0x08) == 0x08) DATA = 1; else DATA = 0;	//disp_data is Device Address = 0xA0 
	drive_clk();
	if((disp_data&0x04) == 0x04) DATA = 1; else DATA = 0;
	drive_clk();
	if((disp_data&0x02) == 0x02) DATA = 1; else DATA = 0;
	drive_clk();
	DATA = 0;   
	drive_clk();
	DATA = 1;                     							//data 0 to 1 for read
	wait1();
	CLK = 1;  												//acknowledge will make data low
	wait1();
	CLK = 0;
	wait1();
	if((byte_temp_add & 0x80) == 0x80) DATA = 1; else DATA = 0;		 //byte_temp_add is byte address 
	drive_clk();
	if((byte_temp_add & 0x40) == 0x40) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x20) == 0x20) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x10) == 0x10) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x08) == 0x08) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x04) == 0x04) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x02) == 0x02) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x01) == 0x01) DATA = 1; else DATA = 0;
	drive_clk();
	CLK = 1;  			  //ack.
	wait1();
	CLK = 0;
	wait1();
	
	DATA = 1;			 //START
	CLK = 1;
	wait1();
	DATA = 0; 
	wait1();
	CLK =0;
	DATA = 1;         //1010 higher byte 0xA
 	wait1();
 	CLK = 1;
	wait1();
	CLK =0;
	wait1();

	DATA = 0;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	DATA = 1;
	wait1();
 	CLK = 1;
	wait1();
	CLK =0;
	wait1();

	DATA = 0;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if((disp_data&0x08) == 0x08) DATA = 1; else DATA = 0;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if((disp_data&0x04) == 0x04) DATA = 1; else DATA = 0;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if((disp_data&0x02) == 0x02) DATA = 1; else DATA = 0;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	
	DATA = 1;   					   //read
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();

	CLK = 1;		 					 //ack.
	wait1();
	CLK = 0;
	wait1();
	if(DATA == 1) data_byte |= 0x80;      // data is being read here in data_byte variable
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x40;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x20;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x10;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x08;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x04;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x02;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();
	if(DATA == 1) data_byte |= 0x01;
	wait1();
	CLK = 1;
	wait1();
	CLK =0;
	wait1();

	DATA = 0;
	CLK = 1;
	wait1();
	DATA = 1;
	wait1();
}
/************************************************************************************/
void write_ee_address(){
	DATA = 1;
	CLK = 1;
	wait1();
	DATA = 0; 
	wait1();
	CLK =0;
	if((disp_data&0x80) == 0x80) DATA = 1; else DATA = 0;			//writing the control bytes
	drive_clk();
	if((disp_data&0x40) == 0x40) DATA = 1; else DATA = 0;
	drive_clk();
	if((disp_data&0x20) == 0x20) DATA = 1; else DATA = 0;
	drive_clk();
	if((disp_data&0x10) == 0x10) DATA = 1; else DATA = 0;
	drive_clk();
	if((disp_data&0x08) == 0x08) DATA = 1; else DATA = 0;			//three byte are for page number
	drive_clk();
	if((disp_data&0x04) == 0x04) DATA = 1; else DATA = 0;
	drive_clk();
	if((disp_data&0x02) == 0x02) DATA = 1; else DATA = 0;
	drive_clk();
	DATA = 0;   													//write
	drive_clk();
	DATA = 1;
	wait1();
	CLK = 1;  														//ack
	wait1();
	CLK = 0;
	wait1();
	if((byte_temp_add & 0x80) == 0x80) DATA = 1; else DATA = 0;		//writing the byte location address
	drive_clk();
	if((byte_temp_add & 0x40) == 0x40) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x20) == 0x20) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x10) == 0x10) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x08) == 0x08) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x04) == 0x04) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x02) == 0x02) DATA = 1; else DATA = 0;
	drive_clk();
	if((byte_temp_add & 0x01) == 0x01) DATA = 1; else DATA = 0;
	drive_clk();
	CLK = 1;  
	wait1();
	CLK = 0;
	wait1();
}
/*****************************************************************************/
void write_ee_data(void){		
	if((write_byte & 0x80) == 0x80) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x40) == 0x40) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x20) == 0x20) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x10) == 0x10) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x08) == 0x08) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x04) == 0x04) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x02) == 0x02) DATA = 1; else DATA = 0;
	drive_clk();
	if((write_byte & 0x01) == 0x01) DATA = 1; else DATA = 0;
	drive_clk();
	CLK = 1;  
	wait1();
	CLK = 0;
	wait1();
}
/*****************************************************************************/
void write_eeprom3(void){
	write_ee_address(); 
	write_ee_data();
	DATA = 0;
	CLK = 1;
	wait1();
	DATA = 1; 
	wait1();
	wait_ee();	
	wait_ee();	
	wait_ee();	
}
/***************************************************************************/
void write_data(unsigned char temp1,unsigned char temp){
	disp_data=0xA0;				 
	byte_temp_add = temp1;	
	write_byte=temp;
	write_eeprom3();
} 
/***************************************************************************************************/
void bin2ASCII(unsigned char i){
	unsigned char byteN,byteNplusOne;
	byteN=data_received[i];
	data2h1=(byteN&0xF0)>>4;
	data2l1=byteN&0x0F;

	byteNplusOne=data_received[i+1];
	data2h=(byteNplusOne&0xF0)>>4;
	data2l=byteNplusOne&0x0F;

	a1=data2h1|0x30;
	b1=data2l1|0x30;			 
	c1=data2h|0x30;
	d1=data2l|0x30;
}
/**********************************************************************************************************/
void up_22_resetDataReceivedArray(){
	unsigned char loop_var;
	for(loop_var=0;loop_var<28;loop_var++){
		data_received[loop_var]=0x00;
	}
}
/***************************************************************************************************/
void up_initializeCPU(void){   /*initialise CPU */
	//configuring Timer2 for baud rate generation
	SCON=0x50;
	T2CON=0X30;
	RCAP2H=0XFF;
	RCAP2L=0XD9;
	TR2=START;
	//Setting up the timer0
	TMOD=0x01;
	TH0=0x00;
	TL0=0x00;
	TR0=START;
	timer=0;
	IE=0x12;	 /*EA=1,Timer1 and Serial Interupt Enable*/
	//parameter defining the priority of interrupts
	PT0=LOW;		//giving lowest priority to timer 
	//PX0=1;		//giving highest priority to external interrupt
	//parameter for micro chip reset function
	RESET=LOW;
	//parameters for receiving data from the sim808 module
	rec_char=0;rec_cnt1=0;rec_cnt2=0;rec_cnt3=0;rec_cnt4=0;rec_cnt5=0;rec_cnt6=0;
	//initializing the EEPROM parameters
	DATA=1;CLK=1;WPP=0;
	write_byte=0;byte_temp_add=0;disp_data=0;data_byte=0;single_tower_byte_count=0;eeprom_add=0;
	//initializing the multiplexer control pin
	mul_A=0;mul_B=0;mul_C=0;
	//initializing GPS data upload parameters
	data2h=0;data2l=0;data2h1=0;data2l1=0;
	a1=0;b1=0;c1=0;d1=0;
	//initializing the current sensor paramters
	sensed_current=0;key_status=0;battery_voltage=0;
	system_overloaded=NO;
	overweight_count=0;count_network=0;network_number=0;
	is_it_end=NO;
	location_set=NO;
	first_time_location_check=YES;stop_timer_count=0;
	key_status_checked=NO;
	//paramters for function execution in main()
	exe_fun_id=1;
	//initializing the pointer for serial data transmission
	ch_ptr=NULL;
	//Resetting the received array
	up_22_resetDataReceivedArray();
}
/**********************************************************************************************************/
void adc0804_23_getDigitalOutput(unsigned char what_to_read){
	//for starting the conversion
	chip_s = 0;                 //Make CS low
	wr = 0;                  	//Make WR low
	wr = 1;                  	//Make WR high
	chip_s = 1;                 //Make CS high
	while (intr);            	//Wait for INTR to go low

	//for reading the converted data
	chip_s = 0;                 //Make CS low
	rd = 0;                  	//Make RD low
	if(what_to_read=='C') sensed_current = adc_port;  //Read ADC port for sensed current
	if(what_to_read=='K') key_status= adc_port;			//read ADC port for key status voltage
	if(what_to_read=='V') battery_voltage= adc_port;			//read ADC port for total battery voltage
	rd = 1;                  	//Make RD high
	chip_s = 1;                 //Make CS high
}

/***************************************************************************************/
void up_recSerialData (void) interrupt 4 using 1 { 				
	if(RI==1){
		rec_char=SBUF;
		//response receiving of EFID=29
		switch(rec_cnt1){
			case 0: if(rec_char=='+') rec_cnt1++; else rec_cnt1=0; break;
			case 1: if(rec_char=='C') rec_cnt1++; else rec_cnt1=0; break;
			case 2: if(rec_char=='L') rec_cnt1++; else rec_cnt1=0; break;
			case 3: if(rec_char=='B') rec_cnt1++; else rec_cnt1=0; break;
			case 4: if(rec_char=='S') rec_cnt1++; else rec_cnt1=0; break;
			case 5: if(rec_char==':') rec_cnt1++; else rec_cnt1=0; break;
			case 6: if(rec_char==' ') rec_cnt1++; else rec_cnt1=0; break;
			case 7: if(rec_char=='0') rec_cnt1++; else {rec_cnt1=0;exe_fun_id=7;} break;
			case 8: rec_cnt1++; data_received[0]=rec_char; break;
			case 9: rec_cnt1++; data_received[1]=rec_char; break;
			case 10: rec_cnt1++; data_received[2]=rec_char; break;
			case 11: rec_cnt1++; data_received[3]=rec_char; break;
			case 12: rec_cnt1++; data_received[4]=rec_char; break;
			case 13: rec_cnt1++; data_received[5]=rec_char; break;
			case 14: rec_cnt1++; data_received[6]=rec_char; break;
			case 15: rec_cnt1++; data_received[7]=rec_char; break;
			case 16: rec_cnt1++; data_received[8]=rec_char; break;
			case 17: rec_cnt1++; data_received[9]=rec_char; break;
			case 18: rec_cnt1++; data_received[10]=rec_char; break;
			case 19: rec_cnt1++; data_received[11]=rec_char; break;
			case 20: rec_cnt1++; data_received[12]=rec_char; break;
			case 21: rec_cnt1++; data_received[13]=rec_char; break;
			case 22: rec_cnt1++; data_received[14]=rec_char; break;
			case 23: rec_cnt1++; data_received[15]=rec_char; break;
			case 24: rec_cnt1++; data_received[16]=rec_char; break;
			case 25: rec_cnt1++; data_received[17]=rec_char; break;
			case 26: rec_cnt1++; data_received[18]=rec_char; break;
			case 27: rec_cnt1++; data_received[19]=rec_char; break;
			case 28: rec_cnt1++; data_received[20]=rec_char; break;
			case 29: rec_cnt1++; data_received[21]=rec_char; break;
			case 30: rec_cnt1++; data_received[22]=rec_char; break;
			case 31: rec_cnt1++; data_received[23]=rec_char; break;
			case 32: rec_cnt1++; data_received[24]=rec_char; break;
			case 33: rec_cnt1++; data_received[25]=rec_char; break;
			case 34: rec_cnt1++; data_received[26]=rec_char; break;
			case 35: rec_cnt1++; data_received[27]=rec_char; break;
			case 36: rec_cnt1=0; exe_fun_id=7; break;	
			//receiving is complete now.
		}
		//response receiving of EFID=6
		//for resoving the registration on network problem
		switch(rec_cnt2){
			case 0: if(rec_char=='+') rec_cnt2++; else rec_cnt2=0; break;
			case 1: if(rec_char=='S') rec_cnt2++; else rec_cnt2=0; break;
			case 2: if(rec_char=='A') rec_cnt2++; else rec_cnt2=0; break;
			case 3: if(rec_char=='P') rec_cnt2++; else rec_cnt2=0; break;
			case 4: if(rec_char=='B') rec_cnt2++; else rec_cnt2=0; break;
			case 5: if(rec_char=='R') rec_cnt2++; else rec_cnt2=0; break;
			case 6: if(rec_char==':') rec_cnt2++; else rec_cnt2=0; break;
			case 7: if(rec_char==' ') rec_cnt2++; else rec_cnt2=0; break;
			case 8: if(rec_char=='1') rec_cnt2++; else rec_cnt2=0; break;
			case 9: if(rec_char==',') rec_cnt2++; else rec_cnt2=0; break;
			case 10: rec_cnt2=0; if(rec_char=='3') exe_fun_id=5; else exe_fun_id=16; break;
			//receiving is complete now.
		}
		//response receiving of EFID=10
		//for resoving the data upload delay problem
		switch(rec_cnt3){
			case 0: if(rec_char=='+') rec_cnt3++; else rec_cnt3=0; break;
			case 1: if(rec_char=='H') rec_cnt3++; else rec_cnt3=0; break;
			case 2: if(rec_char=='T') rec_cnt3++; else rec_cnt3=0; break;
			case 3: if(rec_char=='T') rec_cnt3++; else rec_cnt3=0; break;
			case 4: if(rec_char=='P') rec_cnt3++; else rec_cnt3=0; break;
			case 5: if(rec_char=='A') rec_cnt3++; else rec_cnt3=0; break;
			case 6: if(rec_char=='C') rec_cnt3++; else rec_cnt3=0; break;
			case 7: if(rec_char=='T') rec_cnt3++; else rec_cnt3=0; break;
			case 8: if(rec_char=='I') rec_cnt3++; else rec_cnt3=0; break;
			case 9: if(rec_char=='O') rec_cnt3++; else rec_cnt3=0; break;
			case 10: if(rec_char=='N') rec_cnt3++; else rec_cnt3=0; break;
			case 11: if(rec_char==':') rec_cnt3++; else rec_cnt3=0; break;
			case 12: if(rec_char==' ') rec_cnt3++; else rec_cnt3=0; break;
			case 13: if(rec_char=='0') rec_cnt3++; else rec_cnt3=0; break;
			case 14: if(rec_char==',') rec_cnt3++; else rec_cnt3=0; break;
			case 15: rec_cnt3=0; 
				if(rec_char=='2'){ 
					exe_fun_id=24;
					stop_timer_count=0;
					if(is_it_end==YES){ exe_fun_id=42; ES=STOP; } //es=0 disables the serial interrupt and 
				}else{
					stop_timer_count++; 
					exe_fun_id=10;
					if(is_it_end==YES) exe_fun_id=41;
					if(stop_timer_count==2){ TR0=STOP; exe_fun_id=100;}
				} break;
			//receiving is complete now.
		}
		//response receiving of EFID=30
		switch(rec_cnt4){
			case 0: if(rec_char=='+') rec_cnt4++; else rec_cnt4=0; break;
			case 1: if(rec_char=='C') rec_cnt4++; else rec_cnt4=0; break;
			case 2: if(rec_char=='G') rec_cnt4++; else rec_cnt4=0; break;
			case 3: if(rec_char=='P') rec_cnt4++; else rec_cnt4=0; break;
			case 4: if(rec_char=='S') rec_cnt4++; else rec_cnt4=0; break;
			case 5: if(rec_char=='S') rec_cnt4++; else rec_cnt4=0; break;
			case 6: if(rec_char=='T') rec_cnt4++; else rec_cnt4=0; break;
			case 7: if(rec_char=='A') rec_cnt4++; else rec_cnt4=0; break;
			case 8: if(rec_char=='T') rec_cnt4++; else rec_cnt4=0; break;
			case 9: if(rec_char=='U') rec_cnt4++; else rec_cnt4=0; break;
			case 10: if(rec_char=='S') rec_cnt4++; else rec_cnt4=0; break;
			case 11: if(rec_char==':') rec_cnt4++; else rec_cnt4=0; break;
			case 12: if(rec_char==' ') rec_cnt4++; else rec_cnt4=0; break;
			case 13: if(rec_char=='L') rec_cnt4++; else rec_cnt4=0; break;
			case 14: if(rec_char=='o') rec_cnt4++; else rec_cnt4=0; break;
			case 15: if(rec_char=='c') rec_cnt4++; else rec_cnt4=0; break;
			case 16: if(rec_char=='a') rec_cnt4++; else rec_cnt4=0; break;
			case 17: if(rec_char=='t') rec_cnt4++; else rec_cnt4=0; break;
			case 18: if(rec_char=='i') rec_cnt4++; else rec_cnt4=0; break;
			case 19: if(rec_char=='o') rec_cnt4++; else rec_cnt4=0; break;
			case 20: if(rec_char=='n') rec_cnt4++; else rec_cnt4=0; break;
			case 21: if(rec_char==' ') rec_cnt4++; else rec_cnt4=0; break;
			case 22: rec_cnt4=0; 
				if(rec_char=='3' || rec_char=='2'){ //location is set
					exe_fun_id=19;
					if(first_time_location_check==YES){ exe_fun_id=17; location_set=YES; first_time_location_check=NO;}
				} else {							//location is not set
					exe_fun_id=29;
					if(first_time_location_check==YES){ exe_fun_id=17; location_set=NO; first_time_location_check=NO;}
				}
				break;
			//receiving is complete now.
		}
		//response receiving of EFID=19
		switch(rec_cnt5){
			case 0: if(rec_char=='+') rec_cnt5++; else rec_cnt5=0; break;
			case 1: if(rec_char=='C') rec_cnt5++; else rec_cnt5=0; break;
			case 2: if(rec_char=='G') rec_cnt5++; else rec_cnt5=0; break;
			case 3: if(rec_char=='P') rec_cnt5++; else rec_cnt5=0; break;
			case 4: if(rec_char=='S') rec_cnt5++; else rec_cnt5=0; break;
			case 5: if(rec_char=='I') rec_cnt5++; else rec_cnt5=0; break;
			case 6: if(rec_char=='N') rec_cnt5++; else rec_cnt5=0; break;
			case 7: if(rec_char=='F') rec_cnt5++; else rec_cnt5=0; break;
			case 8: if(rec_char==':') rec_cnt5++; else rec_cnt5=0; break;
			case 9: rec_cnt5++; data_received[0]=rec_char; break;
			case 10: rec_cnt5++; data_received[1]=rec_char; break;
			case 11: rec_cnt5++; data_received[2]=rec_char; break;
			case 12: rec_cnt5++; data_received[3]=rec_char; break;
			case 13: rec_cnt5++; data_received[4]=rec_char; break;
			case 14: rec_cnt5++; data_received[5]=rec_char; break;
			case 15: rec_cnt5++; data_received[6]=rec_char; break;
			case 16: rec_cnt5++; data_received[7]=rec_char; break;
			case 17: rec_cnt5++; data_received[8]=rec_char; break;
			case 18: rec_cnt5++; data_received[9]=rec_char; break;
			case 19: rec_cnt5++; data_received[10]=rec_char; break;
			case 20: rec_cnt5++; data_received[11]=rec_char; break;
			case 21: rec_cnt5++; data_received[12]=rec_char; break;
			case 22: rec_cnt5++; data_received[13]=rec_char; break;
			case 23: rec_cnt5++; data_received[14]=rec_char; break;
			case 24: rec_cnt5++; data_received[15]=rec_char; break;
			case 25: rec_cnt5++; data_received[16]=rec_char; break;
			case 26: rec_cnt5++; data_received[17]=rec_char; break;
			case 27: rec_cnt5++; data_received[18]=rec_char; break;
			case 28: rec_cnt5++; data_received[19]=rec_char; break;
			case 29: rec_cnt5++; data_received[20]=rec_char; break;
			case 30: rec_cnt5++; data_received[21]=rec_char; break;
			case 31: rec_cnt5++; data_received[22]=rec_char; break;
			case 32: rec_cnt5++; data_received[23]=rec_char; break;
			case 33: rec_cnt5++; data_received[24]=rec_char; break;
			case 34: rec_cnt5++; data_received[25]=rec_char; break;
			case 35: rec_cnt5++; data_received[26]=rec_char; break;
			case 36: rec_cnt5++; data_received[27]=rec_char; break;
			case 37: rec_cnt5=0; exe_fun_id=7; break;	//receiving is complete now.
		}
		//response receiving of EFID=19
		switch(rec_cnt6){
			case 0: if(rec_char=='+') rec_cnt6++; else rec_cnt6=0; break;
			case 1: if(rec_char=='C') rec_cnt6++; else rec_cnt6=0; break;
			case 2: if(rec_char=='E') rec_cnt6++; else rec_cnt6=0; break;
			case 3: if(rec_char=='N') rec_cnt6++; else rec_cnt6=0; break;
			case 4: if(rec_char=='G') rec_cnt6++; else rec_cnt6=0; break;
			case 5: if(rec_char==':') rec_cnt6++; else rec_cnt6=0; break;
			case 6: if(rec_char==' ') rec_cnt6++; else rec_cnt6=0; break;
			case 7: if(rec_char==network_number) rec_cnt6++; else rec_cnt6=0; break;
			case 8: if(rec_char==',') rec_cnt6++; else rec_cnt6=0; break;
			case 9: if(rec_char=='"') rec_cnt6++; else rec_cnt6=0; break;
			case 10:  rec_cnt6++; data_received[0]=rec_char; break;
			case 11: rec_cnt6++; data_received[1]=rec_char; break;
			case 12: rec_cnt6++; data_received[2]=rec_char; break;
			case 13: rec_cnt6++; data_received[3]=rec_char; break;
			case 14: rec_cnt6++; data_received[4]=rec_char; break;
			case 15: rec_cnt6++; data_received[5]=rec_char; break;
			case 16: rec_cnt6++; data_received[6]=rec_char; break;
			case 17: rec_cnt6++; data_received[7]=rec_char; break;
			case 18: rec_cnt6++; data_received[8]=rec_char; break;
			case 19: rec_cnt6++; data_received[9]=rec_char; break;
			case 20: rec_cnt6++; data_received[10]=rec_char; break;
			case 21: rec_cnt6++; data_received[11]=rec_char; break;
			case 22: rec_cnt6++; data_received[12]=rec_char; break;
			case 23: rec_cnt6++; data_received[13]=rec_char; break;
			case 24: rec_cnt6++; data_received[14]=rec_char; break;
			case 25: rec_cnt6++; data_received[15]=rec_char; break;
			case 26: rec_cnt6++; data_received[16]=rec_char; break;
			case 27: rec_cnt6++; data_received[17]=rec_char; break;
			case 28: rec_cnt6++; data_received[18]=rec_char; break;
			case 29: rec_cnt6++; data_received[19]=rec_char; break;
			case 30: rec_cnt6++; data_received[20]=rec_char; break;
			case 31: rec_cnt6++; data_received[21]=rec_char; break;
			case 32: rec_cnt6++; data_received[22]=rec_char; break;
			case 33: rec_cnt6++; data_received[23]=rec_char; break;
			case 34: rec_cnt6++; data_received[24]=rec_char; break;
			case 35: rec_cnt6++; data_received[25]=rec_char; break;
			case 36: rec_cnt6++; data_received[26]=rec_char; break;
			case 37: rec_cnt6++; data_received[27]=rec_char; break;
			case 38: rec_cnt6=0; single_tower_byte_count=0; exe_fun_id=35; break;	//receiving is complete now.
		}
		//do receiving
		RI=0;
	}
}
/**********************************************************************************************************/
void sendSerially(unsigned char ch[]){
	TI=0;
	ch_ptr=ch;
	while((*ch_ptr)!='\0'){
		SBUF = *ch_ptr; while(TI==0);TI=0;
		ch_ptr++;
	}
}
/**********************************************************************************************************/
void selectChannel(unsigned int ch){
	switch(ch){
		case 0: mul_C=0;mul_B=0;mul_A=0; break;
		case 1: mul_C=0;mul_B=0;mul_A=1; break;
		case 2: mul_C=0;mul_B=1;mul_A=0; break;
		case 3: mul_C=0;mul_B=1;mul_A=1; break;
		case 4: mul_C=1;mul_B=0;mul_A=0; break;
		case 5: mul_C=1;mul_B=0;mul_A=1; break;
		case 6: mul_C=1;mul_B=1;mul_A=0; break;
		case 7: mul_C=1;mul_B=1;mul_A=1; break;
	}
}
/**********************************************************************************************************/
void simcom_35_sendSeriallyFromEEPROM(){
	unsigned char loop_var;
	TI=0;
	//saving the tower location on EEPROM
	for(loop_var=0;loop_var<27;loop_var++){
		write_data(eeprom_add,data_received[loop_var]); write_data(0xD0,0X30);
		eeprom_add++;
	}
	//sending the tower location on serial port
	while(single_tower_byte_count != 27){
		data2h1=(data_received[single_tower_byte_count] & 0xF0)>>4;
		data2l1=data_received[single_tower_byte_count] & 0x0F;
		a1=data2h1|0x30;
		b1=data2l1|0x30;	
		SBUF=a1; while(TI==0);TI=0;
		SBUF=b1; while(TI==0);TI=0;
		single_tower_byte_count++;
	}
	single_tower_byte_count=0;
	SBUF=0x0D;while(TI==0);TI=0;
	SBUF=0x0A;while(TI==0);TI=0; 
}
/**********************************************************************************************************/
void simcom_39_uploadCellInfoFromEEPROM(){
	unsigned char loop_var;
	eeprom_add=0;
	for(loop_var=0;loop_var<162;loop_var++){
		read_no(eeprom_add);
		data2h1=(data_byte & 0xF0)>>4;
		data2l1=data_byte & 0x0F;
		a1=data2h1|0x30;
		b1=data2l1|0x30;	
		SBUF=a1; while(TI==0);TI=0;
		SBUF=b1; while(TI==0);TI=0;
		eeprom_add++;
	}
	SBUF='"';while(TI==0);TI=0;
	SBUF=0x0D;while(TI==0);TI=0;
	SBUF=0x0A;while(TI==0);TI=0;
	eeprom_add=0;
}
/**********************************************************************************************************/
void tx_2Bytes(){
	TI=0;
	SBUF=a1;while(TI==0);TI=0;
	SBUF=b1;while(TI==0);TI=0;
	SBUF=c1;while(TI==0);TI=0;
	SBUF=d1;while(TI==0);TI=0;
}
/**********************************************************************************************************/
void simcom_9_CandGPSDataToServer(){
	unsigned char i;
	SBUF='V';while(TI==0);TI=0; 									//overcurrent flag
	SBUF=((battery_voltage & 0xF0)>>4) | 0X30;while(TI==0);TI=0; 
	SBUF=(battery_voltage & 0x0F) | 0X30; while(TI==0);TI=0;
	SBUF='F';while(TI==0);TI=0; 									//overcurrent flag
	SBUF=((system_overloaded & 0xF0)>>4) | 0X30;while(TI==0);TI=0; 
	SBUF=(system_overloaded & 0x0F) | 0X30; while(TI==0);TI=0;
	SBUF='C';while(TI==0);TI=0; 									//current sensor data
	SBUF=((sensed_current & 0xF0)>>4) | 0X30;while(TI==0);TI=0; 
	SBUF=(sensed_current & 0x0F) | 0X30; while(TI==0);TI=0;
	/*******************************************************************************************************/
	SBUF='G';while(TI==0);TI=0; 									//GPS location data
	SBUF='P'; while(TI==0);TI=0;
	SBUF='S'; while(TI==0);TI=0;
	for(i=0;i<28;i=i+2){
		bin2ASCII(i);						
		tx_2Bytes();
	}
	/*******************************************************************************************************/
	SBUF='"';while(TI==0);TI=0; 
	SBUF=0x0D;while(TI==0);TI=0;
	SBUF=0x0A;while(TI==0);TI=0; 
	delay_ms(600);
}
/***************************************************************************************/
void timer0_avoidautoreset(void) interrupt 1 using 2 {
	TR0=STOP;
	TF0=0;
	TH0=0x00;
	TL0=0x00;
	TR0=START;
	timer++;

	//at every around 400msec done will be sent
	if(timer%4==0){	RESET=HIGH; delay_us(1); RESET=LOW;}
	//reading channel 0 for sensed current.
	selectChannel(0);
	adc0804_23_getDigitalOutput('C');
	if(sensed_current < 0xCC){
		//system is underloaded.
		system_overloaded=NO;
		overweight_count=13847;
	}else{
		//system is overloaded.
		system_overloaded=YES;
		overweight_count--;
		//send a beep signal from 0-minutes to 2-minutes
		if(overweight_count <= 13845 && overweight_count >= 12014){
			//send beep signal to the buzzer
		}
		//send a beep signal from 13 minutes to 15 minutes
		if(overweight_count >= 1 && overweight_count <=1832){
			//send a beep signal to the buzzer.
		}
		if(overweight_count==0){
			//send signal to cut the supply power of microprocessor.
		}
	}
	//reading channel 1 for key status.
	selectChannel(1);
	adc0804_23_getDigitalOutput('K');
	//reading channel 1 for key status.
	selectChannel(2);
	adc0804_23_getDigitalOutput('V');
	//if key_status voltage read from ADC is less than 10mv then consider it as key off.
	//if(key_status < KEY_STATUS_DETECT_THRESHOLD && key_status_checked==NO ){ ES=STOP; exe_fun_id=33; key_status_checked=YES; }
	// for resetting the processor on key being turned on.
	//if(key_status > KEY_STATUS_DETECT_THRESHOLD && key_status_checked==YES ) EA=STOP;
} 
/**********************************************************************************************************/
void main (void) {
	//define the variable: only accessible to main function.
	//functions to be executed only once
	up_initializeCPU();
	EA=START;									//enabling the Globlal interrrupt
	sendSerially(SIMCOM_SET_FULL_FUN_MODE);
	delay_sec(20);								//wait for 20 seconds for the GPS module to get ready
	sendSerially(SIMCOM_GET_READY);
	delay_sec(2);									//disabling the serial interrupt so that when key is turned on, responce of httpaction should be received 
	//functions to be executed recurrently
	while(1){
		if(exe_fun_id==1)	{ sendSerially(SIMCOM_SET_FULL_FUN_MODE); 								  delay_ms(600); exe_fun_id=2; 		}
		if(exe_fun_id==2)	{ sendSerially(GSM_SET_CON_TYPE); 		 								  delay_ms(600); exe_fun_id=4; 		}
		if(exe_fun_id==4)	{ sendSerially(GSM_SET_APN); 	 										  delay_ms(600); exe_fun_id=5; 		}
		if(exe_fun_id==30)	{ sendSerially(GPS_LOC_STATUS);															 exe_fun_id=100; 	}
		if(exe_fun_id==29)	{ sendSerially(GSM_LBS_GET_LOC); 														 exe_fun_id=100;	}

		//GPRS setting block-2
		if(exe_fun_id==7)	{ sendSerially(GSM_SET_CID_PARA); 					 					  delay_ms(600); exe_fun_id=8; 		}
		if(exe_fun_id==8)	{ sendSerially(GSM_SET_URL);  															 exe_fun_id=9; 		}
		if(exe_fun_id==9)	{ simcom_9_CandGPSDataToServer(); 														 exe_fun_id=22; 	}
		if(exe_fun_id==22)	{ up_22_resetDataReceivedArray(); 														 exe_fun_id=10; 	}
		if(exe_fun_id==10)	{ sendSerially(GSM_TAKE_HTTPACTION); 													 exe_fun_id=100;	} //No function with id=100 hence program will go in infinite loop

		//one time executable
		if(exe_fun_id==5)	{ sendSerially(GSM_REQ_FOR_IP); 							 			  delay_sec(10); exe_fun_id=6; 		}
		if(exe_fun_id==24)	{ sendSerially(GSM_TERM_HTTP_CON); 							 	 		  delay_ms(600); exe_fun_id=3; 		}
		if(exe_fun_id==3)	{ sendSerially(GSM_INIT_HTTP_CON); 							 	 		  delay_ms(600); exe_fun_id=30;		}
		if(exe_fun_id==25)	{ sendSerially(GSM_LBS_GET_CUST_ID); 						 	 		  delay_ms(600); exe_fun_id=26; 	}
		if(exe_fun_id==26)	{ sendSerially(GSM_LBS_HIT_COUNT); 						 	 			  delay_ms(600); exe_fun_id=28; 	}
		if(exe_fun_id==28)	{ sendSerially(GSM_LBS_REG_SERVER);    									  delay_ms(600); exe_fun_id=3; 		}
		
		//GPS functions
		if(exe_fun_id==16)	{ sendSerially(GPS_TURN_POW_ON); 							 			  delay_ms(600); exe_fun_id=44;  	}
		if(exe_fun_id==44)	{ sendSerially(GPS_LOC_STATUS);															 exe_fun_id=100;	}
		if(exe_fun_id==17)	{ if(location_set == YES) 
										sendSerially(GPS_HOT_RESET); 
									  else 
										sendSerially(GPS_COLD_RESET);
																									  delay_ms(600); exe_fun_id=25; 	}
		if(exe_fun_id==19)	{ sendSerially(GPS_GET_LOC); 															 exe_fun_id=100;	}
		//ip problem resolving
		if(exe_fun_id==6)	{ sendSerially(GSM_REQ_CARR_IP); 							   			  exe_fun_id=6; delay_sec(1); 		}

		//managing the first and last location of cell towers.
		if(exe_fun_id==33)	{ delay_sec(5); sendSerially(GSM_SET_CELL_INFO_MODE); 			 delay_ms(600); ES=START; exe_fun_id=34; 	}
		if(exe_fun_id==34)	{ 
			if(count_network<6){
				switch(count_network){
					case 0: network_number='0'; break;
					case 1: network_number='1'; break;
					case 2: network_number='2'; break;
					case 3: network_number='3'; break;
					case 4: network_number='4'; break;
					case 5: network_number='5'; break;
				} 	
				count_network++;
				delay_sec(10);
				sendSerially(GSM_GET_CELL_LOC);
				exe_fun_id=100;
			} else{ exe_fun_id=36; count_network=0; } 													
		}
		if(exe_fun_id==35)	{ simcom_35_sendSeriallyFromEEPROM();	 					 	 			delay_sec(10); exe_fun_id=34; 	}
		if(exe_fun_id==36)	{ sendSerially(GSM_INIT_HTTP_CON); 							 	 			delay_ms(600); exe_fun_id=37; 	}
		if(exe_fun_id==37)	{ sendSerially(GSM_SET_CID_PARA); 					 						delay_ms(600); exe_fun_id=38; 	}
		if(exe_fun_id==38)	{ sendSerially(GSM_SET_URL);  															   exe_fun_id=39; 	}
		if(exe_fun_id==39)	{ simcom_39_uploadCellInfoFromEEPROM();	 					 	 			delay_ms(600); exe_fun_id=40; 	}
		if(exe_fun_id==40)	{ up_22_resetDataReceivedArray(); 														   exe_fun_id=41; 	}
		if(exe_fun_id==41)	{ sendSerially(GSM_TAKE_HTTPACTION); is_it_end=YES;										   exe_fun_id=100;	}
		//setting the module in minimum functionality mode.
		if(exe_fun_id==42)	{ sendSerially(SIMCOM_SET_MIN_FUN_MODE);								 	delay_sec(5);  exe_fun_id=43; 	}	//EA=0 to stop everything and to generate up reset signal from microchip

	}//while(1)	
}//main	