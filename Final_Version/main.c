/*
 * Final_Version.c
 *
 * Created: 4/24/2017 8:03:53 AM
 * Author : Yuanyu Huang, Guangwei Chen
 */ 

#define F_CPU 8000000
#define BAUD 31250
#define MYUBRR F_CPU/16/BAUD - 1
#define Sensor_Amount 7
 //for testing

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//Instrument Button
int pre_TIMBRE_flag = 1;
int led_flag = 0;
int change_instrument_flag = 0;
int Instrument_Type = 0;

//Next button
int pre_NEXT_flag = 1;

//OK button
int pre_OK_flag = 1;

//Mode button
int pre_MODE_flag = 1;

//Laser Decoder
int beam = 0;
int temp_beam = 0;  //only for debugging
int beam_flag0 = 0;
int beam_flag1 = 0;
int beam_flag2 = 0;
int beam_flag3 = 0;
int beam_flag4 = 0;
int beam_flag5 = 0;
int beam_flag6 = 0;
int half_second_counter = 0; //count 500 for 500ms
int time_mark0 = 0;
int time_mark1 = 0;
int time_mark2 = 0;
int time_mark3 = 0;
int time_mark4 = 0;
int time_mark5 = 0;
int time_mark6 = 0;

int free_mode_flag = 1;
int laser_off = 0;
int laser_on = 1;

//ADC and IR sensor detection
int result_array[7] = {0};
int sorted_array[7] = {0};
int compare_value = 0;
int adc_channel = 0;
int adc_sensitive_detect[7] = {0};
int adc_buffer[7] = {0};
int handflag_buffer[7] = {0};
int prehandflag_buffer[7] = {0};

unsigned char note_buffer[21] = {0x48, 0x4a, 0x4c, 0x4d, 0x4f, 0x51, 0x53, 0x3c, 0x3e, 0x40, 0x41, 0x43, 0x45, 0x47, 0x30, 0x32, 0x34, 0x35, 0x37, 0x39, 0x3b}; //HIGH,MID,LOW: CDEFGAB
const char Grand_Piano[6] = {0xC0, 0x01, 0xC1, 0x01, 0xC2, 0x01};
const char Alto_Sax[6] = {0xC0, 0x41, 0xC1, 0x41, 0xC2, 0x41};
const char Guitar[6] = {0xC0, 0x19, 0xC1, 0x19, 0xC2, 0x19};

int tim_cnt = 0;
int adc_flag = 0;
int check = 0;
int cur_music_ok = 0;
int cur_music = 0;

//PB MARCRO
#define PB_MODE 4
#define PB_NEXT 3
#define PB_OK 2
#define PB_TIMBRE 1
#define PB_INITILAZATION() (DDRA &= 0xE1)

// LCD MACRO
#define DataBus PORTC
#define Data_IO DDRC
#define ControlBus PORTA
#define Control_IO DDRA

#define Enable 7
#define RW 6
#define RS 5

#define CLEAR 0x01
#define HOME 0x02
#define ENTRY 0x06
#define FUNCTION 0x38
#define ALL_ON 0x0E
#define FIRSTLINE 0x80
#define SECONDLINE 0xC0
#define THIRDLINE 0x94
#define FOURTHLINE 0xD4

//Timer MACRO
#define TIMER0_START() (TCCR0A |= 0x05)//CS02 CS00
#define TIMER0_STOP() (TCCR0A &= 0xF8)

//SD card (SPI) MACRO
#define SS (1<<PB0)
#define SCK (1<<PB1)
#define MOSI (1<<PB2)
#define MISO (1<<PB3)
#define SPI_DDR DDRB

#define SS_DISABLE() (PORTB |= SS)
#define SS_ENABLE() (PORTB &= ~SS)

#define CMD0 0
#define CMD8 8
#define CMD12 12
#define CMD17 17
#define CMD55 55
#define ACMD41 41

//SD card (SPI) VARIABLES
unsigned char sd_response[6] = {0};
unsigned char sd_addr[4] = {0};
unsigned char sd_data[512] = {0};
	
//MIDI VARIABLES
#define MUSIC_NUM 3
#define INSTR_NUM 8
unsigned char cur_note = 0x00;
unsigned char time_buff[60] = {0};
unsigned char index_buff[170] = {0};
unsigned char music_buff[225] = {0};

unsigned short time_buff_ptr = 0x0000;
unsigned short index_buff_ptr = 0x0000;

unsigned char music1[] = "   London Bridge";
unsigned char music2[] = "   The Dawn  ";
unsigned char music3[] = "   Hail Purdue  ";

int timeout[5] = {0};//SD timeout
uint16_t midi_cnt = 0;//midi resolution counter

//TWI VARIABLES
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

#define DDR_USI             DDRE
#define PORT_USI            PORTE
#define PIN_USI             PINE
#define PORT_USI_SDA        PINE5
#define PORT_USI_SCL        PINE4
#define PIN_USI_SDA         PINE5
#define PIN_USI_SCL         PINE4

#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT  0       // Bit position for (N)ACK bit.

unsigned char ACK_Rev = 0xAA;
unsigned char tempUSISR_8bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
(0x0<<USICNT0);                                     // set USI to shift 8 bits i.e. count 16 clock edges.
unsigned char tempUSISR_1bit = (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Prepare register value to: Clear flags, and
(0xE<<USICNT0);                                     // set USI to shift 1 bit i.e. count 2 clock edges.

/*************************
TWI FUNCTIONS
these functions control TWI communication
**************************/
 /***************************************************
 Read Operation of STC3100
 -----------------------------------------------------------------------------------
   <1>       <2>          <3>        <4>       <5>           <6>                 <7>
 |Start|Device Addr|W|A|Reg Addr|A|Restart|Device Addr|R|A|Reg Data|A|Reg Data|_|Stop|
 |     |  7 Bits   | | | 8 Bits | |       |   7 Bits  | | |  8 Bits| |  8 Bits|A|    |
 -----------------------------------------------------------------------------------
 ^                    ^                                                      ^
 |                    |                                                      |
 Master               Slave ACK                                          Master NACK
 ****************************************************/
 void USI_TWI_Master_Initialise()
 {
	 PORT_USI |= (1<<PIN_USI_SDA);           // Enable pullup on SDA, to set high as released state.
	 PORT_USI |= (1<<PIN_USI_SCL);           // Enable pullup on SCL, to set high as released state.
	 
	 DDR_USI  |= (1<<PIN_USI_SCL);           // Enable SCL as output.
	 DDR_USI  |= (1<<PIN_USI_SDA);           // Enable SDA as output.
	 
	 USIDR    =  0xFF;                       // Preload dataregister with "released level" data.
	 USICR    =  (0<<USISIE)|(0<<USIOIE)|                            // Disable Interrupts.
	 (1<<USIWM1)|(0<<USIWM0)|                            // Set USI in Two-wire mode.
	 (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|                // Software stobe as counter clock source
	 (0<<USITC);
	 USISR   =   (1<<USISIF)|(1<<USIOIF)|(1<<USIPF)|(1<<USIDC)|      // Clear flags,
	 (0x0<<USICNT0);                                     // and reset counter.
 }

 unsigned char USI_TWI_Master_Transfer( unsigned char temp )
 {
	 USISR = temp;                                     // Set USISR according to temp.
	 // Prepare clocking.
	 temp  =  (0<<USISIE)|(0<<USIOIE)|                 // Interrupts disabled
	 (1<<USIWM1)|(0<<USIWM0)|                 // Set USI in Two-wire mode.
	 (1<<USICS1)|(0<<USICS0)|(1<<USICLK)|     // Software clock strobe as source.
	 (1<<USITC);                              // Toggle Clock Port.
	 do
	 {
		 _delay_us(30);
		 USICR = temp;                          // Generate positve SCL edge.
		 while( !(PIN_USI & (1<<PIN_USI_SCL)) );// Wait for SCL to go high.
		 _delay_us(30);
		 USICR = temp;                          // Generate negative SCL edge.
	 }while( !(USISR & (1<<USIOIF)) );        // Check for transfer complete.

	 _delay_us(30);
	 temp  = USIDR;                           // Read out data.
	 USIDR = 0xFF;                            // Release SDA.
	 DDR_USI |= (1<<PIN_USI_SDA);             // Enable SDA as output.

	 return temp;                             // Return the data from the USIDR
 }

 void USI_TWI_Start_Condition(){
	 PORT_USI |= (1<<PIN_USI_SCL);                     // Release SCL.
	 while( !(PIN_USI & (1<<PIN_USI_SCL)) );          // Verify that SCL becomes high.
	 PORT_USI &= ~(1<<PIN_USI_SDA);                    // Force SDA LOW.
	 _delay_us(30);
	 PORT_USI &= ~(1<<PIN_USI_SCL);                    // Pull SCL LOW.
	 PORT_USI |= (1<<PIN_USI_SDA);                     // Release SDA.
 }
 
 void USI_TWI_Stop_Condition(){
	 PORT_USI &= ~(1<<PIN_USI_SDA);           // Pull SDA low.
	 PORT_USI |= (1<<PIN_USI_SCL);            // Release SCL.
	 while( !(PIN_USI & (1<<PIN_USI_SCL)) );  // Wait for SCL to go high.
	 _delay_us(30);
	 PORT_USI |= (1<<PIN_USI_SDA);            // Release SDA.
	 _delay_us(30);
 }

 void USI_TWI_Check_ACK()
 {
	 if( USI_TWI_Master_Transfer( tempUSISR_1bit ) & (1<<TWI_NACK_BIT) ){
		 ACK_Rev = 0x00;
	 }
	 while(!ACK_Rev){
		 ;//error code
	 }
 }

 void USI_TWI_Write_Addr(unsigned char TWI_targetSlaveAddress)
 {
	 /* Write a byte */
	 PORT_USI &= ~(1<<PIN_USI_SCL);                // Pull SCL LOW.
	 USIDR     = TWI_targetSlaveAddress;           // Setup data.
	 USI_TWI_Master_Transfer( tempUSISR_8bit );    // Send 8 bits on bus.
	 
	 /* Clock and verify (N)ACK from slave */
	 DDR_USI  &= ~(1<<PIN_USI_SDA);                // Enable SDA as input.
 }

 unsigned char USI_TWI_Read_Byte()
 {
	 unsigned char TWI_Read;
	 DDR_USI   &= ~(1<<PIN_USI_SDA);               // Enable SDA as input.
	 TWI_Read  = USI_TWI_Master_Transfer( tempUSISR_8bit );
	 USIDR = 0xFF;
	 USI_TWI_Master_Transfer( tempUSISR_1bit ); // Generate ACK/NACK
	 USI_TWI_Stop_Condition();
	 return TWI_Read;
 }
 
 int Read_Battery(){
	unsigned char DeviceAddr_W = 0xE0;
	unsigned char DeviceAddr_R = 0xE1;
	unsigned char RegAddr = 0x08;
	unsigned char TWI_Read = 0x00;
	
	USI_TWI_Master_Initialise();
	USI_TWI_Start_Condition();//step <1>
	USI_TWI_Write_Addr(DeviceAddr_W);//step <2>
	USI_TWI_Check_ACK();//check if ack/nack is received
	USI_TWI_Write_Addr(RegAddr);//step <3>
	USI_TWI_Check_ACK();//check if ack/nack is received
	USI_TWI_Start_Condition();//step <4>
	USI_TWI_Write_Addr(DeviceAddr_R);//step <5>
	USI_TWI_Check_ACK();//check if ack/nack is received
	TWI_Read = USI_TWI_Read_Byte();//step <6> <7>
	
	int battery = TWI_Read * 5 / 255;
	return battery; 
 }
 
/*************************
ADC FUNCTIONS
these functions control ADC sampling
**************************/
void ADC_Channel_Select(int ch)
{
	ADMUX &= 0xF8;//clear original channel selection
	switch(ch){
		case 1 :
		ADMUX |= 0x01;
		break;
		case 2 :
		ADMUX |= 0x02;
		break;
		case 3 :
		ADMUX |= 0x03;
		break;
		case 4 :
		ADMUX |= 0x04;
		break;
		case 5 :
		ADMUX |= 0x05;
		break;
		case 6 :
		ADMUX |= 0x06;
		break;
		case 7 :
		ADMUX |= 0x07;
		break;
	}
}

void adc_init(){
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);
	ADC_Channel_Select(adc_channel);  //0..6
	adc_start();
}

void adc_start(){
	ADCSRA |= (1<<ADSC);
}

/*************************
LCD FUNCTIONS
these functions control LCD display
**************************/
void lcd_init(){
	Control_IO |= 1<<Enable | 1<<RW | 1<<RS;
	_delay_ms(15);
	Send_Command(FUNCTION);
	_delay_ms(5);
	Send_Command(FUNCTION);
	_delay_us(500);
	Send_Command(FUNCTION);
	_delay_us(50);
	Send_Command(CLEAR);
	_delay_ms(2);
	Send_Command(ENTRY);
	_delay_us(50);
	Send_Command(ALL_ON);
	_delay_us(50);
}

void Clear_Line(){
	for(int i=0;i<20;i++){
		Send_Char(0x20);
	}
}
void Check_Busy()
{
	Data_IO = 0x00;
	ControlBus |= 1<<RW;
	ControlBus &= ~1<<RS;

	while (DataBus >= 0x80)
	{
		Toggle();
	}
	
	Data_IO = 0xFF;
	_delay_us(5);
}

void Toggle()
{
	ControlBus |= 1<<Enable;
	//DO NOT make this value lower than 30us
	_delay_us(40);
	ControlBus &= ~(1<<Enable);
}

void Send_Command(unsigned char command)
{
	Check_Busy();
	DataBus = command;
	ControlBus &= ~ ((1<<RW)|(1<<RS));
	Toggle();
	DataBus = 0;
}

void Send_Char(unsigned char character)
{
	Check_Busy();
	DataBus = character;
	ControlBus &= ~(1<<RW);
	ControlBus |= 1<<RS;
	Toggle();
	DataBus = 0;
}

void Send_String(const char *string){
	int i = 0;
	for(i;i<strlen(string);i++){
		Send_Char(string[i]);
	}
}

/*************************
SD CARD FUNCTIONS
these functions control the reading of SD card
**************************/

void SPI_Clock_Change(int divide){
	//refer to Table 19-5
	SPCR &= ~(1<<SPR0 | 1<<SPR1);
	SPSR &= ~(1<<SPI2X);
	
	switch(divide){
		case 2 :
		SPSR |= (1<<SPI2X);
		break;
		case 4 :
		;
		break;
		case 8 :
		SPSR |= (1<<SPI2X);
		SPCR |= (1<<SPR0);
		break;
		case 16 :
		SPCR |= (1<<SPR0);
		break;
		case 32 :
		SPSR |= (1<<SPI2X);
		SPCR |= (1<<SPR1);
		break;
		case 64 :
		SPSR |= (1<<SPI2X);
		SPCR |= (1<<SPR0);
		SPCR |= (1<<SPR1);
		break;
		case 128 :
		SPCR |= (1<<SPR0);
		SPCR |= (1<<SPR1);
		break;
	}
}

unsigned long FAT32_Addr(unsigned long part_set, unsigned long off_set){
	unsigned long fat32_addr;
	fat32_addr = (part_set + off_set) * 0x200;
	return fat32_addr;
}

unsigned char* Address_Byte(unsigned long addr)
{
	sd_addr[0] = addr >> 24;
	sd_addr[1] = (addr << 8) >> 24;
	sd_addr[2] = (addr << 16) >> 24;
	sd_addr[3] = (addr << 24) >> 24;
	return sd_addr;
}

void SPI_Master_init(){
	//in Slave mode, only MISO should be output
	SPI_DDR |= SS | SCK | MOSI;//all output
	//in Slave mode, MSTR should be cleared
	SPCR |= (1<<SPE) | (1<<MSTR);//enable, master mode, Fosc/128
}

unsigned char SPI_Master_Write(unsigned char data){
	SPDR = data;
	while(!(SPSR & (1<<SPIF)))
	;
	return SPDR;
}

//This function should be called before the init command
void Clock_Pulse(){
	SS_DISABLE();
	int i = 0;
	for(i;i<11;i++){//88 clock cycles (minimum 74 )
		SPI_Master_Write(0xFF);//keeping MOSI high
	}
}

int SD_command(unsigned char cmd, unsigned long arg, unsigned char crc, unsigned char read) {
	
	unsigned char CMD = 0x40 | cmd;
	int i = 0;
	int check_response = 0;
	SS_ENABLE();

	SPI_Master_Write(CMD);
	SPI_Master_Write(arg>>24);
	SPI_Master_Write(arg>>16);
	SPI_Master_Write(arg>>8);
	SPI_Master_Write(arg);
	SPI_Master_Write(crc);
	//adjustment
	SPI_Master_Write(0xFF);
	for(i; i<read; i++){
		sd_response[i] = SPI_Master_Write(0xFF);
	}
	SS_DISABLE();
	
	if(cmd == CMD8){
		check_response = Check_Response(arg,4,0);//CMD8 check pattern
	}
	return check_response;
}

int Check_Response(unsigned long arg1, int index, int offset){//compare one byte in the command argument with one byte in response
	int i;
	unsigned char byte2 = (arg1>> (offset*8) ) & 0xFF;
	sd_response[index] == byte2? (i = 0): (i = 1);
	return i;

}

void SD_init(){
	SPI_Master_init();
	Clock_Pulse();
	do{
		SD_command(CMD0,0x00000000,0x95,6);
	}while(sd_response[0] != 0x01);
	
	while(SD_command(CMD8,0x000001AA,0x87,6));
	
	do{
		SD_command(CMD55,0x00000000,0xFF,6);
		SD_command(ACMD41,0x00000000,0xFF,6);
	}while(sd_response[0] != 0x00);
}

void SD_Read_Single(unsigned long address, unsigned short offset, unsigned short length, unsigned char* buffer){
	unsigned short k = offset+length;
	SS_ENABLE();
	unsigned char* addr = Address_Byte(address);
	/************alternitive CMD17****************/
	SPI_Master_Write(0x51);
	SPI_Master_Write(addr[0]);
	SPI_Master_Write(addr[1]);
	SPI_Master_Write(addr[2]);
	SPI_Master_Write(addr[3]);
	SPI_Master_Write(0xFF);
	
	do{
		timeout[0]++;
		SPI_Master_Write(0xFF);
		//lcd message here
	}while(timeout[0] < 30 && SPDR != 0x00);
	
	if(timeout[0] == 30){
		;//lcd message
	}
	
	do{
		timeout[1]++;
		SPI_Master_Write(0xFF);
	}while(timeout[1] < 30 && SPDR != 0xFE);
	
	if(timeout[1] == 30){
		;//lcd message
	}
	
	for(int i=0; i<offset; i++){
		SPI_Master_Write(0xFF);
	}//skip bytes
	
	for(int j=0; j<length; j++){
		buffer[j] = SPI_Master_Write(0xFF);
	}//read part within length only
	
	for(k; k<512; k++){
		SPI_Master_Write(0xFF);
	}
	//skip checksum
	SPI_Master_Write(0xFF);
	SPI_Master_Write(0xFF);
	
	SS_DISABLE();
	
	do{
		timeout[2]++;
		SD_command(CMD12,0x00000000,0xFF,6);
	} while (timeout[2] < 30 && sd_response[0] != 0x00);
	
	if(timeout[2] == 30){
		;//lcd message
	}
}

/*************************
MIDI LOGIC FUNCTIONS
these functions decode data from sd card to midi commands
**************************/
unsigned short SD_2_Buffer(unsigned short offset, unsigned char* buffer){
	unsigned short i = offset;
	unsigned short j = 0;
	for(i;sd_data[i] != 0xff;i++){
		buffer[j] = sd_data[i];
		j++;
	}
	buffer[j] = 0xff;//verify bit
	return i+1;
}

void Index_2_Music(){
	USART_Transmit(0x90);
	USART_Transmit(music_buff[index_buff[index_buff_ptr]]);
	USART_Transmit(music_buff[index_buff[index_buff_ptr]+1]);
	if(music_buff[index_buff[index_buff_ptr]+1] != 0x00){
		cur_note = music_buff[index_buff[index_buff_ptr]];
		laser_Finder(music_buff[index_buff[index_buff_ptr]]);
	}
	
}

void MIDI_Buffer()
{
	unsigned short time_ptr = 0x0000;
	unsigned short index_ptr = 0x0000;
	//reset pointers
	time_buff_ptr = 0x0000;
	index_buff_ptr = 0x0000;
	//reload buffers
	time_ptr = SD_2_Buffer(0x0000,music_buff);
	index_ptr = SD_2_Buffer(time_ptr,time_buff);
	SD_2_Buffer(index_ptr,index_buff);
}

/*************************
TIMER FUNCTIONS
timer initialization
**************************/
void timer_init(){
	//325 and 328p settings are different. TCCR0A is combined with TCCR0B in 328p
	TCCR0A = (1<<WGM01) | (1<<CS02) | (1<<CS00);//set mode and prescaler
	OCR0A = 39;//compare register
	TIMSK0 |= (1<<OCIE0A);//timer mask. choose OCR0A to compare
}

/*************************
UART FUNCTIONS
uart initialization and transmission
**************************/
unsigned char USART_Receive(void){
	while(!(UCSR0A & (1<<RXC0)))
	;
	return UDR0;
}

void USART_Transmit(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0)))
	;
	UDR0 = data;
}

void USART_putstring(const char *theString){
	int i = 0;
	for(i;i<strlen(theString);i++){
		USART_Transmit(theString[i]);
	}
}

void USART_Init(unsigned int ubrr){
	//baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	//enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	//format: 8 bit data, 1 stop bit
	UCSR0C = (3<<UCSZ00);//110
}
/*******************************
PUBLIC FUNCTIONS
these functions are used for general purposes
********************************/

void Clock_Prescale(){
	CLKPR = 0x80;
	CLKPR = 0x00;
}

void LCD_Disp_First(int pwr, int instrm){
	int temp = pwr;
	Send_Command(FIRSTLINE);
	Clear_Line();
	Send_Command(FIRSTLINE);
	if(instrm == 0){
		Send_String("Piano    ");
		}else if(instrm == 1){
		Send_String("Xylophone");
		}else if(instrm == 2){
		Send_String("Bass     ");
		}else if(instrm == 3){
		Send_String("NewAgePad");
		}else if(instrm == 4){
		Send_String("Choir Aah");	
		}else if(instrm == 5){
		Send_String("Dulcimer ");
		}else if(instrm == 6){
	    Send_String("Trumpet  ");
		}else if(instrm == 7){
		Send_String("Flute    ");
	}
	
	Send_String("Power:");
	for(int i=0;i<pwr;i++){
		if(temp != 0){
			Send_Char(0xFF);
			temp--;
			}else{
			Send_Char(0x20);
		}//end else
	}//end for
}

void LCD_Disp_Second(){
	Send_Command(SECONDLINE);
	Clear_Line();
	Send_Command(SECONDLINE);
	Send_String("   Mode: ");
	if(free_mode_flag){
		Send_String("Musician");
		}else{
		Send_String("Autoplay");
	}
}

void LCD_Disp_Third(int curr){//number of current music	
	Send_Command(THIRDLINE);
	Clear_Line();
	Send_Command(THIRDLINE);
	
	if(free_mode_flag == 1){
			Send_String("  Enjoy Playing!");
		}else{
		switch(curr){
			case 0 :
			Send_String(music1);
			break;
			case 1 :
			Send_String(music2);
			break;
			case 2 :
			Send_String(music3);
			break;
			case 3 :
			return 0;
			break;
			case 4 :
			return 0;
			break;
			case 5 :
			return 0;
			break;
			case 6 :
			return 0;
			break;
		}//end sw
	}//end else
}

void LCD_Disp_Fourth(){	
	Send_Command(FOURTHLINE);
	Send_String("Mode Next OK Timbre ");	
}

unsigned long Music_File_Addr(int num){
	switch(num){
		case 0 :
		return 16392;
		break;
		case 1 :
		return 16390;
		break;
		case 2 :
		return 16394;
		break;
		case 3 :
		return 0;
		break;
		case 4 :
		return 0;
		break;
		case 5 :
		return 0;
		break;
		case 6 :
		return 0;
		break;
	}
}

void Hand_Detection(int ch)
{
	int current_note;
	int i = 0;
	for(i;i<(ch+1);i++){
		if(adc_sensitive_detect[i]){
			handflag_buffer[i] = 1;                //hand detected
			}else if(!adc_sensitive_detect[i]){
			handflag_buffer[i] = 0;				   //no hand detected
		}
		
		if(handflag_buffer[i] == 1 && prehandflag_buffer[i] == 0){
			if(adc_buffer[i]){
				USART_Transmit(0x90+i);
				USART_Transmit(note_buffer[7*(adc_buffer[i]-1) + i]);    //7*parameter --- change pitch level of one string
				USART_Transmit(0x40);
				current_note = note_buffer[7*(adc_buffer[i]-1) + i];
			}
		}
		if(handflag_buffer[i] == 0 && prehandflag_buffer[i] == 1){
			USART_Transmit(0x90+i);
			USART_Transmit(note_buffer[7*(adc_buffer[i]-1) + i]);	//7*parameter --- change pitch level of one string
			USART_Transmit(0x00);
			adc_buffer[i] = 0;
		}
		prehandflag_buffer[i] = handflag_buffer[i];
	}
}

void Midi_Switch_Instrument(){
	if(change_instrument_flag == 1){
		USART_Transmit(0X90);
		USART_Transmit(cur_note);
		USART_Transmit(0X00);	
	}
	
	if(change_instrument_flag == 1){
		change_instrument_flag = 0;
		if(Instrument_Type==0){
			USART_Transmit(0XC0);
			USART_Transmit(0X01);
			USART_Transmit(0XC1);
			USART_Transmit(0X01);
			USART_Transmit(0XC2);
			USART_Transmit(0X01);
			USART_Transmit(0XC3);
			USART_Transmit(0X01);
			USART_Transmit(0XC4);
			USART_Transmit(0X01);
			USART_Transmit(0XC5);
			USART_Transmit(0X01);
			USART_Transmit(0XC6);
			USART_Transmit(0X01);
			//USART_putstring(Grand_Piano);
		}else if(Instrument_Type==1){
			USART_Transmit(0XC0);
			USART_Transmit(0X0E);
			USART_Transmit(0XC1);
			USART_Transmit(0X0E);
			USART_Transmit(0XC2);
			USART_Transmit(0X0E);
			USART_Transmit(0XC3);
			USART_Transmit(0X0E);
			USART_Transmit(0XC4);
			USART_Transmit(0X0E);
			USART_Transmit(0XC5);
			USART_Transmit(0X0E);
			USART_Transmit(0XC6);
			USART_Transmit(0X0E);
			//USART_putstring(Alto_Sax);
		}else if(Instrument_Type==2){
			USART_Transmit(0XC0);
			USART_Transmit(0X27);
			USART_Transmit(0XC1);
			USART_Transmit(0X27);
			USART_Transmit(0XC2);
			USART_Transmit(0X27);
			USART_Transmit(0XC3);
			USART_Transmit(0X27);
			USART_Transmit(0XC4);
			USART_Transmit(0X27);
			USART_Transmit(0XC5);
			USART_Transmit(0X27);
			USART_Transmit(0XC6);
			USART_Transmit(0X27);
			//USART_putstring(Sync-Bass);
		}else if(Instrument_Type==3){
			USART_Transmit(0XC0);
			USART_Transmit(0X59);
			USART_Transmit(0XC1);
			USART_Transmit(0X59);
			USART_Transmit(0XC2);
			USART_Transmit(0X59);
			USART_Transmit(0XC3);
			USART_Transmit(0X59);
			USART_Transmit(0XC4);
			USART_Transmit(0X59);
			USART_Transmit(0XC5);
			USART_Transmit(0X59);
			USART_Transmit(0XC6);
			USART_Transmit(0X59);
			//USART_putstring(Sync-Bass);
		}else if(Instrument_Type==4){
			USART_Transmit(0XC0);
			USART_Transmit(0X35);
			USART_Transmit(0XC1);
			USART_Transmit(0X35);
			USART_Transmit(0XC2);
			USART_Transmit(0X35);
			USART_Transmit(0XC3);
			USART_Transmit(0X35);
			USART_Transmit(0XC4);
			USART_Transmit(0X35);
			USART_Transmit(0XC5);
			USART_Transmit(0X35);
			USART_Transmit(0XC6);
			USART_Transmit(0X35);
			//USART_putstring(Sync-Bass);
		}else if(Instrument_Type==5){
			USART_Transmit(0XC0);
			USART_Transmit(0X10);
			USART_Transmit(0XC1);
			USART_Transmit(0X10);
			USART_Transmit(0XC2);
			USART_Transmit(0X10);
			USART_Transmit(0XC3);
			USART_Transmit(0X10);
			USART_Transmit(0XC4);
			USART_Transmit(0X10);
			USART_Transmit(0XC5);
			USART_Transmit(0X10);
			USART_Transmit(0XC6);
			USART_Transmit(0X10);
			//USART_putstring(Sync-Bass);
		}else if(Instrument_Type==6){
			USART_Transmit(0XC0);
			USART_Transmit(0X39);
			USART_Transmit(0XC1);
			USART_Transmit(0X39);
			USART_Transmit(0XC2);
			USART_Transmit(0X39);
			USART_Transmit(0XC3);
			USART_Transmit(0X39);
			USART_Transmit(0XC4);
			USART_Transmit(0X39);
			USART_Transmit(0XC5);
			USART_Transmit(0X39);
			USART_Transmit(0XC6);
			USART_Transmit(0X39);
			//USART_putstring(Sync-Bass);
		}else if(Instrument_Type==7){
			USART_Transmit(0XC0);
			USART_Transmit(0X4A);
			USART_Transmit(0XC1);
			USART_Transmit(0X4A);
			USART_Transmit(0XC2);
			USART_Transmit(0X4A);
			USART_Transmit(0XC3);
			USART_Transmit(0X4A);
			USART_Transmit(0XC4);
			USART_Transmit(0X4A);
			USART_Transmit(0XC5);
			USART_Transmit(0X4A);
			USART_Transmit(0XC6);
			USART_Transmit(0X4A);
			//USART_putstring(Sync-Bass);
		}
	}
	
}

void laser_Initilazation(){
		DDRA |= ~(0xFE);
		DDRD |= ~(0xFC);
		DDRE |= ~(0xF3);
		DDRG |= ~(0xE7);
}

void laser_Finder(unsigned char note_played){
	unsigned char reminder;
	reminder = note_played % 0x0C;
	if(reminder == 0 || reminder ==1 ){
		beam = 0;   //do & do#
		}else if(reminder == 2 || reminder ==3 ){
		beam = 1;   //re & re#
		}else if(reminder == 4 ){
		beam = 2;   //mi
		}else if(reminder == 5 || reminder == 6 ){
		beam = 3;   //fa & fa#
		}else if(reminder == 7 || reminder == 8 ){
		beam = 4;   //so & so#
		}else if(reminder == 9 || reminder ==10 ){
		beam = 5;   //la & la#
		}else if(reminder == 11 ){
		beam = 6;   //si
		}else{
		beam = 7;
	}
}
// void all_laser_control(){
// 	if(free_mode_flag){
// 		PORTA |= 0x01;
// 		PORTD |= 0x02;
// 		PORTD |= 0x01;
// 		PORTE |= 0x04;
// 		PORTE |= 0x08;
// 		PORTG |= 0x10;
// 		PORTG |= 0x08;
// 	}else{
// 		PORTA &= ~0x01;
// 		PORTD &= ~0x02;
// 		PORTD &= ~0x01;
// 		PORTE &= ~0x04;
// 		PORTE &= ~0x08;
// 		PORTG &= ~0x10;
// 		PORTG &= ~0x08;
// 	}
// 
// }

void all_laser_on(){
	PORTA |= 0x01;
	PORTD |= 0x02;
	PORTD |= 0x01;
	PORTE |= 0x04;
	PORTE |= 0x08;
	PORTG |= 0x10;
	PORTG |= 0x08;
}

void all_laser_off(){
	PORTA &= 0xFE;;
	PORTD &= 0XFD;
	PORTD &= 0XFE;
	PORTE &= 0XFB;
	PORTE &= 0XF7;
	PORTG &= 0XEF;
	PORTG &= 0XF7;
}
void Demutiplexer_Output(int time_point){

	//turn on logic
	if(beam==0 && beam_flag0==0){
		beam = 7;
		PORTA |= 0x01;
		beam_flag0 = 1;
		time_mark0 = time_point;
	}
	if(beam==1 && beam_flag1==0){
		beam = 7;
		PORTD |= 0x02;
		beam_flag1 = 1;
		time_mark1 = time_point;
	}
	if(beam==2 && beam_flag2==0){
		beam = 7;
		PORTD |= 0x01;
		beam_flag2 = 1;
		time_mark2 = time_point;
	}
	if(beam==3 && beam_flag3==0){
		beam = 7;
		PORTE |= 0x04;
		beam_flag3 = 1;
		time_mark3 = time_point;
	}
	if(beam==4  && beam_flag4==0){
		beam = 7;
		PORTE |= 0x08;
		beam_flag4 = 1;
		time_mark4 = time_point;
	}
	if(beam==5 && beam_flag5==0){
		beam = 7;
		PORTG |= 0x10;
		beam_flag5 = 1;
		time_mark5 = time_point;
	}
	if(beam==6 && beam_flag6==0){
		beam = 7;
		PORTG |= 0x08;
		beam_flag6 = 1;
		time_mark6 = time_point;
	}

	//turn off logic
	if(beam_flag0 = 1 && time_mark0 == (time_point+1)){
		beam_flag0 = 0;
		PORTA &= (0xFE);
	}
	if(beam_flag1 = 1 && time_mark1 == (time_point+1)){
		beam_flag1 = 0;
		PORTD &= (0xFD);
	}
	if(beam_flag2 = 1 && time_mark2 == (time_point+1)){
		beam_flag2 = 0;
		PORTD &= (0xFE);
	}
	if(beam_flag3 = 1 && time_mark3 == (time_point+1)){
		beam_flag3 = 0;
		PORTE &= (0xFB);
	}
	if(beam_flag4 = 1 && time_mark4 == (time_point+1)){
		beam_flag4 = 0;
		PORTE &= (0xF7);
	}
	if(beam_flag5 = 1 && time_mark5 == (time_point+1)){
		beam_flag5 = 0;
		PORTG &= (0xEF);
	}
	if(beam_flag6 = 1 && time_mark6 == (time_point+1)){
		beam_flag6 = 0;
		PORTG &= (0xF7);
	}
}

void array_sorter(){
	memcpy(sorted_array, result_array, sizeof(result_array));
	int temp = 0;
	for(int i=1; i<7; i++){
		temp = sorted_array[i];
		for(int j=i; j>=1 && temp<sorted_array[j-1]; j--){
			sorted_array[j] = sorted_array[j-1];
			sorted_array[j-1] = temp;
		}
	}
}

void ADC_quantization(){
	compare_value = sorted_array[0];   //value position in buffer
	adc_sensitive_detect[adc_channel] = (compare_value>280);//190
	if((compare_value>300)&&(compare_value<400)){//220 300
		adc_buffer[adc_channel] = 1;
		}else if(compare_value>430){//340
		adc_buffer[adc_channel] = 2;
	}
	
}

int main(void)
{	
	volatile int infi = 0;
	Clock_Prescale();
	//LCD display
	_delay_ms(150);//LCD power up delay
	lcd_init();
	LCD_Disp_First(Read_Battery(),Instrument_Type);
	LCD_Disp_Second();
	LCD_Disp_Third(cur_music);
	LCD_Disp_Fourth();
	//push buttons
	PB_INITILAZATION();
	//laser output
	laser_Initilazation();
	//SD card
	SD_init();
	SPI_Clock_Change(32);
	SD_Read_Single(FAT32_Addr(144,Music_File_Addr(cur_music)),0x0000,0x0200,sd_data);
	MIDI_Buffer();
	//timer
	timer_init();
	//ADC (sensor)
	adc_init();
	//UART (MIDI)
	USART_Init(MYUBRR);
	sei();//Global Interrupt Enable
	
	do{
		if(cur_music_ok){
			cur_music_ok = 0;
			midi_cnt = 0;
			SD_Read_Single(FAT32_Addr(144,Music_File_Addr(cur_music)),0x0000,0x0200,sd_data);
			MIDI_Buffer();
		}
	}while(infi < 20);//infinite
	
}

ISR(ADC_vect){
	check = 1;
	result_array[adc_flag] = ADC;
	adc_flag = (adc_flag + 1) % 7;//7
	if(adc_flag == 6){//6
		result_array[adc_flag] = ADC;
		array_sorter();
		ADC_quantization();
		adc_channel = (adc_channel + 1) % Sensor_Amount;//0..7
		ADC_Channel_Select(adc_channel);
	}
}

ISR(TIMER0_COMP_vect){
	//make sure 100Hz
	unsigned long music_addr = 0;
	midi_cnt++;
	half_second_counter = (half_second_counter + 1) % 120;
	
	
	//Change mode button
	if(!(PINA & (1<<PB_MODE)) && pre_MODE_flag == 1){
		free_mode_flag = !free_mode_flag;
		if(free_mode_flag){
			laser_on = 1;
		}else if(free_mode_flag == 0){
			laser_off = 1;
		}
		LCD_Disp_Second();
		LCD_Disp_Third(cur_music);
	}
	pre_MODE_flag = ((PINA & (1<<PB_MODE)) >> PB_MODE);
	
	//OK button
	if(!(PINA & (1<<PB_OK)) && pre_OK_flag == 1){
		cur_music_ok = 1;
	}
	pre_OK_flag = ((PINA & (1<<PB_OK)) >> PB_OK);
	
	//change instrument button
	if(!(PINA & (1<<PB_TIMBRE)) && pre_TIMBRE_flag == 1){
		led_flag = !led_flag;
		change_instrument_flag = 1;
		Instrument_Type = (Instrument_Type + 1)  % INSTR_NUM;
		LCD_Disp_First(Read_Battery(),Instrument_Type);
	}
	pre_TIMBRE_flag = ((PINA & (1<<PB_TIMBRE)) >> PB_TIMBRE);
	Midi_Switch_Instrument();//outside
	
	//free mode
	if(free_mode_flag){
		if(laser_on){
			laser_on = 0;
			all_laser_on();
		}
		Hand_Detection(Sensor_Amount - 1);
	}
	
	//Autoplay mode
	if(!free_mode_flag){
		if(laser_off){
			laser_off = 0;
			all_laser_off();
		}
		//Next button
		if(!(PINA & (1<<PB_NEXT)) && pre_NEXT_flag == 1){
			cur_music = (cur_music + 1) % MUSIC_NUM;
			LCD_Disp_Third(cur_music);
		}
		pre_NEXT_flag = ((PINA & (1<<PB_NEXT)) >> PB_NEXT);
		
		if(0){//lan de gai le
// 			cur_music_ok = 0;
// 			midi_cnt = 0;
// 			SD_Read_Single(FAT32_Addr(144,Music_File_Addr(cur_music)),0x0000,0x0200,sd_data);
// 			MIDI_Buffer();
		}else{
			if(midi_cnt == 25){//25
				midi_cnt = 0;
				
				if(time_buff[time_buff_ptr] != 0xFF){
					if(time_buff[time_buff_ptr] != 0){
						time_buff[time_buff_ptr]--;
						}else{
						if(index_buff[index_buff_ptr] != 0xFF){
							while(index_buff[index_buff_ptr] != 0){
								Index_2_Music();  //laser finder included
								index_buff_ptr++;
							}
							index_buff_ptr++;
							time_buff_ptr++;
						}
					}
					}else{
						MIDI_Buffer();
					}//end else timbuff 0xFF				
				}//end if midi_cnt == 25
			}//end else cur_music_ok
		Demutiplexer_Output(half_second_counter);//inside if(Autoplay_flag)
		}
}//end ISR

