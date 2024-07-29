/*
  bootloader for AM32 ESC firmware

  based on https://github.com/AlkaMotors/AT32F421_AM32_Bootloader
 */
#include <main.h>
#include <stdio.h>

#include <version.h>

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdio.h>

#include "main.h"
#include <eeprom.h>

//#define USE_ADC_INPUT      // will go right to application and ignore eeprom
//#define UPDATE_EEPROM_ENABLE

// use this to check the clock config for the MCU (with a logic
// analyser on the input pin)
//#define BOOTLOADER_TEST_CLOCK

// when there is no app fw yet, disable jump()
//#define DISABLE_JUMP

#include <string.h>

#define STM32_FLASH_START 0x08000000
#define FIRMWARE_RELATIVE_START 0x1000

#ifdef SIXTY_FOUR_KB_MEMORY
#define EEPROM_RELATIVE_START 0xf800
#else
#define EEPROM_RELATIVE_START 0x7c00
#endif

typedef void (*pFunction)(void);

#define APPLICATION_ADDRESS     (uint32_t)(STM32_FLASH_START + FIRMWARE_RELATIVE_START) // 4k

#define EEPROM_START_ADD         (uint32_t)(STM32_FLASH_START + EEPROM_RELATIVE_START)
#define FLASH_END_ADD           (uint32_t)(STM32_FLASH_START + 0x7FFF)               // 32 k


#define CMD_RUN             0x00
#define CMD_PROG_FLASH      0x01
#define CMD_ERASE_FLASH     0x02
#define CMD_READ_FLASH_SIL  0x03
#define CMD_VERIFY_FLASH    0x03
#define CMD_VERIFY_FLASH_ARM 0x04
#define CMD_READ_EEPROM     0x04
#define CMD_PROG_EEPROM     0x05
#define CMD_READ_SRAM       0x06
#define CMD_READ_FLASH_ATM  0x07
#define CMD_KEEP_ALIVE      0xFD
#define CMD_SET_ADDRESS     0xFF
#define CMD_SET_BUFFER      0xFE

#ifdef USE_PA2
#define input_pin        GPIO_PIN(2)
#define input_port       GPIOA
#define PIN_NUMBER       2
#define PORT_LETTER      0
#elif defined(USE_PB4)
#define input_pin         GPIO_PIN(4)
#define input_port        GPIOB
#define PIN_NUMBER        4
#define PORT_LETTER       1
#elif defined(USE_PA15)
#define input_pin         GPIO_PIN(15)
#define input_port        GPIOA
#define PIN_NUMBER        15
#define PORT_LETTER       0
#else
#error "Bootloader comms pin not defined"
#endif

#include <blutil.h>

static uint16_t low_pin_count;
static char receiveByte;
static int count;
static char messagereceived;
static uint16_t invalid_command;
static uint16_t address_expected_increment;
static int cmd;
static char eeprom_req;
static int received;


static uint8_t pin_code = PORT_LETTER << 4 | PIN_NUMBER;
#ifdef SIXTY_FOUR_KB_MEMORY
static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x64,0x35,0x06,0x06,0x01,0x30};
#else
static uint8_t deviceInfo[9] = {0x34,0x37,0x31,0x00,0x1f,0x06,0x06,0x01,0x30};      // stm32 device info
#endif


static uint8_t rxBuffer[258];
static uint8_t payLoadBuffer[256];
static char rxbyte;
static uint32_t address;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

static uint16_t len;
static uint8_t calculated_crc_low_byte;
static uint8_t calculated_crc_high_byte;
static uint16_t payload_buffer_size;
static char incoming_payload_no_command;

static uint32_t JumpAddress;
static pFunction JumpToApplication;

/* USER CODE BEGIN PFP */
static void serialwriteChar(char data);
static void sendString(uint8_t data[], int len);
static void receiveBuffer();

#define BAUDRATE      19200
#define BITTIME          52 // 1000000/BAUDRATE
#define HALFBITTIME      26 // 500000/BAUDRATE

static void delayMicroseconds(uint32_t micros)
{
    bl_timer_reset();
    while (bl_timer_us() < micros) {
    }
}

/*
  jump to the application firmware
 */
static void jump()
{
#ifndef DISABLE_JUMP
    __disable_irq();
    JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
    uint8_t value = *(uint8_t*)(EEPROM_START_ADD);
    if (value != 0x01) {      // check first byte of eeprom to see if its programmed, if not do not jump
	invalid_command = 0;
	return;
    }
    bl_timer_disable();
    JumpToApplication = (pFunction) JumpAddress;

    __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
    JumpToApplication();
#endif
}



static void makeCrc(uint8_t* pBuff, uint16_t length)
{
    static uint8_16_u CRC_16;
    CRC_16.word=0;

    for(int i = 0; i < length; i++) {


	uint8_t xb = pBuff[i];
	for (uint8_t j = 0; j < 8; j++)
	{
	    if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
		CRC_16.word = CRC_16.word >> 1;
		CRC_16.word = CRC_16.word ^ 0xA001;
	    } else {
		CRC_16.word = CRC_16.word >> 1;
	    }
	    xb = xb >> 1;
	}
    }
    calculated_crc_low_byte = CRC_16.bytes[0];
    calculated_crc_high_byte = CRC_16.bytes[1];
}

static char checkCrc(uint8_t* pBuff, uint16_t length)
{

    char received_crc_low_byte2 = pBuff[length];          // one higher than len in buffer
    char received_crc_high_byte2 = pBuff[length+1];
    makeCrc(pBuff,length);

    if((calculated_crc_low_byte==received_crc_low_byte2)   && (calculated_crc_high_byte==received_crc_high_byte2)){
	return 1;
    }else{
	return 0;
    }
}


static void setReceive()
{
    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
    received = 0;
}

static void setTransmit()
{
    gpio_set(input_pin);
    gpio_mode_set_output(input_pin, GPIO_OUTPUT_PUSH_PULL);
}


static void send_ACK()
{
    setTransmit();
    serialwriteChar(0x30);             // good ack!
    setReceive();
}

static void send_BAD_ACK()
{
    setTransmit();
    serialwriteChar(0xC1);                // bad command message.
    setReceive();
}

static void send_BAD_CRC_ACK()
{
    setTransmit();
    serialwriteChar(0xC2);                // bad command message.
    setReceive();
}

static void sendDeviceInfo()
{
    setTransmit();
    sendString(deviceInfo,9);
    setReceive();
}

static bool checkAddressWritable(uint32_t address)
{
    return address >= APPLICATION_ADDRESS;
}

static void decodeInput()
{
    if (incoming_payload_no_command) {
	len = payload_buffer_size;
	if (checkCrc(rxBuffer,len)) {
	    memset(payLoadBuffer, 0, sizeof(payLoadBuffer));             // reset buffer

	    for(int i = 0; i < len; i++){
		payLoadBuffer[i]= rxBuffer[i];
	    }
	    send_ACK();
	    incoming_payload_no_command = 0;
	    return;
	}else{
	    send_BAD_CRC_ACK();
	    return;
	}
    }

    cmd = rxBuffer[0];

    if (rxBuffer[16] == 0x7d) {
	if(rxBuffer[8] == 13 && rxBuffer[9] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;

	}
	return;
    }

    if (rxBuffer[20] == 0x7d) {
	if(rxBuffer[12] == 13 && rxBuffer[13] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;
	    return;
	}

    }
    if (rxBuffer[40] == 0x7d) {
	if (rxBuffer[32] == 13 && rxBuffer[33] == 66) {
	    sendDeviceInfo();
	    rxBuffer[20]= 0;
	    return;
	}
    }

    if (cmd == CMD_RUN) {
	// starts the main app
	if((rxBuffer[1] == 0) && (rxBuffer[2] == 0) && (rxBuffer[3] == 0)) {
	    invalid_command = 101;
	}
    }

    if (cmd == CMD_PROG_FLASH) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	if (!checkAddressWritable(address)) {
	    send_BAD_ACK();

	    return;
	}

	save_flash_nolib((uint8_t*)payLoadBuffer, payload_buffer_size,address);
	send_ACK();

	return;
    }

    if (cmd == CMD_SET_ADDRESS) {
	// command set addressinput format is: CMD, 00 , High byte
	// address, Low byte address, crclb ,crchb
	len = 4;  // package without 2 byte crc
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}


	// will send Ack 0x30 and read input after transfer out callback
	invalid_command = 0;
	address = STM32_FLASH_START + (rxBuffer[2] << 8 | rxBuffer[3]);
	send_ACK();

	return;
    }

    if (cmd == CMD_SET_BUFFER) {
	// for writing buffer rx buffer 0 = command byte.  command set
	// address, input , format is CMD, 00 , 00 or 01 (if buffer is 256),
	// buffer_size,
	len = 4;  // package without 2 byte crc
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

        // no ack with command set buffer;
       	if(rxBuffer[2] == 0x01){
	    payload_buffer_size = 256;                          // if nothing in this buffer
       	}else{
	    payload_buffer_size = rxBuffer[3];
        }
	incoming_payload_no_command = 1;
	address_expected_increment = 256;
        setReceive();

        return;
    }

    if (cmd == CMD_KEEP_ALIVE) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	setTransmit();
	serialwriteChar(0xC1);                // bad command message.
	setReceive();

	return;
    }

    if (cmd == CMD_ERASE_FLASH) {
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	if (!checkAddressWritable(address)) {
	    send_BAD_ACK();

	    return;
	}

	send_ACK();
	return;
    }

    if (cmd == CMD_READ_EEPROM) {
	eeprom_req = 1;
    }

    if (cmd == CMD_READ_FLASH_SIL) {
	// for sending contents of flash memory at the memory location set in
	// bootloader.c need to still set memory with data from set mem
	// command
	len = 2;
	if (!checkCrc((uint8_t*)rxBuffer, len)) {
	    send_BAD_CRC_ACK();

	    return;
	}

	count++;
	uint16_t out_buffer_size = rxBuffer[1];//
	if(out_buffer_size == 0){
	    out_buffer_size = 256;
	}
	address_expected_increment = 128;

	setTransmit();
	uint8_t read_data[out_buffer_size + 3];        // make buffer 3 larger to fit CRC and ACK
	memset(read_data, 0, sizeof(read_data));
        //    read_flash((uint8_t*)read_data , address);                 // make sure read_flash reads two less than buffer.
	read_flash_bin((uint8_t*)read_data , address, out_buffer_size);

        makeCrc(read_data,out_buffer_size);
        read_data[out_buffer_size] = calculated_crc_low_byte;
        read_data[out_buffer_size + 1] = calculated_crc_high_byte;
        read_data[out_buffer_size + 2] = 0x30;
        sendString(read_data, out_buffer_size+3);

	setReceive();

	return;
    }

    setTransmit();

    serialwriteChar(0xC1);                // bad command message.
    invalid_command++;
    setReceive();
}


static void serialreadChar()
{
    rxbyte=0;
    while (!gpio_read(input_pin)) { // wait for rx to go high
	if(bl_timer_us() > 20000){
	    invalid_command = 101;
	    return;
	}
    }


    while (gpio_read(input_pin)) {   // wait for it go go low
	if(bl_timer_us() > 250 && messagereceived){
	    return;
	}
    }

    delayMicroseconds(HALFBITTIME);//wait to get the center of bit time

    int bits_to_read = 0;
    while (bits_to_read < 8) {
	delayMicroseconds(BITTIME);
	rxbyte = rxbyte | gpio_read(input_pin) << bits_to_read;
	bits_to_read++;
    }

    delayMicroseconds(HALFBITTIME); //wait till the stop bit time begins
    messagereceived = 1;
    receiveByte = rxbyte;
}


void serialwriteChar(char data)
{
    gpio_clear(input_pin);
    char bits_to_read = 0;
    while (bits_to_read < 8) {
	delayMicroseconds(BITTIME);
	if (data & 0x01) {
	    //GPIO_BOP(input_port) = input_pin;
	    gpio_set(input_pin);
	} else {
	    // GPIO_BC(input_port) = input_pin;
	    gpio_clear(input_pin);
	}
	bits_to_read++;
	data = data >> 1;
    }

    delayMicroseconds(BITTIME);
    gpio_set(input_pin);
}


static void sendString(uint8_t *data, int len)
{
    for(int i = 0; i < len; i++){
	serialwriteChar(data[i]);
	delayMicroseconds(BITTIME);
    }
}

static void receiveBuffer()
{
    count = 0;
    messagereceived = 0;
    memset(rxBuffer, 0, sizeof(rxBuffer));

    for(int i = 0; i < sizeof(rxBuffer); i++){
	serialreadChar();

	if(incoming_payload_no_command) {
	    if(count == payload_buffer_size+2){
		break;
	    }
	    rxBuffer[i] = rxbyte;
	    count++;
	} else {
	    if(bl_timer_us() > 250){
	
		count = 0;

		break;
	    } else {
		rxBuffer[i] = rxbyte;
		if(i == 257){
		    invalid_command+=20;       // needs one hundred to trigger a jump but will be reset on next set address commmand

		}
	    }
	}
    }
    decodeInput();
	
}

#ifdef UPDATE_EEPROM_ENABLE
static void update_EEPROM()
{
    read_flash_bin(rxBuffer , EEPROM_START_ADD , 48);
    if(BOOTLOADER_VERSION != rxBuffer[2]) {
	if (rxBuffer[2] == 0xFF || rxBuffer[2] == 0x00){
	    return;
	}
	rxBuffer[2] = BOOTLOADER_VERSION;
	save_flash_nolib(rxBuffer, 48, EEPROM_START_ADD);
    }
}
#endif // UPDATE_EEPROM_ENABLE

static void checkForSignal()
{
    gpio_mode_set_input(input_pin, GPIO_PULL_DOWN);
	
    delayMicroseconds(500);

    for(int i = 0 ; i < 500; i ++){
	if(!gpio_read(input_pin)){
	    low_pin_count++;
	}else{
	}

	delayMicroseconds(10);
    }
    if (low_pin_count > 450) {
	if (!bl_was_software_reset()) {
	    jump();
	}
    }

    gpio_mode_set_input(input_pin, GPIO_PULL_UP);
	
    delayMicroseconds(500);

    for (int i = 0 ; i < 500; i++) {
	if( !(gpio_read(input_pin))){
	    low_pin_count++;
	}else{

	}
	delayMicroseconds(10);
    }
    if (low_pin_count == 0) {
	return;           // all high while pin is pulled low, bootloader signal
    }

    low_pin_count = 0;

    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);

    delayMicroseconds(500);

    for (int i = 0 ; i < 500; i ++) {
	if( !(gpio_read(input_pin))){
	    low_pin_count++;
	}

	delayMicroseconds(10);
    }
    if (low_pin_count == 0) {
	return;            // when floated all
    }

    if (low_pin_count > 0) {
	jump();
    }
}

#ifdef BOOTLOADER_TEST_CLOCK
/*
  this should provide a 2ms low followed by a 1ms high if the clock is correct
 */
static void test_clock(void)
{
    setTransmit();
    while (1) {
	gpio_clear(input_pin);
	bl_timer_reset();
	while (bl_timer_us() < 2000) ;
	gpio_set(input_pin);
	bl_timer_reset();
	while (bl_timer_us() < 1000) ;
    }
}
#endif // BOOTLOADER_TEST_CLOCK

int main(void)
{
    bl_clock_config();
    bl_timer_init();
    bl_gpio_init();

#ifdef BOOTLOADER_TEST_CLOCK
    test_clock();
#endif

    checkForSignal();

    gpio_mode_set_input(input_pin, GPIO_PULL_NONE);
		
#ifdef USE_ADC_INPUT  // go right to application
    jump();
#endif

    deviceInfo[3] = pin_code;

#ifdef UPDATE_EEPROM_ENABLE
     update_EEPROM();
#endif

    while (1) {
	  receiveBuffer();
	  if (invalid_command > 100) {
	      jump();
	  }
    }
}
