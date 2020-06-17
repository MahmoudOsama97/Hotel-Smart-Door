#include <stdint.h>
#include <math.h>
#include "C:\Keil_v5\EE319Kware\inc\tm4c123gh6pm.h" 
#define PORTA 0x40004000 
#define PORTB 0x40005000
#define PORTC 0x40006000
#define PORTD 0x40007000
#define PORTE 0x40024000
#define PORTF 0x40025000
#define SYSCTL_PRGPIO ((volatile uint32_t *)0x400FEA08)
#define SYSCTL_RCGCGPIO ((volatile uint32_t *)0x400FE608)
#define RCGC2_REG ((volatile uint32_t *)0x400FE108) // really don't know why we are using this
#define UNLOCK 0x4C4F434B
#define PORT_OFFSET 0x3FC
#define HIGH 0xFF 
#define LOW 0x00
#define Memory(X, Y) (*((volatile uint32_t *)(((uint32_t)X) + ((uint32_t)Y))))
#define Log(X) (int)log2(X)
#define UART_F 0x02
typedef enum PinMode{DIGITAL,ANALOG,} PinMode;
typedef enum GPIORegistersOffset{GPIO_DIR = 0x400,GPIO_IS = 0x404,GPIO_IBE = 0x408,GPIO_IEV = 0x40C,GPIO_IM = 0x410,GPIO_RIS = 0x414,GPIO_MIS = 0x418,GPIO_ICR = 0x41C,GPIO_AFSEL = 0x420,GPIO_DRV2M = 0x500,GPIO_DRV4M = 0x504,GPIO_DRV8M = 0x508,GPIO_ODR = 0x50C,GPIO_PUR = 0x510,GPIO_PDR = 0x514,GPIO_SLR = 0x518,GPIO_DEN = 0x51C,GPIO_LCK = 0x520,GPIO_CR = 0x524,GPIO_AMSEL = 0x528,GPIO_PCTL = 0x52C} GPIORegistersOffset;
typedef enum Pin{PIN0 = 0x001,PIN1 = 0x002,PIN2 = 0x004,PIN3 = 0x008,PIN4 = 0x010,PIN5 = 0x020,PIN6 = 0x040,PIN7 = 0x080,} Pin;
typedef enum PinDir{INPUT = 0x00,OUTPUT = 0xFF,PERIPHERAL,} PinDir;
typedef enum InterruptEvent{RISING,FALLING,BOTH,} InterruptEvent;
#define CLK_SPEED 50000000 
#define UART_BUFFER_SIZE 64 
#define Memory(X, Y) (*((volatile uint32_t *)(((uint32_t)X) + ((uint32_t)Y)))) 
#define BAUD_IDIV(B) (int)(CLK_SPEED / (16 * B)) 
#define BAUD_FDIV(B) (int)(((CLK_SPEED / (16 * B)) - BAUD_IDIV(B)) * 64) 
#define UART_CLK ((volatile uint32_t *)0x400FE618) 
#define RCGC2_REG ((volatile uint32_t *)0x400FE108) //really don't know why we are using this 
typedef enum Uart {UART0 = 0x4000C000, UART1 = 0x4000D000,UART2 = 0x4000E000,UART3 = 0x4000F000, UART4 = 0x40010000, UART5 = 0x40011000, UART6 = 0x40012000, UART7 = 0x40013000, } Uart; 
typedef enum UARTRegistersOffset { UART_DATA = 0x000,  UART_RS_EC = 0x004,  UART_FLAG = 0x018,  UART_IBRD = 0x024,   UART_FBRD = 0x028,   UART_LCTL = 0x02C,  UART_CTL = 0x030,   UART_IFLS = 0x034, UART_IM = 0x038,  UART_RIS = 0x03C, UART_MIS = 0x040, UART_IC = 0x044, UART_DMACTL = 0x048,  UART_PP = 0xFC0,  UART_CC = 0xFC8, } UARTRegistersOffset; 
typedef enum UART_BAUDRATE { UART_BAUD_300 = 300,  UART_BAUD_1200 = 1200,  UART_BAUD_2400 = 2400,  UART_BAUD_4800 = 4800, UART_BAUD_9600 = 9600,  UART_BAUD_19200 = 19200,  UART_BAUD_38400 = 38400,UART_BAUD_57600 = 57600,  UART_BAUD_74880 = 74880,  UART_BAUD_115200 = 115200, } UART_BAUDRATE; 
void (*callbackTable[6][8])(); 
void GPIO_setClkSource(volatile uint32_t portAddress){ 
    switch(portAddress){ 
case PORTA:(*RCGC2_REG) |= 0x01; (*SYSCTL_RCGCGPIO) |= 0x01; while(!((*SYSCTL_PRGPIO)|= 0x01)); break; 
case PORTB:(*RCGC2_REG) |= 0x02;  (*SYSCTL_RCGCGPIO) |= 0x02; while(!((*SYSCTL_PRGPIO)|= 0x02)); break; 
case PORTC:(*RCGC2_REG) |= 0x04; (*SYSCTL_RCGCGPIO) |= 0x04; while(!((*SYSCTL_PRGPIO)&0x00000004));  break; 
case PORTD:(*RCGC2_REG) |= 0x08;  (*SYSCTL_RCGCGPIO) |= 0x08;  while(!((*SYSCTL_PRGPIO)|= 0x08));   break;  
case PORTE:(*RCGC2_REG) |= 0x10;  (*SYSCTL_RCGCGPIO) |= 0x10; while(!((*SYSCTL_PRGPIO)|= 0x10));  break; 
case PORTF:(*RCGC2_REG) |= 0x20;(*SYSCTL_RCGCGPIO) |= 0x20; while(!((*SYSCTL_PRGPIO)|= 0x20)); break;  } } 
void GPIO_initPin(volatile uint32_t portAddress, Pin pin, PinMode mode, PinDir dir){ 
    GPIO_setClkSource(portAddress); Memory(portAddress, GPIO_LCK) = UNLOCK; Memory(portAddress, GPIO_CR)  |= pin;     
    if(mode == DIGITAL){ Memory(portAddress, GPIO_DEN) |= pin; Memory(portAddress, GPIO_AMSEL) &= ~(pin); 
    }else{ Memory(portAddress, GPIO_DEN) &= ~(pin); Memory(portAddress, GPIO_AMSEL) |= pin; } 
    if(dir == OUTPUT){ Memory(portAddress, GPIO_DIR) |= pin;  Memory(portAddress, GPIO_AFSEL) &= ~(pin); 
    }else if(dir == INPUT){  Memory(portAddress, GPIO_DIR) &= ~(pin);  Memory(portAddress, GPIO_AFSEL) &= ~(pin); 
    }else if(dir==PERIPHERAL){ Memory(portAddress, GPIO_AFSEL) |= (pin);  }} 
uint8_t GPIO_readPin(volatile uint32_t portAddress,Pin pin){ return Memory(portAddress, PORT_OFFSET) & pin; } 
void GPIO_writePin(volatile uint32_t portAddress, Pin pin, uint8_t value){  Memory(portAddress, (pin<<2)) = value; } 
void GPIO_initPort(volatile uint32_t portAddress, PinDir dir){ 
    GPIO_setClkSource(portAddress); Memory(portAddress, GPIO_LCK)= UNLOCK; Memory(portAddress, GPIO_CR)= 0xFF; Memory(portAddress, GPIO_DEN)= 0xFF; Memory(portAddress, GPIO_AMSEL)= 0x00; Memory(portAddress, GPIO_AFSEL) = 0x00; Memory(portAddress, GPIO_PCTL)= 0x00; Memory(portAddress, GPIO_DIR)= dir; } 
uint8_t GPIO_readPort(volatile uint32_t portAddress){return Memory(portAddress, PORT_OFFSET);}
void GPIO_writePort(volatile uint32_t portAddress, uint8_t value){Memory(portAddress, PORT_OFFSET) = value;}
void GPIO_setPullup(volatile uint32_t portAddress, Pin pin){Memory(portAddress, GPIO_PUR) |= pin;}
void GPIO_setPulldown(volatile uint32_t portAddress, Pin pin){	Memory(portAddress, GPIO_PDR) |= pin;}
void GPIO_setPCTL(volatile uint32_t portAddress, Pin pin, uint8_t function){Memory(portAddress, GPIO_PCTL) = (Memory(portAddress, GPIO_PCTL) & (~(0x0000000F << (Log(pin)*4))))|(function << ((Log(pin)*4)));}
void GPIO_setOpenDrain(volatile uint32_t portAddress, Pin pin){Memory(portAddress, GPIO_ODR) |= pin;}
char key_pad [4][4] = {{'1','2','3','A'},{'4','5','6','B'},  {'7','8','9','C'}, {'*','0','#','D'}}; 
void Keypad_init(void) {GPIO_initPort(PORTE,OUTPUT); 
GPIO_initPort(PORTC,INPUT); 
GPIO_initPin(PORTC,PIN4,DIGITAL,INPUT); GPIO_initPin(PORTC,PIN5,DIGITAL,INPUT); GPIO_initPin(PORTC,PIN6,DIGITAL,INPUT); GPIO_initPin(PORTC,PIN7,DIGITAL,INPUT); 
GPIO_setPullup(PORTC,PIN4); GPIO_setPullup(PORTC,PIN5); GPIO_setPullup(PORTC,PIN6); GPIO_setPullup(PORTC,PIN7); 
GPIO_setOpenDrain(PORTC,PIN4); GPIO_setOpenDrain(PORTC,PIN5); GPIO_setOpenDrain(PORTC,PIN6); GPIO_setOpenDrain(PORTC,PIN7); 
GPIO_initPin(PORTE,PIN1,DIGITAL,OUTPUT); GPIO_initPin(PORTE,PIN2,DIGITAL,OUTPUT); GPIO_initPin(PORTE,PIN3,DIGITAL,OUTPUT);GPIO_initPin(PORTE,PIN4,DIGITAL,OUTPUT); 
GPIO_setPullup(PORTE,PIN1); GPIO_setPullup(PORTE,PIN2); GPIO_setPullup(PORTE,PIN3); GPIO_setPullup(PORTE,PIN4); 
GPIO_setOpenDrain(PORTE,PIN1); GPIO_setOpenDrain(PORTE,PIN2); GPIO_setOpenDrain(PORTE,PIN3); GPIO_setOpenDrain(PORTE,PIN4);} 
char keypad_getkey(void){ int  col; col= GPIO_readPort(PORTC); GPIO_writePort(PORTE,0x00); 
if(col==0xF0){return 'x'; } 
while(1){ GPIO_writePort(PORTE,0x1C);/*delayMs(100);*/ col = GPIO_readPort(PORTC) & 0xF0; 
if (col == 0xE0){ return key_pad [0][0];/*delayMs(100);*/break;} if (col == 0xD0){ return key_pad [0][1];/*delayMs(100);*/break;} 
if (col == 0xB0){ return key_pad [0][2];/*delayMs(100);*/break;} if (col == 0x70){ return key_pad [0][3];/*delayMs(100);*/break;} 
GPIO_writePort(PORTE,0x1A);/*delayMs(100);*/col = GPIO_readPort(PORTC) & 0xF0; 
if (col == 0xE0){ return key_pad [1][0];/*delayMs(100);*/break;} if (col == 0xD0){ return key_pad [1][1];/*delayMs(100);*/break;} 
if (col == 0xB0){ return key_pad [1][2];/*delayMs(100);*/break;} if (col == 0x70){ return key_pad [1][3];/*delayMs(100);*/break;}
GPIO_writePort(PORTE,0x16);/*delayMs(100);*/col = GPIO_readPort(PORTC)& 0xF0; 
if (col == 0xE0){ return key_pad [2][0];/*delayMs(100);*/break;} if (col == 0xD0){ return key_pad [2][1];/*delayMs(100);*/break;} 
if (col == 0xB0){ return key_pad [2][2];/*delayMs(100);*/break;} if (col == 0x70){ return key_pad [2][3];/*delayMs(100);*/break;} 
GPIO_writePort(PORTE,0x0E); /*delayMs(100);*/col = GPIO_readPort(PORTC) & 0xF0; 
if (col == 0xE0){ return key_pad [3][0];/*delayMs(100);*/break;} if (col == 0xD0){ return key_pad [3][1];/*delayMs(100);*/ break;} 
if (col == 0xB0){ return key_pad [3][2];/*delayMs(100);*/ break;} if (col == 0x70){ return key_pad [3][3];/*delayMs(100);*/break;}}} 
char buffer[UART_BUFFER_SIZE]; 
volatile uint8_t buffer_index = 0; volatile Uart uartAttachedToBuffer; 
void UART_Init(Uart uart, UART_BAUDRATE baudrate){ 
if(uart == UART1){ 
GPIO_initPin(PORTC, PIN4, DIGITAL, PERIPHERAL); 
GPIO_initPin(PORTC, PIN5, DIGITAL, PERIPHERAL); 
GPIO_setPCTL(PORTC, PIN4, UART_F); 
GPIO_setPCTL(PORTC, PIN5, UART_F); 
(*UART_CLK) |= (1 << 1);// activate UART1  
(*RCGC2_REG) |= (1 << 2);} 
Memory(uart, UART_CTL) &= ~0x00000001;// Disable UART Module
Memory(uart, UART_IBRD) = BAUD_IDIV(baudrate);// writing integer baudrate divisor 
Memory(uart, UART_FBRD) = BAUD_FDIV(baudrate);// Fractional integer baudrate divisor 
Memory(uart, UART_CC) &= ~0x0000000F;// clock source is system clock 
Memory(uart, UART_LCTL) = 0x00000060;// 8 bit, no parity bits, one stop, no FIFOs 
Memory(uart, UART_CTL) |= 0x00000001;} 
uint8_t UART_readChar(Uart uart){while((Memory(uart, UART_FLAG)&0x0010) != 0);return (uint8_t)Memory(uart, UART_DATA); } 
void UART_ReadString(char *dest,int L){ for(char i=0;i<L;i++){dest[i]=UART_readChar(UART1);    } } 
int main(){  
GPIO_initPin(PORTF,PIN0,DIGITAL,INPUT); //enterpassword switch 
GPIO_initPin(PORTF,PIN1,DIGITAL,OUTPUT);// lock  
UART_Init(UART1,UART_BAUD_9600); Keypad_init(); 
uint8_t roomID=UART_readChar(UART1);uint8_t status=UART_readChar(UART1);  
int flag=1; char check[4]={1,2,3,4}; char password[4]={1,2,3,4};  
while(1){if(GPIO_readPin(PORTF,PIN0)){ 
if((UART0_FR_R&0x0010)==0){
if(roomID==UART_readChar(UART1)){ uint8_t status=UART_readChar(UART1); 
if(status==0)GPIO_writePin(PORTF,PIN1,LOW); 
else if(status==2)GPIO_writePin(PORTF,PIN1,HIGH); 
else if(status==1){UART_ReadString(password,4);  
GPIO_writePin(PORTF,PIN1,LOW);} } } 
}else{if(status==1) { 
for(int i=0;i<4;i++){ check[i]=keypad_getkey();if(check[i]!=password[i]){flag=0; break;} } 
if(flag==1) GPIO_writePin(PORTF,PIN1,HIGH); 
else GPIO_writePin(PORTF,PIN1,LOW); flag=1; }}}} 