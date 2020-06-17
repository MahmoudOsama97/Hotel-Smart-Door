//#include "i2c.h"

//to do :
//resolve header file problem
//send i2c number (recognize the port and pins automatically) in initcommunication , by base address and offset
//may integrate initcomm with initmaster
//PCTL problem
//some #defines



//void I2C_initCommunication(){
//	SYSCTL_RCGCI2C_R = (1<<1);
//	while((SYSCTL_PRI2C_R & (1<<1))==0);	
//	GPIO_initPin(PORTA,PIN6,DIGITAL,PERIPHERAL);
//	GPIO_initPin(PORTA,PIN7,DIGITAL,PERIPHERAL);
//	GPIO_setPullup(PORTA,PIN6);
//	GPIO_setPullup(PORTA,PIN7);
//	GPIO_setOpenDrain(PORTA,PIN7);
////	GPIO_setPCTL(PORTA,PIN6,I2C_F);
////	GPIO_setPCTL(PORTA,PIN7,I2C_F);
//	GPIO_PORTA_PCTL_R &= ~(0xFF000000);
//	GPIO_PORTA_PCTL_R = (3<<28)|(3<<24);

//}


//void I2C_initMaster(uint8_t mode){
//	I2C1_MCR_R = (1<<4);
//	I2C1_MTPR_R = 7;
//	if(mode==0) I2C1_MSA_R &= ~(1<<0);  //transmit	
//	else I2C1_MSA_R |= (1<<0);  //receive
//}

//void I2C_setSlaveAddress(uint8_t address){
//	I2C1_MSA_R = (address<<1);
//}

//void I2C_sendChar(char x){
//	I2C1_MDR_R = x;
//	I2C1_MCS_R = 0x07;
//	while((I2C1_MCS_R & (1<<0)) != 0 );
//	for(int j =0 ; j<100 ; j++);
//}

//void I2C_sendString(char* data){
//int i=0;
//	do
//	{
//		I2C_sendChar(data[i]);
//		i++;
//	}while(data[i]!=0);
//	I2C_sendChar('#');  //to mark the end of transaction
//}