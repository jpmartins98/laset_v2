/**
  ******************************************************************************
  * @file    main.c
  * @author  3S0 FreeRTOS nnd@isep.ipp.pt
  * @version V1.1
  * @date    28/11/2018
  * @brief   SISTR/SOTER FreeRTOS Example project
  ******************************************************************************
*/


/*
 *
 * LED blink
 *
 */

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

struct stReceiveI2C {
   char  values[15];
}stReceiveI2C;

 /* Configure RCC clock at 72 MHz */
//static void prvSetupRCC( void );

/* Configure RCC clock at 72 MHz */
static void prvSetupRCCHSI( void );  	//60hz

 /* Configure GPIO. */
static void prvSetupGPIO( void );

/* Simple LED toggle task. */
//static void prvFlashTask1( void *pvParameters );

/* Simple I2C2 read task. */
static void prvI2C2ReadTask( void *pvParameters );

/* Simple ADC read task. */
static void prvADCReadTask( void *pvParameters );


/********** Useful functions **********/
/* USART2 configuration. */
static void prvSetupUSART2( void );

/* I2C2 configuration. */
static void prvSetupI2C2( void );

/* DMA configuration. */
static void prvSetupDMA( void );

/* ADC configuration. */
static void prvSetupADC( void );

/* ADC configuration. */
static void prvSetupTIM2(void);

/* USART2 send message. */
static void prvSendMessageUSART2(char *message);

/* I2C2 read message. */

void prvSendMessageI2C2(uint8_t int8AddressI2C, uint8_t byte);

struct stReceiveI2C prvReceivesMessagesI2C2(uint8_t int8AddressI2C, uint8_t int8Iterations);


void prvADCChannels(void);

//uint16_t prvPressureCalulation(void);





/***************************************/


/* Task 1 handle variable. */
TaskHandle_t HandleTask1, HandleTask2;
uint16_t u16Pressure[2];



int main( void )
{
	/*Setup the hardware, RCC, GPIO, etc...*/
    //prvSetupRCC();
    prvSetupRCCHSI();
    prvSetupGPIO();
    prvSetupUSART2();
    prvSetupI2C2();
    prvSetupDMA();
    prvSetupADC();
    prvSetupTIM2();


    /* Create the tasks */
    //xTaskCreate( prvADCReadTask, "ADCRead", configMINIMAL_STACK_SIZE, NULL, 1, &HandleTask1 );
 	xTaskCreate( prvI2C2ReadTask, "I2C2Read", configMINIMAL_STACK_SIZE*3, NULL, 1, &HandleTask2 );


	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}
/*-----------------------------------------------------------*/



static void prvI2C2ReadTask( void *pvParameters )
{
	struct stReceiveI2C stReceiveData;
	uint8_t address= 0xC8;
	int mode=0;
    for( ;; )
	{
    	  switch ( mode )
    	  {
    	  	case 0 :
    	    mode=1;
    	    break;

    	    case 1 :
    	    prvSendMessageI2C2(address	,'R');
    	    mode=2;
    	    break;

    	    case 2 :
    	    stReceiveData=prvReceivesMessagesI2C2(address,11);
    	    char buf[11];
    	    sprintf(buf,"new read: ");
    	    prvSendMessageUSART2(buf);
    	    prvSendMessageUSART2(stReceiveData.values);
    	    mode=1;
    	    break;

    	  }
    	vTaskSuspend( NULL );
	}
}

/*-----------------------------------------------------------*/



static void prvADCReadTask( void *pvParameters )
{

    for( ;; )
	{

    	prvADCChannels();

	}
}
/*-----------------------------------------------------------*/

//static void prvFlashTask1( void *pvParameters )
//{
//    for( ;; )
//	{
//
//    	/* Block 1 second. */
//    	vTaskDelay( ( TickType_t ) 1000 / portTICK_PERIOD_MS  );
//
//    	/* Toggle the LED */
//    	GPIO_WriteBit(GPIOB, GPIO_Pin_0, ( 1-GPIO_ReadOutputDataBit( GPIOB, GPIO_Pin_0 ) ) );
//
//	}
//}
/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/



static void prvSetupRCCHSI( void )
{
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PCLK2Config(RCC_HCLK_Div1);

	RCC_HSICmd(ENABLE);
	FLASH_SetLatency(FLASH_Latency_2);



	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
	RCC_PLLCmd(ENABLE);
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	while(RCC_GetSYSCLKSource() != 0x08);
}
/*-----------------------------------------------------------*/


static void prvSetupGPIO( void )
{
    /* GPIO configuration */
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

}
/*-----------------------------------------------------------*/



static void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    /* Configure the USART2 */
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }

/*-----------------------------------------------------------*/

static void prvSetupI2C2( void )
{
I2C_InitTypeDef I2C_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* I2C2 is configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    /* I2C2 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    /*PB7 -> I2C2 SDA  & PB8 -> I2C2 SCL */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Initialize I2C */
    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_ClockSpeed = 800;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 1;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    /* Configure the I2C2 */
    I2C_Init(I2C2, &I2C_InitStructure);//I2Cx->I2C2

    /* Enable the i2c2 */
    I2C_Cmd(I2C2, ENABLE);//I2Cx->I2C2


 }

/*-----------------------------------------------------------*/
static void prvSetupDMA( void )
{
	//DMA_1
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);


	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr =  (uint32_t)u16Pressure;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //Better for SCAN and CONTINOUS ADC modes
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);
 }


/*-----------------------------------------------------------*/

static void prvSetupADC( void )
{
	ADC_InitTypeDef ADC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	// Initialize the ADC1 according to the ADC_InitStructure members

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2; // xyz
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_7Cycles5); //Z
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_7Cycles5); //Y
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_7Cycles5); //X

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);

	// Calibrar
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd ( ADC1 , ENABLE );

 }

/*-----------------------------------------------------------*/

static void prvSetupTIM2(void)
{
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseStructInit(&TIMER_InitStructure);

	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIMER_InitStructure.TIM_Prescaler = 1024;
	TIMER_InitStructure.TIM_Period = 62500;
	TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

}

/*-----------------------------------------------------------*/


/* This is a blocking send USART function */
static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;

    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}


/*-----------------------------------------------------------*/

void  prvSendMessageI2C2(uint8_t int8AddressI2C, uint8_t byte){
	int counter = 0;
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	//while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
	//vTaskDelay( ( TickType_t ) 600 / portTICK_PERIOD_MS  );
	I2C_GenerateSTART(I2C2, ENABLE);
	while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	I2C_Send7bitAddress(I2C2, int8AddressI2C, I2C_Direction_Transmitter);
	//vTaskDelay( ( TickType_t ) 1000 / portTICK_PERIOD_MS  );
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C2,byte);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C2, ENABLE);
}

/*-----------------------------------------------------------*/

struct stReceiveI2C prvReceivesMessagesI2C2(uint8_t int8AddressI2C, uint8_t int8Iterations){
	struct stReceiveI2C stReceiveData;
	int i;
	if (int8Iterations <= 15){
		I2C_GenerateSTART(I2C2,ENABLE);
		while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)) );
		I2C_Send7bitAddress(I2C2, int8AddressI2C, I2C_Direction_Receiver);
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		//while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		//char val = I2C_ReceiveData(I2C2);
		for ( i=0; i<=int8Iterations; i++){
			//vTaskDelay( ( TickType_t ) 300 / portTICK_PERIOD_MS  );
			while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
			stReceiveData.values[i]=I2C_ReceiveData(I2C2);
		}
		I2C_AcknowledgeConfig(I2C2, DISABLE);
		I2C_GenerateSTOP(I2C2, ENABLE);
		while(I2C_GetFlagStatus(I2C2,I2C_FLAG_STOPF));
		return stReceiveData;
	}
		else{
			return stReceiveData;
		}
}


/*-----------------------------------------------------------*/

void prvADCChannels(void){
		char buf[30];
		while(DMA_GetFlagStatus(DMA1_FLAG_TC1) == RESET);
		DMA_ClearFlag(DMA1_FLAG_TC1);
		//while(ADC_GetFlagStatus(ADC_FLAG_STRT) == RESET);
		//ADC_ClearFlag(ADC_FLAG_STRT);
		sprintf(buf, "x-> %d mg, y-> %d mg \r\n",u16Pressure[0],u16Pressure[1]);
		prvSendMessageUSART2(buf);
}

/*-----------------------------------------------------------*/



