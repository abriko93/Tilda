/**
  ******************************************************************************
  * @file    app.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides all the Application firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/ 

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"



//*********************************************************************
//USB

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
   
__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

volatile extern uint8_t APP_Rx_Buffer []; 
volatile extern uint32_t APP_Rx_ptr_in; 
extern uint8_t mybyte = 0;


extern uint16_t ADD_VCP_ByteIn(uint8_t *Buf) 
{

  APP_Rx_Buffer[APP_Rx_ptr_in] = *Buf;
  APP_Rx_ptr_in++;
  if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
  {
    APP_Rx_ptr_in = 0;
  } 

  return USBD_OK;
}

//*********************************************************************




//*********************************************************************
//DAC
#define DAC_DHR12R2_ADDRESS    0x40007414
#define DAC_DHR8R1_ADDRESS     0x40007410
DAC_InitTypeDef  DAC_InitStructure;


const uint16_t aSine12bit[24] = { 4095, 4025, 3821, 3495, 3071, 2577, 2048,
1518, 1024, 600, 274, 70, 0, 70, 274, 600, 1024, 1518, 2047, 2577, 3071, 3495,
3821, 4025 };
const uint16_t aCos12bit[24] = { 4095, 4025, 3821, 3495, 3071, 2577, 2048,
1518, 1024, 600, 274, 70, 0, 70, 274, 600, 1024, 1518, 2047, 2577, 3071, 3495,
3821, 4025 };

const uint8_t aCos8bit[24] = { 
  191, 160, 128, 95, 64, 37,
17, 4,
  0, 4, 17, 37, 64, 95, 127, 160, 191,
  218, 238, 251, 255, 251, 238, 218};


static void TIM_Config(void);
static void TIM1_Config(void);

static void DAC_Ch1_CosWaveConfig(void);
static void DAC_Ch2_SineWaveConfig(void);
//*********************************************************************

//*********************************************************************
//SPI
void SPI2_Init(void);
uint8_t reo1_byte1 = 255;
uint8_t reo1_byte2 = 255;
uint8_t reo1_byte3 = 255;
uint8_t reo2_byte1 = 255;
uint8_t reo2_byte2 = 255;
uint8_t reo2_byte3 = 255;
uint8_t byte2_1 = 255;
uint8_t byte2_2 = 255;
uint8_t byte2_3 = 255;
uint8_t reo_byte1=255;
uint8_t reo_byte2=255;
uint8_t reo_byte3=255;
int32_t reo = 0;
int32_t reo1 = 0;
int32_t reo2 = 0;
uint32_t data = 0;

uint8_t key = 0x00; 


uint8_t iii = 0x00; 


void spi_send(uint8_t db) {
    SPI_I2S_SendData(SPI2, db); //Передаем байт через SPI2
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);  // ждём пока данные уйдут
    //while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == SET) //Передатчик занят?
; // значит ничего не делаем
}
uint8_t spi_recieve() {
  uint8_t received = 0;
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  // ждём пока данные появтся
  //if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == SET) { //Если данные пришли
    received = SPI_I2S_ReceiveData(SPI2); //Читаем принятые данные  
  //}  
  return received;
}


void _delay_us(uint16_t delay)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
   TIM7->PSC = 15-1;
    TIM7->ARR = delay;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->CR1 = TIM_CR1_CEN|TIM_CR1_OPM;
    while ((TIM7->CR1 & TIM_CR1_CEN)!=0);
 }
void _delay_1(uint16_t delay)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
   TIM7->PSC = 1-1;
    TIM7->ARR = delay;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->CR1 = TIM_CR1_CEN|TIM_CR1_OPM;
    while ((TIM7->CR1 & TIM_CR1_CEN)!=0);
 }
//*********************************************************************

       int main(void)
{
  __IO uint32_t i = 0;  
  __enable_interrupt();
 
  
  
  USBD_Init(&USB_OTG_dev,          
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);
  
  
  
  
  //*********************************************************************
  //DAC
  
  /* Preconfiguration before using DAC----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* GPIOA clock enable (to be used with DAC) */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                         
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  /* DAC channel 1 & 2 (DAC_OUT1 = PA.4)(DAC_OUT2 = PA.5) configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* TIM6 Configuration ------------------------------------------------------*/
  TIM_Config();
    
  DAC_DeInit(); 
  DAC_Ch1_CosWaveConfig();
  //DAC_Ch2_SineWaveConfig();
  //*********************************************************************

  //*********************************************************************  
// Отключаем дисплей
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
GPIO_InitTypeDef  ledinit;  // создаем структуру
ledinit.GPIO_Mode = GPIO_Mode_OUT;  // направление - выход
ledinit.GPIO_OType = GPIO_OType_PP;  // Двухтактный выход
ledinit.GPIO_PuPd = GPIO_PuPd_NOPULL;  // Без подтяжки
ledinit.GPIO_Speed = GPIO_Speed_2MHz;  // Скорость низкая
ledinit.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_9 | GPIO_Pin_8;
GPIO_Init(GPIOE, &ledinit);  // Функция, выполняющая настройку портов
GPIO_ResetBits(GPIOE, GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_9 | GPIO_Pin_8);
  //********************************************************************* 
 


//*********************************************************************  
// SPI
SPI2_Init();
  // NSS
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_SetBits(GPIOA, GPIO_Pin_7);
//GPIO_ResetBits(GPIOC, GPIO_Pin_2);
  //********************************************************************* 
uint8_t ii = 0;


//uint8_t byte1_1 = 255;
//uint8_t byte1_2 = 255;
//uint8_t byte1_3 = 255;
//uint8_t byte2_1 = 255;
//uint8_t byte2_2 = 255;
//uint8_t byte2_3 = 255;
//int32_t reo1 = 0;

//byte1_3 = (byte1_3 & 0xe0);
//reo1 =  0xffffc000 & ((byte1_1 << 25) | (byte1_2 << 17) | (byte1_3 << 9));

//byte1_3 =  reo1 >> 10;
  /* SysTick Configuration */


  TIM1_Config();

  /* Main loop */
  while (1)
  { 

    if(key==1)
    {
      /*
    //ii++;
    //ADD_VCP_ByteIn(&ii);

    spi_send(0xFF);
    reo1_byte1 = spi_recieve();
    
    spi_send(0xFF);
    reo1_byte2 = spi_recieve();
    
    spi_send(0xFF);
    reo1_byte3 = spi_recieve();
    reo1_byte3 = (reo1_byte3 & 0xe0);
    reo1 =  0xffffc000 & ((reo1_byte1 << 25) | (reo1_byte2 << 17) | (reo1_byte3 << 9));
    */
    /*
    reo_byte3 = reo1 >> 9;
    reo_byte2 = reo1 >> 17;
    reo_byte1 = reo1 >> 25;
    ADD_VCP_ByteIn(&reo_byte1);
    ADD_VCP_ByteIn(&reo_byte2);
    ADD_VCP_ByteIn(&reo_byte3);
    */
    key=2;
    }
    
    if(key==3)
    {
//    ii++;
    //ADD_VCP_ByteIn(&ii);
/*    spi_send(0xFF);
    reo2_byte1 = spi_recieve();
    
    spi_send(0xFF);
    reo2_byte2 = spi_recieve();
    
    spi_send(0xFF);
    reo2_byte3 = spi_recieve();
    reo2_byte3 = (reo2_byte3 & 0xe0);
    reo2 =  0xffffc000 & ((reo2_byte1 << 25) | (reo2_byte2 << 17) | (reo2_byte3 << 9));
    reo = reo2-reo1;
    reo_byte3 = reo >> 9;
    reo_byte2 = reo >> 17;
    reo_byte1 = reo >> 25; */
  //  ADD_VCP_ByteIn(&reo_byte1);
  //  ADD_VCP_ByteIn(&reo_byte2);
  //  ADD_VCP_ByteIn(&reo_byte3); 
    key=4;
    }
    
    if(key==4)
    {
    
   // GPIO_SetBits(GPIOC, GPIO_Pin_2);
   // _delay_us(1);
   // GPIO_ResetBits(GPIOC, GPIO_Pin_2);
      
   // spi_send(0xFF);
   // byte2_1 = spi_recieve();
    
   // spi_send(0xFF);
   // byte2_2 = spi_recieve();
    
   // spi_send(0xFF);
   // byte2_3 = spi_recieve();

   // ADD_VCP_ByteIn(&byte2_1);
  //  ADD_VCP_ByteIn(&byte2_2);
  //  ADD_VCP_ByteIn(&byte2_3);
    
    key=0;
    }
  
  }
} 

static void TIM1_Config(void)
{ 
  uint32_t pulse = 64*1; //68/2*2.5;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* GPIOA clocks enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA Configuration: Channel 1*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Connect TIM pins to AF1 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    
  /* Enable the TIM1 Trigger and commutation interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef       TIM_OCInitStructure;
  
  
/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 2400-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = pulse-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse-10-1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  /* Clear TIM1 COM pending bit */
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    iii++;
    ADD_VCP_ByteIn(&iii);
    
    spi_send(0xFF);
    reo1_byte1 = spi_recieve();
    
    spi_send(0xFF);
    reo1_byte2 = spi_recieve();
    
    spi_send(0xFF);
    reo1_byte3 = spi_recieve();
    
    reo1_byte3 = (reo1_byte3 & 0xe0);
    reo1 =  0xffffc000 & ((reo1_byte1 << 25) | (reo1_byte2 << 17) | (reo1_byte3 << 9));
    
    reo_byte3 = reo1 >> 9;
    reo_byte2 = reo1 >> 17;
    reo_byte1 = reo1 >> 25;
    

    
    ADD_VCP_ByteIn(&reo_byte1);
    ADD_VCP_ByteIn(&reo_byte2);
    ADD_VCP_ByteIn(&reo_byte3);
    
    ADD_VCP_ByteIn(&reo2_byte1);
    ADD_VCP_ByteIn(&reo2_byte2);
    ADD_VCP_ByteIn(&reo2_byte3);
    
    // GPIO_SetBits(GPIOA, GPIO_Pin_7);
    //_delay_us(1);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    //GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    
      /*
    if(key==0)
    {
    //GPIO_SetBits(GPIOC, GPIO_Pin_1);
      GPIO_SetBits(GPIOA, GPIO_Pin_7);
    _delay_us(1);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    key=1;
    }
    if(key==2)
    {
    //GPIO_SetBits(GPIOC, GPIO_Pin_1);
      GPIO_SetBits(GPIOA, GPIO_Pin_7);
    _delay_us(1);
    //GPIO_ResetBits(GPIOC, GPIO_Pin_1);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    key=3;
    }
    */
    
    }
}

static void TIM_Config(void)
{
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  /* TIM Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM3, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 34-1;          //34
  TIM_TimeBaseStructure.TIM_Prescaler = 1-1;       //0
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  /* TIM TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
  
  /* TIM enable counter */
  TIM_Cmd(TIM6, ENABLE);
}

static void DAC_Ch2_SineWaveConfig(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  
  /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* DMA1_Stream6 channel7 configuration **************************************/
  DMA_DeInit(DMA1_Stream6);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&aSine12bit;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 24;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

  /* Enable DMA1_Stream6 */
  DMA_Cmd(DMA1_Stream6, ENABLE);

  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /* Enable DMA for DAC Channel2 */
  DAC_DMACmd(DAC_Channel_2, ENABLE);
}

static void DAC_Ch1_CosWaveConfig(void)
{
  DMA_InitTypeDef DMA_InitStructure;

  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* DMA1_Stream5 channel7 configuration **************************************/  
  DMA_DeInit(DMA1_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_7;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR8R1_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&aCos8bit;
  DMA_InitStructure.DMA_BufferSize = 24;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);    

  /* Enable DMA1_Stream5 */
  DMA_Cmd(DMA1_Stream5, ENABLE);
  
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /* Enable DMA for DAC Channel1 */
  DAC_DMACmd(DAC_Channel_1, ENABLE);
}


void SPI2_Init(void)
{

GPIO_InitTypeDef GPIO_InitStructure;
SPI_InitTypeDef SPI_InitStructure;

// Тактирование модуля SPI и порта
RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

// Настраиваем ноги SPI для работы в режиме альтернативной функции
GPIO_PinAFConfig(GPIOB,GPIO_PinSource14, GPIO_AF_SPI2);
GPIO_PinAFConfig(GPIOC,GPIO_PinSource3,GPIO_AF_SPI2);
GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_SPI2);

GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
GPIO_Init(GPIOC, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_10;
GPIO_Init(GPIOB, &GPIO_InitStructure);

//Заполняем структуру с параметрами SPI модуля
SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //полный дуплекс
SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // передаем по 8 бит
SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Полярность и
SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // фаза тактового сигнала
SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; // Управлять состоянием сигнала NSS программно
SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //32 Предделитель SCK
SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // Режим - мастер
SPI_Init(SPI2, &SPI_InitStructure); //Настраиваем SPI2
SPI_Cmd(SPI2, ENABLE); // Включаем модуль SPI2....

// Поскольку сигнал NSS контролируется программно, установим его в единицу
// Если сбросить его в ноль, то наш SPI модуль подумает, что
// у нас мультимастерная топология и его лишили полномочий мастера.

SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
}


#ifdef USE_FULL_ASSERT
/**
* @brief  assert_failed
*         Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  File: pointer to the source file name
* @param  Line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @}
  */ 


/**
  * @}
  */ 


/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
