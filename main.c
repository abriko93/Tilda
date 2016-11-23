#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "Queue_Array.h"
#include "stdbool.h"
#include "math.h"

//Переменные очереди
QueueArray F;
queueArrayBaseType b;

//Переменные USART
_Bool  USART_newdata = false;
_Bool  USART_txready = true;
uint8_t USART_receivedData;
uint8_t USART_sendData;

//Используемые функции
void USART1_Config();
void TIM5_Config();
void State_change_function(uint8_t SC_data);
void DAC_Ch1_Config();
void DAC_Ch2_Config();

//Счетчик для TIM (для отладки)
uint8_t i=0;
_Bool  key = true;

//Переменные DAC
#define DAC_DHR12R2_ADDRESS    0x40007414
#define DAC_DHR8R1_ADDRESS     0x40007410
DAC_InitTypeDef  DAC_InitStructure;
uint8_t Sinus8bit[21];
uint8_t Sinus_i; 


int main()
{
    // Включаем прерывания
    __enable_irq();
    DAC_Ch1_Config();
    DAC_Ch2_Config();

    // Включаем таймер 5
    TIM5_Config();

    // Вызываем функцию инициализации USART
    USART1_Config();
   
    //Обнуляем очередь
    initQueueArray(&F);

    //TEST
       /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /* Set DAC channel2 DHR12RD register */
  DAC_SetChannel2Data(DAC_Align_12b_R, 0x100);
   
    while(1)
    {
      //  __NOP();    
      
    while (isEmptyQueueArray(&F) != 1)
      {
      if (USART_txready == true)
        {
        getQueueArray(&F, &b);
        USART_SendData(USART1, b);
        USART_txready = false;
        }
      }
      
    }
}

//Функция изменения состояния
void State_change_function(uint8_t SC_data)
{
     putQueueArray(&F, SC_data);
}

// Запускаем DAC1, DMA, TIM
 void DAC_Ch1_Config()
 {
  for(Sinus_i=0; Sinus_i<21; Sinus_i++)
  {
    Sinus8bit[Sinus_i] = (uint8_t) lround((sin(Sinus_i*(2*3.14159/21)) + 1)*255/2);
  }
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
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  /* TIM Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 40-1;          //42
  TIM_TimeBaseStructure.TIM_Prescaler = 1-1;       //0
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  /* TIM TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
  
  /* TIM enable counter */
  TIM_Cmd(TIM6, ENABLE);
     
  DAC_DeInit(); 
  
  /* DAC_WaveConfig */
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
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&Sinus8bit;
  DMA_InitStructure.DMA_BufferSize = 21;
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

// Запускаем DAC1, DMA, TIM
 void DAC_Ch2_Config()
 {
  /* Preconfiguration before using DAC----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

  /* DMA1 clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* GPIOA clock enable (to be used with DAC) */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);                         
  /* DAC Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  /* DAC channel 1 & 2 (DAC_OUT1 = PA.4)(DAC_OUT2 = PA.5) configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* TIM6 Configuration ------------------------------------------------------*/
  //TIM6_Config(); 
    
} 

// Инициализация USART
void USART1_Config()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    // Запускаем тактирование
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
 
    // Инициализация нужных пинов контроллера, для USART2 – 
    // PA9 и PA10
    GPIO_StructInit(&GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
 
   // А теперь настраиваем модуль USART
   USART_StructInit(&usart);
   usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
   usart.USART_BaudRate = 9600;	
   USART_Init(USART1, &usart);	
 
    //Кофигурируем NVIC прерывания
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART1, ENABLE);
    
    // Включаем прерывание по окончанию передачи
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    // Включаем прерывание по приему данных
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    //Подаем питание на модуль Bluetooth
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_7); //ON
    //GPIO_SetBits(GPIOA, GPIO_Pin_7); //OFF
        
    //Подаем питание на модуль FTDI
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_4); //ON
    //GPIO_SetBits(GPIOC, GPIO_Pin_4); //OFF
}

void TIM5_Config()
{
    TIM_TimeBaseInitTypeDef timer;
    //Включаем тактирование таймера TIM5
    //Таймер 4 у нас висит на шине APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    //Заполняем поля структуры дефолтными значениями
    TIM_TimeBaseStructInit(&timer);
    //Выставляем предделитель
    timer.TIM_Prescaler = 4200-1;
    //Тут значение, досчитав до которого таймер сгенерирует прерывание
    //Кстати это значение мы будем менять в самом прерывании
    timer.TIM_Period = 2000-1; //Частота 100Гц
    //Инициализируем TIM5 нашими значениями
    TIM_TimeBaseInit(TIM5, &timer);	
    //Настраиваем таймер для генерации прерывания по обновлению (переполнению)
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    //Запускаем таймер 
    TIM_Cmd(TIM5, ENABLE);
    //Разрешаем соответствующее прерывание
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
   
}


void TIM5_IRQHandler()
{	
     TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
     
     putQueueArray(&F, i++);
        
     if (key)
        {
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        key = false;      
        }
     else
     {
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        key = true;   
     }
     
}

void USART1_IRQHandler()
{
      // Убеждаемся, что прерывание вызвано новыми данными в регистре данных
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // Чистим флаг прерывания
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
        // Принимаем данные
        USART_receivedData  = USART_ReceiveData(USART1);
        
        // Передаем данные в функцию изменения состояния
        State_change_function(USART_receivedData);
        
        USART_newdata = true;
        
        
        
    }
      
     // Проверяем, действительно ли прерывание вызвано окончанием передачи
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        // Очищаем флаг прерывания 
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        
        USART_txready = true; 
    }
} 





/*******************************************************************/

