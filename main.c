#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "Queue_Array.h"
#include "stdbool.h"

//Переменные очереди
QueueArray F;
queueArrayBaseType b;

//Переменные USART
_Bool  USART_newdata = false;
_Bool  USART_txready = true;
uint8_t USART_receivedData;
uint8_t USART_sendData;

//Используемые процедуры
void USART1_Config();
void TIM4_Config(void);

int main()
{
    // Включаем прерывания
    __enable_irq();
 

    // Вызываем функцию инициализации USART
    USART1_Config();
   
    //Обнуляем очередь
    initQueueArray(&F);

   // TIM4_Config();
    
   
    
    while(1)
    {
    // А тут мы ничего не делаем, вся работа у нас в прерывании
        __NOP();
      /*
    while (isEmptyQueueArray(&F) != 1)
      {
      if (USART_txready == true)
        {
        getQueueArray(&F, &b);
        USART_SendData(USART2, b);
        USART_txready = false;
        }
      }
      */
    }
}


// Инициализация USART
void USART1_Config()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    // Запускаем тактирование
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
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
   usart.USART_BaudRate = 256000;	
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
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    //GPIO_SetBits(GPIOA, GPIO_Pin_7);
        
    //Подаем питание на модуль FTDI
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOC, GPIO_Pin_4);
    //GPIO_SetBits(GPIOC, GPIO_Pin_4);
}

void TIM4_Config(void)
{
    TIM_TimeBaseInitTypeDef timer;
    //Включаем тактирование таймера TIM4
    //Таймер 4 у нас висит на шине APB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //Заполняем поля структуры дефолтными значениями
    TIM_TimeBaseStructInit(&timer);
    //Выставляем предделитель
    timer.TIM_Prescaler = 300-1;
    //Тут значение, досчитав до которого таймер сгенерирует прерывание
    //Кстати это значение мы будем менять в самом прерывании
    timer.TIM_Period = 2000-1; //Частота 100Гц
    //Инициализируем TIM4 нашими значениями
    TIM_TimeBaseInit(TIM4, &timer);	
    //Настраиваем таймер для генерации прерывания по обновлению (переполнению)
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Запускаем таймер 
    TIM_Cmd(TIM4, ENABLE);
    //Разрешаем соответствующее прерывание
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
   
}


void TIM4_IRQHandler()
{	
     TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

}

void USART1_IRQHandler()
{
      // Убеждаемся, что прерывание вызвано новыми данными в регистре данных
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        // Чистим флаг прерывания
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        
        // То ради чего все затеяли – принимаем данные )
        USART_receivedData  = USART_ReceiveData(USART1);

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

