/****************************************/
/*			Очередь на массиве			*/
/****************************************/
#include "stdio.h"
#include "Queue_Array.h"
#include "stdint.h"

/*Переменная ошибок*/
uint8_t  errorQueueArray;

/*Инициализация очереди*/
void initQueueArray(QueueArray *F) {
	F->ukBegin = 0;
	F->ukEnd = 0;
	F->len = 0;
	errorQueueArray = okQueueArray;
}

/*Включение в очередь*/
void putQueueArray(QueueArray *F, queueArrayBaseType E) {

	/*Если очередь переполнена*/
	if (isFullQueueArray(F)) {
		return;
	}
	/*Иначе*/
	F->buf[F->ukEnd] = E;									// Включение элемента
	F->ukEnd = (F->ukEnd + 1) % SIZE_QUEUE_ARRAY;			// Сдвиг указателя
	F->len++;												// Увеличение количества элементов очереди

}

/*Исключение из очереди*/
void getQueueArray(QueueArray *F, queueArrayBaseType *E) {

	/*Если очередь пуста*/
	if (isEmptyQueueArray(F) == 0) 
        {
	*E = F->buf[F->ukBegin];								// Запись элемента в переменную
	F->ukBegin = (F->ukBegin + 1) % SIZE_QUEUE_ARRAY;		// Сдвиг указателя
	F->len--;
        }												// Уменьшение длины
}

/*Предикат: полна ли очередь*/
uint8_t  isFullQueueArray(QueueArray *F) {
	if (F->len == SIZE_QUEUE_ARRAY) {
		errorQueueArray = fullQueueArray;
		return 1;
	}
	return 0;
}

/*Предикат: пуста ли очередь*/
uint8_t  isEmptyQueueArray(QueueArray *F) {
	if (F->len == 0) {
		errorQueueArray = emptyQueueArray;
		return 1;
	}
	return 0;
}

