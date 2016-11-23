/****************************************/
/*			������� �� �������			*/
/****************************************/
#include "stdio.h"
#include "Queue_Array.h"
#include "stdint.h"

/*���������� ������*/
uint8_t  errorQueueArray;

/*������������� �������*/
void initQueueArray(QueueArray *F) {
	F->ukBegin = 0;
	F->ukEnd = 0;
	F->len = 0;
	errorQueueArray = okQueueArray;
}

/*��������� � �������*/
void putQueueArray(QueueArray *F, queueArrayBaseType E) {

	/*���� ������� �����������*/
	if (isFullQueueArray(F)) {
		return;
	}
	/*�����*/
	F->buf[F->ukEnd] = E;									// ��������� ��������
	F->ukEnd = (F->ukEnd + 1) % SIZE_QUEUE_ARRAY;			// ����� ���������
	F->len++;												// ���������� ���������� ��������� �������

}

/*���������� �� �������*/
void getQueueArray(QueueArray *F, queueArrayBaseType *E) {

	/*���� ������� �����*/
	if (isEmptyQueueArray(F) == 0) 
        {
	*E = F->buf[F->ukBegin];								// ������ �������� � ����������
	F->ukBegin = (F->ukBegin + 1) % SIZE_QUEUE_ARRAY;		// ����� ���������
	F->len--;
        }												// ���������� �����
}

/*��������: ����� �� �������*/
uint8_t  isFullQueueArray(QueueArray *F) {
	if (F->len == SIZE_QUEUE_ARRAY) {
		errorQueueArray = fullQueueArray;
		return 1;
	}
	return 0;
}

/*��������: ����� �� �������*/
uint8_t  isEmptyQueueArray(QueueArray *F) {
	if (F->len == 0) {
		errorQueueArray = emptyQueueArray;
		return 1;
	}
	return 0;
}

