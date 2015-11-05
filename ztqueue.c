#include "ztqueue.h"
#include "kal_general_types.h"
#include "mmi_frm_mem_gprot.h"

void initQueue(LinkQueue *_pQueue)
{
	_pQueue->front=_pQueue->rear=NULL;
}
/*�жϿ�*/
kal_bool QueueEmpty(LinkQueue *_pQueue)
{
	return _pQueue->front == NULL;
}
/*���*/
void EnQueue(LinkQueue* _pQueue,void* data)
{
	QueueNode* qn = (QueueNode*)get_ctrl_buffer(sizeof(QueueNode));
	qn->data = data;
	qn->next = NULL;
	if(_pQueue->front == NULL){	//����������֮ǰ�����ǿյ�
		_pQueue->front=_pQueue->rear=qn;
		return;
	}
	_pQueue->rear->next=qn;
	_pQueue->rear = qn;
}
/*����*/
void* DeQueue(LinkQueue* _pQueue)
{	
	QueueNode* tempQueue;
	void* data;

	if(QueueEmpty(_pQueue))
	{//���ǿյ�
		return NULL;
	}
	if(_pQueue->front==_pQueue->rear)
	{//����ֻ��һ��Ԫ��
		data = _pQueue->front->data;
		free_ctrl_buffer(_pQueue->front);
		_pQueue->front = _pQueue->rear = NULL;
	}
	else
	{
		data = _pQueue->front->data;
		tempQueue = _pQueue->front->next;
		free_ctrl_buffer(_pQueue->front);
		_pQueue->front = tempQueue;
	}

	return data;
}
/*���ζ�ȡ����Ԫ�ص�����*/
kal_bool ReadQueue(LinkQueue* _pQueue,void* data)
{
	if(_pQueue->front == _pQueue->rear)
	{//���ǿյ�
		data = _pQueue->front->data;
		return KAL_FALSE;
	}
	
	data = _pQueue->front->data;
	_pQueue->front=_pQueue->front->next;
	return KAL_TRUE;
}
/*ȡ��ͷԪ��*/
void* QueueFront(LinkQueue* _pQueue)
{
	if(QueueEmpty(_pQueue))
	{//���ǿյ�
		return NULL;
	}
	return _pQueue->front->data;
}

int QueueLen(LinkQueue* _pQueue)
{
	QueueNode* q = _pQueue->front;
	int length = 0;

	while(q != NULL)
	{
		length++;
		q = q->next;
	}

	return length;
}
