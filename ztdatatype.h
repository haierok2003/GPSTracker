#ifndef __ZT_DATATYPE_H__
#define __ZT_DATATYPE_H__

typedef struct Node{
	void* data;
	struct Node* next;
}QueueNode;

typedef struct queue{
	QueueNode *front,*rear;
}LinkQueue;

#endif