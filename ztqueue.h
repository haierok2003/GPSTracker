#ifndef __ZTQUEUE_H__
#define __ZTQUEUE_H__
#include "ztdatatype.h"
#include "kal_general_types.h"

extern void initQueue(LinkQueue *_pQueue);
extern kal_bool QueueEmpty(LinkQueue *_pQueue);
extern void EnQueue(LinkQueue* _pQueue,void* data);
extern void* DeQueue(LinkQueue* _pQueue);
extern kal_bool ReadQueue(LinkQueue* _pQueue,void* data);
extern void* QueueFront(LinkQueue* _pQueue);
extern int QueueLen(LinkQueue* _pQueue);
#endif
