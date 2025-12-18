# stack_queue

```cpp
#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

//#define N 10
//struct stack
//{
//    int a[N];
//    int top;
//};

typedef int STDataType;

typedef struct Stack
{
    STDataType* a;
    int top;
    int capacity;
}ST;

void STInit(ST* ps);
void STDestroy(ST* ps);

void STPush(ST* ps, STDataType x);
void STPop(ST* ps);
int STSize(ST* ps);
bool STEmpty(ST* ps);
STDataType STTop(ST* ps);
```

```cpp
#include"Stack.h"



void STInit(ST* ps)
{
    if (ps == NULL)
    {
        printf("Stact.c: %d", __LINE__);
        return;
    }
    ps->a = (STDataType*)malloc(sizeof(STDataType) * 4);
    ps->capacity = 4;
    ps->top = 0;
}


void STDestroy(ST* ps)
{
    if (ps == NULL)
    {
        printf("Stact.c: %d", __LINE__);
        return;
    }
    free(ps->a);
    ps->a = NULL;
}

void STPush(ST* ps, STDataType x)
{
    if (ps == NULL)
    {
        printf("Stact.c: %d", __LINE__);
        return;
    }
    if (ps->top == ps->capacity)
    {
        STDataType* p = (STDataType*)realloc(ps->a, sizeof(STDataType) * (ps->capacity * 2));
        if (p == NULL)
        {
            printf("Stact.c: %d", __LINE__);
            return;
        }
        ps->a = p;
        ps->capacity = ps->capacity * 2;
    }
    (ps->a)[ps->top] = x;
    (ps->top)++;
}

void STPop(ST* ps)
{
    if (ps->top == 0)
    {
        printf("Stact.c: %d", __LINE__);
        return;
    }
    (ps->top)--;
}
int STSize(ST* ps)
{
    return ps->top;
}
bool STEmpty(ST* ps)
{
    return (ps->top == 0);
}
STDataType STTop(ST* ps)
{
    if (ps->top == 0)
    {
        printf("Stact.c: %d", __LINE__);
        return 0;
    }
    return (ps->a)[ps->top - 1];
}
```

```cpp
#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<assert.h>

typedef int QDatatype;

typedef struct QueueNode
{
    struct QueueNode* next;
    QDatatype data;
}QNode;


typedef struct Queue
{
    QNode* head;
    QNode* tail;
    int size;
}Queue;


void Queueinit(Queue* pq);
void QueueDestroy(Queue* pq);
void QueuePush(Queue* pq, QDatatype x);
void QueuePop(Queue* pq);
int QueueSize(Queue* pq);
bool QueueEmpty(Queue* pq);
QDatatype QueueFront(Queue* pq);
QDatatype QueueBack(Queue* pq);
```

```cpp
#include"Queue.h"

QNode* BuyNode()
{
    QNode* q = (QNode*)malloc(sizeof(QNode));
    assert(q);
    q->next = NULL;
    return q;
}

void Queueinit(Queue* pq)
{
    assert(pq);
    pq->head = NULL;
    pq->tail = NULL;
    pq->size = 0;
}
void QueueDestroy(Queue* pq)
{
    assert(pq);
    QNode* cur = pq->head;
    while (cur != NULL)
    {
        QNode* tmp = cur->next;
        free(cur);
        cur = tmp;
    }
    pq->head = NULL;
    pq->tail = NULL;
    pq->size = 0;
}



void QueuePush(Queue* pq, QDatatype x)
{
    assert(pq);
    QNode* newnode = BuyNode();
    newnode->data = x;
    if (pq->head == NULL)
    {
        assert(pq->tail == NULL);
        pq->head = newnode;
        pq->tail = newnode;
    }
    else
    {
        pq->tail->next = newnode;
        pq->tail = newnode;
    }
    pq->size++;
}



void QueuePop(Queue* pq)
{
    assert(pq);
    if (pq->size == 0)
    {
        printf("already empty");
        return;
    }
    else if (pq->size == 1)
    {
        QNode* tmp = pq->head;
        pq->head = NULL;
        pq->tail = NULL;
        free(tmp);
    }
    else
    {
        QNode* tmp = pq->head;
        pq->head = pq->head->next;
        free(tmp);
    }
    pq->size--;
}



int QueueSize(Queue* pq)
{
    assert(pq);
    return pq->size;
}
bool QueueEmpty(Queue* pq)
{
    assert(pq);
    return pq->size == 0;
}
QDatatype QueueFront(Queue* pq)
{
    assert(pq);
    assert(!QueueEmpty(pq));
    return pq->head->data;
}
QDatatype QueueBack(Queue* pq)
{
    assert(pq);
    assert(!QueueEmpty(pq));
    return pq->tail->data;
}
```

```cpp
#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

typedef int SLTDataType;

typedef struct SListNode
{
    SLTDataType data;
    struct SListNode* next;
}SLTNode;




void SLTPrint(SLTNode* phead);
void SLTPushBack(SLTNode** pphead, SLTDataType x);
void SLTPushFront(SLTNode** pphead, SLTDataType x);
void SLTPopBack(SLTNode** pphead);
void SLTPopFront(SLTNode** pphead);

//单链表查找
SLTNode* SLTFind(SLTNode* phead, SLTDataType x);
//pos之前插入
void SLTInsert(SLTNode** pphead, SLTNode* pos, SLTDataType x);
//pos位置删除
void SListErase(SLTNode** pphead, SLTNode* pos);
//pos后面插入
void SListInsertAfter(SLTNode* pos, SLTDataType x);
//pos位置后面删除
void SListEraseAfter(SLTNode* pos);
//链表的销毁
void SLTDistroy(SLTNode** pphead);
//链表反转
void Reverse(SLTNode** pphead);
//从小到大排序
SLTNode* Sort2(SLTNode* phead1, SLTNode* phead2);
//将比这个值大的放在前面且顺序不变
SLTNode* Sort2x(SLTNode* phead, int x);
//判断是否是回文结构
bool Noon(SLTNode* phead);
//判断两个链表是否交互于一点
bool Intersect(SLTNode* phead1, SLTNode* phead2, SLTNode** pphead);
//判断链表是否带环
bool Circle(SLTNode* phead);
//求环的长度
int CircleLength(SLTNode* phead);
//求入环的第一个节点
SLTNode* CircleFirstNode(SLTNode* phead);
//求入环的第一个节点2
SLTNode* CircleFirstNode2(SLTNode* phead);
```

```cpp
#define _CRT_SECURE_NO_WARNINGS 1
#include"SList.h"

void SLTPrint(SLTNode* phead)
{
    SLTNode* cur = phead;
    while (cur != NULL)
    {
        printf("%d->", cur->data);
        cur = cur->next;
    }
    printf("NULL\n");
}

SLTNode* STLBuyNode(SLTDataType x)
{
    SLTNode* newnode = (SLTNode*)malloc(sizeof(SLTNode));
    if (newnode == NULL)
    {
        perror("SLTNode* newnode = (SLTNode*)malloc(sizeof(SLTNode))");
        return NULL;
    }
    newnode->data = x;
    newnode->next = NULL;
    return newnode;
}


void SLTPushBack(SLTNode** pphead, SLTDataType x)
{
    SLTNode* newnode = STLBuyNode(x);
    if (newnode == NULL)
    {
        perror("SLTNode* newnode = (SLTNode*)malloc(sizeof(SLTNode))");
        return;
    }


    if (*pphead == NULL)
    {
        *pphead = newnode;
        return;
    }


    SLTNode* tail = *pphead;
    while (tail->next != NULL)
    {
        tail = tail->next;
    }
    tail->next = newnode;
}

void SLTPushFront(SLTNode** pphead, SLTDataType x)
{
    SLTNode* newnode = STLBuyNode(x);
    if (newnode == NULL)
    {
        perror("SLTNode* newnode = (SLTNode*)malloc(sizeof(SLTNode))");
        return;
    }

    if (*pphead == NULL)
    {
        *pphead = newnode;
        return;
    }

    newnode->next = *pphead;
    *pphead = newnode;
}

void SLTPopBack(SLTNode** pphead)
{

    if (pphead == NULL || *pphead == NULL)
    {
        return;
    }
    SLTNode* tail = *pphead;
    if (tail->next == NULL)
    {
        free(tail);
        *pphead = NULL;
        return;
    }
    while (tail->next->next != NULL)
    {
        tail = tail->next;
    }

    free(tail->next);
    tail->next = NULL;

}
void SLTPopFront(SLTNode** pphead)
{
    if (*pphead == NULL)
    {
        return;
    }
    SLTNode* tmp = *pphead;
    if (tmp->next == NULL)
    {
        free(tmp);
        *pphead = NULL;
        return;
    }
    *pphead = tmp->next;
    free(tmp);
}

SLTNode* SLTFind(SLTNode* phead, SLTDataType x)
{
    SLTNode* tmp = phead;
    while (tmp != NULL)
    {
        if (tmp->data == x)
        {
            return tmp;
        }
        tmp = tmp->next;
    } 
    return NULL;
}

void SLTInsert(SLTNode** pphead, SLTNode* pos, SLTDataType x)
{
    if (*pphead == NULL && pos == NULL)
    {
        perror("what are you doing man");
        return;
    }

    if (pos == *pphead)
    {
        SLTPushFront(pphead, x);
        return;
    }

    SLTNode* cur = *pphead;
    while (cur->next != pos)
    {
        if (cur->next == NULL)
        {
            return;
        }
        cur = cur->next;
    }
    SLTPushFront(&pos, x);
    cur->next = pos;
    return;
}


void SListInsertAfter(SLTNode* pos, SLTDataType x)
{
    if (pos == NULL)
    {
        perror("what are you doing man");
        return;
    }
    if (pos->next == NULL)
    {
        SLTPushBack(&pos, x);
    }
    else
    {
        SLTInsert(&pos, pos->next, x);
    }

}


void SListErase(SLTNode** pphead, SLTNode* pos)
{
    if (pphead == NULL || *pphead == NULL || pos == NULL)
    {
        printf("hhhhhhhh");
        return;
    }
    if (*pphead == pos)
    {
        SLTPopFront(pphead);
    }
    else
    {
        SLTNode* cur = *pphead;
        while (cur->next != pos)
        {
            if (cur->next == NULL)
            {
                return;
            }
            cur = cur->next;
        }
        cur->next = pos->next;
        free(pos);
        pos = NULL;
    }
}

void SListEraseAfter(SLTNode* pos)
{
    if (pos == NULL || pos->next == NULL)
    {
        printf("nnnnnn");
        return;
    }
    if (pos->next->next == NULL)
    {
        free(pos->next);
        pos->next = NULL;
    }
    else
    {
        SLTNode* tmp = pos->next;
        pos->next = pos->next->next;
        free(tmp);
        tmp = NULL;
    }
}


void SLTDistroy(SLTNode** pphead)
{
    if (pphead == NULL || *pphead == NULL)
    {
        printf("what can i say");
        return;
    }
    SLTNode* cur = *pphead;
    SLTNode* next = *pphead;
    while (next != NULL)
    {
        next = cur->next;
        free(cur);
        cur = next;
    }
    *pphead = NULL;
}


//void Reverse(SLTNode** pphead)
//{
//    if (pphead == NULL || *pphead == NULL || (*pphead)->next == NULL)
//    {
//        printf("what can i say");
//        return;
//    }
//    SLTNode* cur = *pphead;
//    SLTNode* sentry = *pphead;
//    while (cur->next != NULL)
//    {
//        cur = cur->next;
//    }
//    sentry = cur;
//    SLTNode* bef = *phead;
//    while (bef != sentry)
//    {
//        cur->next = bef;
//        cur = cur->next;
//        bef = bef->next;
//        cur->next = NULL;
//    }
//    *phead = sentry;
//}


void Reverse(SLTNode** pphead)
{
    if (pphead == NULL || *pphead == NULL || (*pphead)->next == NULL)
    {
        printf("what can i say");
        return;
    }
    SLTNode* cur = *pphead;
    SLTNode* sentry = (*pphead)->next;
    SLTNode* next = *pphead;
    while (sentry != NULL)
    {
        next = sentry->next;
        sentry->next = cur;
        cur = sentry;
        sentry = next;
    }
    (*pphead)->next = NULL;
    (*pphead) = cur;
}

//SLTNode* Sort2(SLTNode* phead1, SLTNode* phead2)
//{
//    if (phead1 == NULL && phead2 == NULL)
//    {
//        return NULL;
//    }
//    SLTNode* cur1 = phead1;
//    SLTNode* cur2 = phead2;
//    SLTNode* head = NULL;
//    SLTNode* tail = NULL;
//    while (cur1 != NULL && cur2 != NULL)//继续的条件
//    {
//        if (cur1->data < cur2->data)
//        {
//            if (head == NULL)
//            {
//                head = tail = cur1;
//                //cur1 = cur1->next;
//            }
//            else
//            {
//                tail->next = cur1;
//                tail = tail->next;
//                //cur1 = cur1->next;
//            }
//            cur1 = cur1->next;
//        }
//        else
//        {
//            if (head == NULL)
//            {
//                head = tail = cur2;
//                //cur2 = cur2->next;
//            }
//            else
//            {
//                tail->next = cur2;
//                tail = tail->next;
//                //cur2 = cur2->next;
//            }
//            cur2 = cur2->next;
//        }
//    }
//    if (cur1 == NULL)
//    {
//        if (tail == NULL)
//        {
//            head = cur2;
//        }
//        else
//        {
//            tail->next = cur2;
//        }
//        /*tail->next = cur2;*/
//    }
//    else
//    {
//        /*tail->next = cur1;*/
//        if (tail == NULL)
//        {
//            head = cur1;
//        }
//        else
//        {
//            tail->next = cur1;
//        }
//    }
//    return head;
//}



SLTNode* Sort2(SLTNode* phead1, SLTNode* phead2)
{
    if (phead1 == NULL && phead2 == NULL)
    {
        return NULL;
    }
    SLTNode* cur1 = phead1;
    SLTNode* cur2 = phead2;
    SLTNode* guard = NULL;
    SLTNode* tail = NULL;
    guard = tail = (SLTNode*)malloc(sizeof(SLTNode));
    tail->next = NULL;
    while (cur1 != NULL && cur2 != NULL)//继续的条件
    {
        if (cur1->data < cur2->data)
        {
            tail->next = cur1;
            tail = tail->next;
            cur1 = cur1->next;
        }
        else
        {
            tail->next = cur2;
            tail = tail->next;
            cur2 = cur2->next;
        }
    }
    if (cur1 == NULL)
    {
        tail->next = cur2;
    }
    else
    {
        tail->next = cur1;
    }
    SLTNode* head = guard->next;
    free(guard);
    return head;
}


SLTNode* Sort2x(SLTNode* phead, int x)
{
    if (phead == NULL)
    {
        return NULL;
    }
    SLTNode* castle = phead;

    while ((--x) > 0)
    {
        castle = castle->next;
        if (castle == NULL)
        {
            return phead;
        }
    }

    SLTNode* guardian1 = (SLTNode*)malloc(sizeof(SLTNode));
    SLTNode* guardian2 = (SLTNode*)malloc(sizeof(SLTNode));
    SLTNode* tail1 = guardian1;
    SLTNode* tail2 = guardian2;
    tail1->next = NULL;
    tail2->next = NULL;

    SLTNode* cur = phead;
    while (cur)
    {
        if (cur->data <= castle->data)
        {
            tail1->next = cur;
            tail1 = tail1->next;
        }
        else
        {
            tail2->next = cur;
            tail2 = tail2->next;
        }
        cur = cur->next;
    }
    tail1->next = guardian2->next;
    tail2->next = NULL;
    phead = guardian1->next;
    free(guardian1);
    free(guardian2);
    return phead;

}

bool Noon(SLTNode* phead)
{
    if (phead == NULL || phead->next == NULL)
    {
        return true;
    }
    SLTNode* cur1 = phead;
    SLTNode* cur2 = phead;
    while (cur2 && cur2->next)
    {
        cur1 = cur1->next;
        cur2 = cur2->next->next;
    }
    SLTNode* guardian1 = (SLTNode*)malloc(sizeof(SLTNode));
    guardian1->next = NULL;
    while (cur1)
    {
        SLTNode* tmp = guardian1->next;
        guardian1->next = cur1;
        cur1 = cur1->next;
        guardian1->next->next = tmp;
    }
    SLTNode* cur11 = phead;
    SLTNode* cur22 = guardian1->next;
    while (cur22)
    {
        if (cur11->data != cur22->data)
        {
            return false;
        }
        cur11 = cur11->next;
        cur22 = cur22->next;
    }
    return true;
}

bool Intersect(SLTNode* phead1, SLTNode* phead2, SLTNode** pphead)//单纯判断是否相交而不用找到焦点直接判断尾节点是否相等
{
    if (phead1 == NULL || phead2 == NULL)
    {
        return false;
    }
    SLTNode* cur1 = phead1;
    SLTNode* cur2 = phead2;
    int sum1 = 0;
    int sum2 = 0;
    while (cur1)
    {
        sum1++;
        cur1 = cur1->next;
    }
    while (cur2)
    {
        sum2++;
        cur2 = cur2->next;
    }
    cur1 = phead1;
    cur2 = phead2;
    if (sum1 >= sum2)
    {
        int difference = sum1 - sum2;
        while (difference)
        {
            cur1 = cur1->next;
            difference--;
        }
        while (cur1)
        {
            if (cur1 == cur2)
            {
                *pphead = cur1;
                return true;
            }
            cur1 = cur1->next;
            cur2 = cur2->next;
        }
    }
    else
    {
        int difference = sum2 - sum1;
        while (difference)
        {
            cur2 = cur2->next;
            difference--;
        }
        while (cur1)
        {
            if (cur1 == cur2)
            {
                *pphead = cur1;
                return true;
            }
            cur1 = cur1->next;
            cur2 = cur2->next;
        }
    }
    return false;
}


bool Circle(SLTNode* phead)
{
    if (phead == NULL)
    {
        return false;
    }
    SLTNode* slow = phead;
    SLTNode* fast = phead;
    while (fast != NULL && fast->next != NULL)
    {
        fast = fast->next->next;
        //if (fast->next == NULL)
        //{
        //    break;
        //}
        //fast = fast->next;
        slow = slow->next;
        if (slow == fast)
        {
            return true;
        }
    }
    return false;
}


int CircleLength(SLTNode* phead)
{
    if (phead == NULL)
    {
        return false;
    }
    SLTNode* slow = phead;
    SLTNode* fast = phead;
    bool flag = false;
    while (fast != NULL && fast->next != NULL)
    {
        fast = fast->next->next;
        //if (fast->next == NULL)
        //{
        //    break;
        //}
        //fast = fast->next;
        slow = slow->next;
        if (slow == fast)
        {
            flag = true;
            break;
        }
    }
    int sum = 0;
    if (flag)
    {
        fast = fast->next;
        sum++;
        while (fast != slow)
        {
            fast = fast->next;
            sum++;
        }
        return sum;
    }
    return false;
}


SLTNode* CircleFirstNode(SLTNode* phead)
{
    if (phead == NULL)
    {
        return NULL;
    }
    SLTNode* slow = phead;
    SLTNode* fast = phead;
    bool flag = false;
    while (fast != NULL && fast->next != NULL)
    {
        fast = fast->next->next;
        //if (fast->next == NULL)
        //{
        //    break;
        //}
        //fast = fast->next;
        slow = slow->next;
        if (slow == fast)
        {
            flag = true;
            break;
        }
    }
    SLTNode* First = phead;
    if (flag)
    {
        while (First != fast)
        {
            fast = fast->next;
            if (fast == slow)
            {
                First = First->next;
            }
        }
        return First;
    }
    return NULL;
}

SLTNode* CircleFirstNode2(SLTNode* phead)
{
    if (phead == NULL)
    {
        return NULL;
    }
    SLTNode* slow = phead;
    SLTNode* fast = phead;
    bool flag = false;
    while (fast != NULL && fast->next != NULL)
    {
        fast = fast->next->next;
        //if (fast->next == NULL)
        //{
        //    break;
        //}
        //fast = fast->next;
        slow = slow->next;
        if (slow == fast)
        {
            flag = true;
            break;
        }
    }
    SLTNode* First = phead;
    if (flag)
    {
        while (First != fast)
        {
            fast = fast->next;
            First = First->next;
        }
        return First;
    }
    return NULL;
}
```

```cpp
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

typedef int LTDataType;
typedef struct ListNode
{
    struct ListNode* next;
    struct ListNode* prev;
    LTDataType data;
}LTNode;


LTNode* LTInit(LTNode* phead);
bool LTEmpty(LTNode* phead);
void LTPushBack(LTNode* phead, LTDataType x);
void LTPopBack(LTNode* phead);
void LTPrint(LTNode* phead);
void LTPushFront(LTNode* phead, LTDataType x);
void LTPopFront(LTNode* phead);
void LTErase(LTNode* pos);
LTNode* LTFind(LTNode* phead, LTDataType x);
void LTDestroy(LTNode* phead);
```

```cpp
#define _CRT_SECURE_NO_WARNINGS 1
#include"List.h"

LTNode* BuyListNode(LTDataType x)
{
    LTNode* newnode = (LTNode*)malloc(sizeof(LTNode));
    newnode->next = NULL;
    newnode->prev = NULL;
    newnode->data = x;
    return newnode;
}
bool LTEmpty(LTNode* phead)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return true;
    }
    return (phead->next == phead);
}

void LTPrint(LTNode* phead)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* next = phead->next;
    printf("head->");
    while (next != phead)
    {
        printf("%d->", next->data);
        next = next->next;
    }
    printf("head\n");
}

LTNode* LTInit(LTNode* phead)
{
    phead = BuyListNode(0);
    phead->next = phead;
    phead->prev = phead;
    return phead;
}


void LTPushBack(LTNode* phead, LTDataType x)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* newnode = BuyListNode(x);
    if (newnode == NULL)
    {
        printf("error in list.c at line %d: newnode == NULL\n", __LINE__);
        return;
    }
    LTNode* tail = phead->prev;

    tail->next = newnode;
    newnode->prev = tail;
    newnode->next = phead;
    phead->prev = newnode;
}
void LTPopBack(LTNode* phead)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    if (LTEmpty(phead))
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* tail = phead->prev;
    LTNode* tailPrev = tail->prev;

    phead->prev = tailPrev;
    tailPrev->next = phead;
    free(tail);
    tail = NULL;
}

void LTPushFront(LTNode* phead, LTDataType x)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* newnode = BuyListNode(x);

    newnode->next = phead->next;
    newnode->prev = phead;
    phead->next = newnode;
    newnode->next->prev = newnode;
}
void LTPopFront(LTNode* phead)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    if (LTEmpty(phead))
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* front = phead->next;
    LTNode* frontnext = front->next;

    phead->next = frontnext;
    frontnext->prev = phead;
    free(front);
    front = NULL;

}


void LTErase(LTNode* pos)
{
    if (pos == NULL)
    {
        printf("error in list.c at line %d: pos == NULL\n", __LINE__);
        return;
    }
    if (pos->next == pos->prev)
    {
        LTNode* phead = pos->prev;
        free(pos);
        phead->next = phead;
        phead->prev = phead;
    }
    else
    {
        pos->prev->next = pos->next;
        pos->next->prev = pos->prev;
        free(pos);
    }
    return;
}

LTNode* LTFind(LTNode* phead, LTDataType x)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return NULL;
    }
    LTNode* find = phead;
    while (find->next != phead)
    {
        if (find->next->data == x)
        {
            return find->next;
        }
        else
        {
            find = find->next;
        }
    }
    printf("%d: I can't find !!!\n", __LINE__);
    return NULL;
}


void LTDestroy(LTNode* phead)
{
    if (phead == NULL)
    {
        printf("error in list.c at line %d: phead == NULL\n", __LINE__);
        return;
    }
    LTNode* cur = phead->next;
    while (cur != phead)
    {
        cur = cur->next;
        free(cur->prev);
    }
    phead->next = phead;
    phead->prev = phead;
    return;
}
```
