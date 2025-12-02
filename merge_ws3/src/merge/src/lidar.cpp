#ifndef __LIDAR_H_
#define __LIDAR_H_

#include<ros/ros.h>
#include <atomic>
#include <iostream>
#include <mutex>


namespace Ten
{

class T
{
public:
    int test = 0;
};

template<typename T>
class Ten_Node
{
public:
    // Ten_Node(const Ten_Node& node) = delete;
    // Ten_Node& operator=(const Ten_Node& node) = delete;
    Ten_Node()
    :next_(nullptr)
    {

    }

    Ten_Node(const T& msg)
    :data_(msg),
    next_(nullptr)
    {

    }

    Ten_Node(const Ten_Node& node)
    :data_(node.data_),
    next_(node.next_)
    {

    }

    Ten_Node& operator=(const Ten_Node& node)
    {
        data_ = node.data_;
        next_ = node.next_;
        return *this;
    }

    ~Ten_Node()
    {

    }


//private:
T data_;
Ten_Node* next_;
};



template<typename T>
class Ten_lock_list
{
public:
    Ten_lock_list(Ten_lock_list& list) = delete;
    Ten_lock_list& operator=(Ten_lock_list& list) = delete;
    Ten_lock_list(size_t max_size = 3)
    :head_(nullptr),
    tail_(nullptr),
    size_(0),
    max_size_(max_size >= 3 ? max_size : 3)
    {
        Ten_Node<T>* head = new Ten_Node<T>();
        Ten_Node<T>* tail = new Ten_Node<T>();
        head->next_ = tail;
        head_.store(head);
        tail_.store(tail);
        size_ = 2;
    }
    
    T Get_data()
    {
        // if(size_ < 2)
        // {
        //     return T();
        // }
        std::lock_guard<std::mutex> lock(mtx_get_);
        if(size_.load() == 2)
        {
            return head_.load()->data_;
        }
        else
        {
            Ten_Node<T>* tmp = head_.load();
            T data = tmp->data_;
            PopFront();
            return data;
        }
    }

    void Push_data(const T& msg)
    {
        if(size_.load() > max_size_)
        {
            //PopFront();
            Get_data();
        }
        PushBack(msg);
    }

    ~Ten_lock_list()
    {
        Ten_Node<T>* cur = head_.load();
        while(cur != nullptr)
        {
            Ten_Node<T>* tmp = cur->next_;
            delete cur;
            cur = tmp;
        }
    }
private:
    void PopFront()
    {
        if(size_.load() <= 2)
        {
            return;
        }
        Ten_Node<T>* head = head_.load();
        Ten_Node<T>* next = head->next_;
        head_.store(next);
        delete head;
        //size_--;
        size_.fetch_sub(1); // 修正：原子减操作，避免计数错误
    }

    void PushBack(const T& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_push_);
        Ten_Node<T>* tmp = new Ten_Node<T>(msg);
        Ten_Node<T>* tail = tail_.load();
        tail->next_ = tmp;
        tail_.store(tmp);
        //size_++;
        size_.fetch_add(1);
    }

std::atomic<Ten_Node<T>*> head_;
std::atomic<Ten_Node<T>*> tail_;
std::mutex mtx_get_;
std::mutex mtx_push_;
std::atomic<size_t> size_;
const size_t max_size_; 
};


}

#endif


