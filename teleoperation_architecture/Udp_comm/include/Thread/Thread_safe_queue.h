//
// Created by zican on 27.06.22.
//

#ifndef SIMPLE_UDP_CORE_THREAD_SAFE_QUEUE_H
#define SIMPLE_UDP_CORE_THREAD_SAFE_QUEUE_H

#include <mutex>
#include <condition_variable>
#include <deque>
#include <memory>

template <class T>
class Thread_safe_queue{
private:
    mutable std::mutex mux;
    std::deque<T> data;
    std::condition_variable data_cond;
    int MAX_SIZE=2048;


public:
    Thread_safe_queue()=default;
    Thread_safe_queue(Thread_safe_queue<T> const & other)
    {
        std::lock_guard<std::mutex> lk(other.mux);
        data=other.data;
    };

    void expand_max_size(int size_to_be_added)
    {
        if(size_to_be_added<=0)return;
        std::lock_guard<std::mutex> lk(mux);
        MAX_SIZE+=size_to_be_added;
    }

    void push_back(T input)
    {
        std::lock_guard<std::mutex> lk(mux);
        if(data.size()>=MAX_SIZE)data.pop_front();
        data.push_back(input);
        data_cond.notify_one();
    }

    void wait_and_pop_front(T& value)
    {
        //unique_lock does not automatically lock the mutex on construction
        //and provide interface to lock and unlock the mutex
        std::unique_lock<std::mutex> lk(mux);

        //wait greatly reduce the polling times with system signals
        data_cond.wait(lk,[this]{return !data.empty();});
        value=data.front();

        //unique_lock unlock the mutex on destruction so unlock by hand is not needed

        data.pop_front();
    }

    std::shared_ptr<T> wait_and_pop_front()
    {
        std::unique_lock<std::mutex> lk(mux);

        data_cond.wait(lk,[this]{return !data.empty();});
        std::shared_ptr<T> res(std::make_shared<T>(data.front()));

        data.pop_front();

        return res;
    }

    bool try_pop_front(T& value)
    {
        std::lock_guard<std::mutex> lk(mux);
        if(data.empty())return false;
        value=data.front();
        data.pop_front();
        return true;
    }

    std::shared_ptr<T> try_pop_front()
    {
        std::lock_guard<std::mutex> lk(mux);
        if(data.empty())return false;

        std::shared_ptr<T> res(std::make_shared<T>(data.front()));
        data.pop_front();
        return res;
    }

    bool is_empty() const
    {
        std::lock_guard<std::mutex> lk(mux);
        return data.empty();
    }

};

#endif //SIMPLE_UDP_CORE_THREAD_SAFE_QUEUE_H
