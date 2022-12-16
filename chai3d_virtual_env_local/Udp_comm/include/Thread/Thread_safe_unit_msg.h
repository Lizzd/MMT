//
// Created by zican on 23.09.22.
//

#ifndef TELEOPERATION_ARCHITECTURE_AHTALKA_THREAD_SAFE_UNIT_MSG_H
#define TELEOPERATION_ARCHITECTURE_AHTALKA_THREAD_SAFE_UNIT_MSG_H

#include <mutex>
#include <condition_variable>
#include <memory>

template <class T>
class Thread_safe_unit_msg{
private:
    mutable std::mutex mux;
    T data;
    std::condition_variable data_cond;
    bool unused_data=false;


public:
    Thread_safe_unit_msg(){};

    Thread_safe_unit_msg(Thread_safe_unit_msg<T> const & other)
    {
        std::lock_guard<std::mutex> lk(other.mux);
        data=other.data;
    }

    // no matter whether there exists data or not, update data and set the unused flag to true
    void push_back(T input)
    {
        std::lock_guard<std::mutex> lk(mux);
        data=input;
        unused_data=true;
        data_cond.notify_one();
    }

    void wait_and_pop_front(T& value)
    {
        //unique_lock does not automatically lock the mutex on construction
        //and provide interface to lock and unlock the mutex
        std::unique_lock<std::mutex> lk(mux);

        //wait greatly reduce the polling times with system signals
        data_cond.wait(lk,[this]{return this->unused_data;});

        value=data;
        unused_data=false;

        //unique_lock unlock the mutex on destruction so unlock by hand is not needed
    }

    std::shared_ptr<T> wait_and_pop_front()
    {
        std::unique_lock<std::mutex> lk(mux);

        data_cond.wait(lk,[this]{return this->unused_data;});
        std::shared_ptr<T> res(std::make_shared<T>(data));
        unused_data=false;
        return res;
    }

    bool try_pop_front(T& value)
    {
        std::lock_guard<std::mutex> lk(mux);
        if(!unused_data)return false;
        value=data;
        unused_data=false;
        return true;
    }

    bool is_empty() const
    {
        return !unused_data;
    }

};

#endif //TELEOPERATION_ARCHITECTURE_AHTALKA_THREAD_SAFE_UNIT_MSG_H
