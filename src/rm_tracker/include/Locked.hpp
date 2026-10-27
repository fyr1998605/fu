#ifndef LOCKED
#define LOCKED
#include <mutex>
#include <iostream>
// 老登遗留: 锁 目前未经过使用和测试
namespace Tools{
    template<class T>
    class Locked{
    private:
        bool locked;
        T value;
        std::mutex lock;
    public:
        Locked():locked(false),value(){}
        Locked(T v):value(v),locked(false){}
        void Change(T nValue){
            lock.lock();
            value = nValue;
            lock.unlock();
            return;
        }
        T GetWithoutLock(){
            lock.lock();
            T result(value);
            lock.unlock();
            return std::move(result);
        }
        T* Get(){
            lock.lock();
            locked = true;
            return &value;
        }
        void Release(){
            if(!locked)
                return;
            lock.unlock();
            locked = false;
            return;
        }
        void Run(std::function<void(T&)> func){
            lock.lock();
            func(value);
            lock.unlock();
        }
        operator T()const{
            return GetWithoutLock();
        }
    };
}
#endif