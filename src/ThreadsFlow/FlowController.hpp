#pragma once
#include <thread>
#include <unistd.h>
#include <functional>
#include <condition_variable>

class FlowThread : public std::thread
{
public:
    inline FlowThread(std::function<void()> thread, int CPUID, float clockingHZ);

    inline void FlowTryStop();

    inline void FlowStopAndWait();

    uint32_t TimeOut_MAX = 0;
    uint32_t TimeDT = 0;
    float RunClockHz = 0;

private:
    uint32_t GetTimestamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return ((tv.tv_sec * (uint32_t)1000000 + (tv.tv_nsec / 1000)));
    }
    uint32_t TimeThreadStart = 0;
    uint32_t TimeStart = 0;
    uint32_t TimeNext = 0;
    uint32_t TimeEnd = 0;
    uint32_t ThreadSpeed = 0;
    uint32_t Time__Max = 1000000; // defaultly run 1HZ
    uint32_t SleepOffset = 100;

    float targetClock = 0;
    int clockcount = 1;
    float tmpClock = 0;

    std::mutex copylock;
    bool lockFlag = false;
    bool IsThreadRunning = false;

    std::condition_variable notifyWait;
    std::function<void()> threadMain = [] {};
};

FlowThread::FlowThread(std::function<void()> thread, int CPUID, float clockingHZ)
    : std::thread([&]
                  {
    sleep(1);
    IsThreadRunning = true;

    std::unique_lock<std::mutex> lockwait{copylock};
    while (!lockFlag)
        notifyWait.wait(lockwait);

    TimeThreadStart = GetTimestamp();
    while (IsThreadRunning)
    {
        TimeStart = GetTimestamp() - TimeThreadStart;
        TimeNext = TimeStart - TimeEnd;
        //
        threadMain();
        //
        TimeEnd = GetTimestamp() - TimeThreadStart;
        ThreadSpeed = TimeEnd - TimeStart;

        if (Time__Max > 0)
        {
            if(ThreadSpeed+TimeNext+SleepOffset>Time__Max|TimeNext<0)
            {
                usleep(1);
            }
            else
            {
                usleep(Time__Max - ThreadSpeed-TimeNext-SleepOffset);
            }
            if (ThreadSpeed + TimeNext > TimeOut_MAX)
            {
                TimeOut_MAX = ThreadSpeed;

            }     

            if(clockcount>targetClock)
            {
                RunClockHz = (1.f / (tmpClock / (float)clockcount) * 1000000.f);
                clockcount = 0;
                tmpClock = 0;

            } 
                tmpClock += TimeDT;
                clockcount++;

            }
            TimeEnd = GetTimestamp() - TimeThreadStart;
            TimeDT = TimeEnd - TimeStart;
    } })
{
    threadMain = thread;
    targetClock = clockingHZ;
    Time__Max = clockingHZ > 0.0 ? ((int)((1.f / clockingHZ) * 1000000.f)) : 0;
    if (CPUID >= 0)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(CPUID, &cpuset);
        int rc = pthread_setaffinity_np(this->native_handle(), sizeof(cpu_set_t), &cpuset);
    }
    {
        std::unique_lock<std::mutex> lockWaitMain{copylock};
        lockFlag = true;
    }
    notifyWait.notify_all();
}

void FlowThread::FlowTryStop()
{
    IsThreadRunning = false;
}

void FlowThread::FlowStopAndWait()
{
    FlowTryStop();
    if (this->joinable())
        this->join();
}