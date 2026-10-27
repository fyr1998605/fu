#include <chrono>
#include "UTools.hpp"
#ifndef Counter
#define Counter

using namespace std;
//range是时间范围单位是ms
template<int range>
class FrameCounter{
private:
    long Begin = 0;
    long During = 0;
    int Frame = 0;
    const int Range = range;
    long TotalFrame = 0;
    double fps = 0.0; // frame_per_second
public:
    double GetFPS(){
        return fps;
    }
    void Count(){
        if(!Frame)
            Begin = Tools::GetTimestamp();
        Frame += 1;
        if(TotalFrame == INT64_MAX)
            TotalFrame = 0;
        TotalFrame += 1;
        if(Frame >= range){
            During = Tools::GetTimestamp() - Begin;
            fps = Frame * 1.0 / ((During * 1.0) / 1000);
            Frame = 0;
        }
    }
    long COUNT(){
        return TotalFrame;
    }
    long GetFrameTime(){
        if(!fps){
            if(!Begin)
                return 20; // 没开始默认帧时间20ms
            During = Tools::GetTimestamp() - Begin;
            fps = Frame * 1.0 / ((During * 1.0) / 1000);
        }
        return (long)(1000/fps);
    }
};

template<int range>
class AverageCounter{
private:
    int Current = 0;
    double IntegralValue;
    double Value;
    double AverageValue;
public:
    double* Values;
    AverageCounter(){
        //range至少为1
        if(range <= 0)
            throw range;
        Current = 0;
        IntegralValue = 0.0;
        AverageValue = 0.0;
        Values = NULL;
        Values = new double[range]{0};
        memset(Values,0,range * sizeof(double));
    }
    ~AverageCounter(){
        if(Values){
            delete[] Values;
            Values = NULL;
        }
    }
    //清空数据
    void Clear(){
        if(Values){
            delete[] Values;
            Values = NULL;
        }
        Values = new double[range]{0};
        memset(Values,0,range * sizeof(double));
        Current = 0;
    }
    //排头去除
    double Pop(){
        if(!Values)
            throw Values;
        double result = Values[0];
        if(range == 1)
            Values[0] = 0.0;
        else{
            double* tempArr = new double[range];
            memset(tempArr,0,range * sizeof(double));
            memcpy(tempArr,Values + 1,(range - 1) * sizeof(double));
            delete[] Values;
            Values = tempArr;
            tempArr = NULL;
        }
        IntegralValue -= result;
        return result;
    }
    void Updata(double value){
        if(!Values)
            throw Values;
        IntegralValue += value;
        if (Current == range) {
            Pop();
            Values[range - 1] = value;
            AverageValue = IntegralValue / range;
        }
        else {
            Values[Current++] = value;
            if(Current)
                AverageValue = IntegralValue / Current;
        }
    }
    double GetAverageValue(){
        return AverageValue;
    }
    int GetRange(){
        return range;
    }
};
#endif