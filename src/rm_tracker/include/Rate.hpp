# include <iostream>
#include <chrono>
using namespace std;

// 帧率控制器
// 未经过使用和测试
// 仅供参考
class Rate 
{
public: 
    // 委托构造: C++11引入的委托构造函数,允许同一个类的一个构造函数通过其初始化列表调用另一个构造函数
    // 多个构造函数共享相同的初始化逻辑,可将公共初始化部分放在一个主构造函数中,其他构造函数通过委托它来时间
    // 减少重复代码,提高可维护性,增强代码清晰度
    // 避免循环委托, 不能混合使用: 在委托构造函数的初始化列表中,只能调用另一个构造函数,不能再初始化其他成员变量
    // 委托构造默认使用30Hz
    Rate() : Rate(30){}

    // 主构造函数
    Rate(unsigned int r)
    {
        m_LoopRate(r);
        m_start_time_width_usec(0);
    }
    double m_start_time_width_usec;

    void sleep()
    {
        double period = 1 / m_LoopRate;
        double cur_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        double expect_end_time = m_start_time_width_usec + period;

        if (expect_end_time > cur_time)
        {
            usleep((expect_end_time - cur_time) * 1000000);
        }
        m_start_time_width_usec = now();
    }

private:
    int m_LoopRate;
}