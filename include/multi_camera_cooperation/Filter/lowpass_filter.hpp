#ifndef LOWPASS_FILTER_HPP_
#define LOWPASS_FILTER_HPP_

#include <iostream>
#include <vector>
#include <cmath>
 
using namespace std;
 
// 低通滤波器类
namespace lowpass {
    class LowPassFilter {
    public:
        // 构造函数
        LowPassFilter(double sample_rate, double cutoff_frequency);

        // 更新滤波器输出
        double update(double input);
    
    private:
        double alpha_;
        double prev_output_;
    };
}

#endif // LOWPASS_FILTER_HPP_