#include "multi_camera_cooperation/Filter/lowpass_filter.hpp"

namespace lowpass {

    // 构造函数
    LowPassFilter::LowPassFilter(double sample_rate, double cutoff_frequency) {
        double dt = 1.0 / sample_rate;
        double RC = 1.0 / (cutoff_frequency * 2.0 * M_PI);
        alpha_ = dt / (dt + RC);
        prev_output_ = 0.0;
    }
 
    // 更新滤波器输出
    double LowPassFilter::update(double input) {
        double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
        prev_output_ = output;
        return output;
    }
}