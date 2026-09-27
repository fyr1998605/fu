#pragma once

#include "buff_config.hpp"

// Legacy FFT/Ceres Buff motion predictor extracted from Fu.cpp without logic changes.

class buff_predicted//预测器
//基于 FFT（快速傅里叶变换），从角度时序数据里算出能量机关的旋转频率，用于运动周期预测、提前瞄准。

{
public:
// 预测器参数注入 (Fu.cpp 从 Buff.legacy_predictor 载入)。默认值 == 迁移前硬编码值。
void configure(const buff::LegacyPredictorConfig& config) { legacy_cfg_ = config; }

// Reset all fitted state when the runtime changes between small- and large-buff
// prediction. The historical samples remain owned by the caller.
// 注意: reset 会整体重建对象, 必须保留配置 (否则重置后参数回到默认值)。
void reset() {
    const buff::LegacyPredictorConfig saved_cfg = legacy_cfg_;
    *this = buff_predicted();
    legacy_cfg_ = saved_cfg;
}

//计算原始角度数据的估计频率
double findFrequency(const std::vector<double>& data, double sampleRate) {
    int N = data.size();// 获取输入数据的大小

    fftw_complex *in, *out;// 分配内存以存储FFT的输入和输出数据
    fftw_plan p;

    in = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);// 使用fftw_malloc分配内存
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);

    p = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);// 创建一个FFT计划，指定数据大小、输入输出数组、FFT方向和计划优化方式

    for (int i = 0; i < N; ++i) {    // 填充输入数组，将实部数据赋值给输入数组，虚部设置为0
        in[i][0] = data[i]; // 实部
        in[i][1] = 0;       // 虚部
    }

    fftw_execute(p);// 执行FFT计划

    double maxMagnitude = 0;// 初始化最大幅度和对应的索引
    int maxIndex = 0;

    for (int i = 1; i < N/2; ++i) {// 遍历FFT的输出数据，找到最大幅度的频率分量（避免直流分量）
        double magnitude = sqrt(out[i][0]*out[i][0] + out[i][1]*out[i][1]);
        if (magnitude > maxMagnitude) {
            maxMagnitude = magnitude;
            maxIndex = i;
        }
    }

    double frequency = (double)maxIndex * sampleRate / N;    // 根据最大幅度的索引和采样率计算频率

    fftw_destroy_plan(p);    // 清理资源，销毁FFT计划并释放内存
    fftw_free(in);
    fftw_free(out);

    return frequency;// 返回计算出的频率
}

//计算原始角度数据的平均值
template<typename T>
T mean(vector<T>data){
    T sum = 0;
    for(T t : data)
        sum += t;
    return sum / data.size();
}

// 定义一个模板函数，用于计算余弦模型的值
template<typename T>
static T func(T x, T a, T omega, T phi, T level){
    return a * ceres::cos(omega * x + phi) + level;
}

//定义一个模板反函数，用于计算余
//用 inverse_func 反算每个角度对应的相位
template<typename T>
T inverse_func(const T y, const T a, const T level) {
    // 确保 a 不为0，避免除以0的错误
    if (a == 0) {
        throw std::invalid_argument("Amplitude 'a' cannot be zero.");
    }
    
    // 确保 y - level 的值在 [-a, a] 范围内，因为 arccos 的范围是 [0, PI]
    T cos_value = (y - level) / a;
    if (cos_value > 1) {
        //throw std::out_of_range("The value of (y - level) / a must be within [-1, 1].");
        return 0;
    }else if (cos_value < -1){
        //throw std::out_of_range("The value of (y - level) / a must be within [-1, 1].");
        return PI;
    }
    
    // 计算反函数
    return std::acos(cos_value);//std::acos 函数返回的值在 [0,  PI] 范围内，第二个结果只能手动加负号
}

// 定义一个用于正弦拟合的CostFunction结构体
struct SineModel {
    // 6B
    //接收并存储一个观测数据点的坐标 (x, y)，其中：

    //x：自变量（通常是时间、角度等）

    //y：因变量（观测到的实际值）       
    SineModel(double x, double y) : x_(x), y_(y) {}

    // 模板函数，用于计算残差
    template <typename T> bool operator()(
        const T* const a,       // 振幅参数a的指针
        const T* const omega,   // 角频率参数omega的指针
        const T* const phi,     // 相位参数phi的指针
        const T* const level,   // 基线值level的指针
        T* residual              // 用于存储残差的数组
    ) const {
        // 计算模型预测值y_ - func<T>与实际观测值y_之间的残差
        residual[0] = (T(x_), a[0], omega[0], phi[0], level[0]);
        return true;
        //预测角度 = 振幅 * cos(ω * 时间 + 相位) + 平均角度
    }

private:
    const double x_;  // 存储x值的常量成员变量
    const double y_;  // 存储y值的常量成员变量
};

//让视觉识别出的目标运动轨迹变得平滑连续，消除噪声导致的抖动，从而提升云台瞄准的稳定性和打击精度。
//滑动窗口法平滑角度数据
template<typename T>
//对连续数据进行平滑处理，去除高频随机噪声，同时保留信号的总体趋势。
std::vector<T> moving_average(const std::vector<T>& data, size_t window_size) {
    // 检查窗口大小是否有效
    if (window_size <= 0) {
        throw std::invalid_argument("窗口大小必须是一个正整数。");
    }
    if (window_size > data.size()) {
        throw std::invalid_argument("窗口大小不能大于数据长度。");
    }
    if (data.empty()) {
        throw std::invalid_argument("数据不能为空。");
    }

    std::vector<T> averaged_data;
    //提前给向量开辟足够的内存空间，避免频繁扩容，让程序更快、更稳定。
    //分配内存
    //用push_back方法向向量末尾添加元素时，如果当前容量不足以容纳新元素，向量会自动扩容。扩容通常涉及分配新的内存空间、复制现有元素到新空间，并释放旧空间。这些操作会导致性能下降，尤其是在大量数据的情况下。
    averaged_data.reserve(data.size() - window_size + 1);

    // 使用 std::accumulate 计算第一个窗口的总和
    //T{0} = 创建一个类型为 T、值为 0 的临时对象

    //作用：作为 std::accumulate 的初始值，确保累加器的类型是 T
    T sum = std::accumulate(data.begin(), data.begin() + window_size, T{0});
    averaged_data.push_back(sum / window_size);

    // 遍历数据，更新窗口内的数据总和，并计算新的平均值
    for (size_t i = window_size; i < data.size(); ++i) {
        sum += data[i] - data[i - window_size];
        averaged_data.push_back(sum / window_size);
    }

    return averaged_data;
}

    //预测函数（delta_angle_data为差分角，要求经过了异常处理，只包含纯合理的差分角。angle_time_data为每个差分角对应的时间。MoveMode为打击目标标识符，即大小符。delta_t为差分时间。fps为帧率）
    double predicted(const std::vector<double>& delta_angle_data, const std::vector<double>& angle_time_data,std::string &MoveMode,const double &delta_t,const double &fps)
    {
            //判断数据是否正常，不正常则返回零
            if (delta_angle_data.size() < 2 || angle_time_data.size() < 2 ||
                MoveMode.empty() || delta_t <= 0 || fps <= 0){
                return 0;
            }
        // Mode selection is handled by the runtime MoveModeDetector. The old
        // exact-size (delta_angle_data.size() == fps*PI) trigger was removed:
        // it was frame-rate dependent and could overwrite a confirmed mode.
        //判断是大符还是小符
        if (MoveMode == "big")//大符预测
        {
            //初始化要带入方程的参数
            if (init_a_omega_level){//init_a_omega_level等于true即未初始化，反之为已初始化
            
                //检查数据是否足够判断大符的顺逆 （角速度为负（通常定义逆时针为正））,判断顺逆用于初始化level
                if (delta_angle_data.size() <
                    static_cast<std::size_t>(legacy_cfg_.big_min_init_samples)){  // 原 < 6
                    return 0;
                }

                //初始化定值函数的参数（函数取值范围的均值）
                const_level = legacy_cfg_.big_const_level * delta_t;//平移 (原 1.1775)
                const_omega = legacy_cfg_.big_const_omega;//角速度 (原 1.924)
                const_a = (2 * (legacy_cfg_.big_angular_speed - const_level/delta_t) *
                           sin(const_omega * delta_t / 2)) / const_omega;//振幅 (原 2.090)

                //求平均数，只取5个值的平均
                double average_ = 0;
                double sum_ = 0;
                for (int i = 1; i <= delta_angle_data.size(); i++){
                    sum_ = sum_ + (delta_angle_data[delta_angle_data.size() - i]) * 1000;
                }
                average_ = sum_ / delta_angle_data.size();
 
                //初始化要带入方程的参数为定植参数
                a = const_a;
                omega = const_omega;
                //判断大符旋转方向（平均值大于零为逆时针，反之则反）
                if (average_ > 0){
                    level = const_level;//顺时针
                    big_rotate_state = true;//true顺，反则反
                }else{
                    level = -const_level;//逆时针
                    big_rotate_state = false;//false逆，反则反
                }

                //初始化用于优化的数据数量
                ceres_data_count = static_cast<int>(fps*PI) * legacy_cfg_.ceres_trigger_factor;  // 原 *2
                
                //初始标识符
                init_a_omega_level = false;
            }


            //如果符合条件则优化
            //这段代码是Ceres 优化器的触发条件，
            //作用是当收集到的历史数据达到预设数量时，启动一次非线性优化，精确拟合正弦运动模型的参数。
            if (!seres_stop && delta_angle_data.size()  ==  ceres_data_count)
            {
                std::chrono::steady_clock::time_point ceres_start = std::chrono::steady_clock::now();//优化首时间

                //过滤异常值
                vector<double> delta_angle_filter;
                int remove_date_mub = 0;
                for (int i = 1; i < delta_angle_data.size(); i++)
                {
                    //除去突变差分角(角度相邻间隔不超过2.5度)
                    if (abs(delta_angle_data[i] - delta_angle_data[i-1]) < PI * legacy_cfg_.big_jump_ratio){  // 原 PI*0.014
                        delta_angle_filter.push_back(delta_angle_data[i]);
                    }else{
                        remove_date_mub++;
                    }
                }

                //平滑数据
                moving_average_angle_data.clear();
                if (delta_angle_filter.size() >=
                    static_cast<std::size_t>(legacy_cfg_.big_moving_average_window)) {
                    moving_average_angle_data = moving_average(
                        delta_angle_filter, legacy_cfg_.big_moving_average_window);
                } else if (!delta_angle_filter.empty()) { moving_average_angle_data = delta_angle_filter; }//滑动窗口

                //估计参数
                //计算振幅、基线、周期等运动参数

                int maxIndex = 0;
                int minIndex = 0;

                for (size_t i = 1; i < moving_average_angle_data.size(); ++i) {
                    if (moving_average_angle_data[i] > moving_average_angle_data[maxIndex]) {
                        maxIndex = i;
                    }
                    if (moving_average_angle_data[i] < moving_average_angle_data[minIndex]) {
                        minIndex = i;
                    }
                }
                // 获得两极值之间相差的时间
                double extreme_time_interval = std::abs(angle_time_data[maxIndex] - angle_time_data[minIndex])*0.001 + (remove_date_mub)/(2*fps);
                cout << "最大值下标:" << maxIndex << endl;
                cout << "最小值下标:" << minIndex << endl;
                cout << "获得两极值之间相差的时间:" << extreme_time_interval << endl;


                double sampleRate = delta_angle_filter.size()/(PI*1.5);//采样率（用于估计频率）
                double estimatedFrequency = findFrequency(moving_average_angle_data, sampleRate);//估计数据频率
                a =(*max_element(moving_average_angle_data.begin(), moving_average_angle_data.end()) - *min_element(moving_average_angle_data.begin(), moving_average_angle_data.end()))/2;//估计数据振幅
                a_temp = a;//保留原始数据的振幅
                // omega = estimatedFrequency*2*PI;//估计数据角速度
                omega = PI/extreme_time_interval;
                double omega_temp = omega;//保留原始数据的角速度
                double phi = legacy_cfg_.big_initial_phase;//估计初相位 (原 0.75)
                level = (*max_element(moving_average_angle_data.begin(), moving_average_angle_data.end()) + *min_element(moving_average_angle_data.begin(), moving_average_angle_data.end()))/2;;//估计平移
                double level_temp = level;//保留原始数据的基线

                //转换时间单位为秒
                std::vector<double> angle_time_data_;
                for (int i = 0; i < moving_average_angle_data.size(); i++){
                    angle_time_data_.push_back(moving_average_angle_data[i]/1000);
                }

                /*初始化优化器*/
                //整体作用：为正弦曲线拟合，创建 Ceres 自动求导的代价函数，用于求解最优模型参数。
                ceres::Problem problem;//设置问题
                for (int i = 0; i < moving_average_angle_data.size(); ++i) {
                    //<SineModel, 残差数=1, 参数1维度=1, 参数2维度=1, 参数3维度=1, 参数4维度=1>
                    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<SineModel, 1, 1, 1, 1, 1>(//用来算「观测角度 - 模型预测角度」的误差。
                            new SineModel(angle_time_data_[i], moving_average_angle_data[i])
                    );
                    problem.AddResidualBlock(cost_function, nullptr, &a, &omega, &phi, &level);
                }
                // 设置优化选项(更改可提升速度)
                ceres::Solver::Options options;
                options.preconditioner_type = ceres::JACOBI;
                options.max_num_iterations = legacy_cfg_.ceres_max_num_iterations;      // 原 500
                options.function_tolerance = legacy_cfg_.ceres_function_tolerance;      // 原 1e-15
                options.gradient_tolerance = legacy_cfg_.ceres_gradient_tolerance;      // 原 1e-15
                options.parameter_tolerance = legacy_cfg_.ceres_parameter_tolerance;    // 原 1e-15
                options.minimizer_type = ceres::TRUST_REGION;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.linear_solver_type = ceres::DENSE_QR;
                options.num_threads = legacy_cfg_.ceres_num_threads;  // 原 2
                //options.minimizer_progress_to_stdout = true;//日志记录输出
                /*  优化参数说明
                    options.preconditioner_type = ceres::JACOBI; 默认值ceres::JACOBI，预处理器。
                    function_tolerance：默认值是1e-6，表示目标函数值的变化小于这个值时优化停止。
                    gradient_tolerance：默认值是1e-10，表示目标函数的梯度（即雅可比矩阵的最大值）小于这个值时优化停止。
                    parameter_tolerance：默认值是1e-8，表示参数的变化量小于这个值时优化停止。
                    max_num_iterations：默认值是50，表示最大迭代次数。
                    initial_trust_region_radius：默认值是1e4，表示初始信任域的大小。
                    max_trust_region_radius：默认值是1e16，表示信任域的最大大小。
                    min_trust_region_radius：默认值是1e-32，表示信任域的最小大小。
                    min_relative_decrease：默认值是1e-3，表示每次迭代中目标函数值的相对减少量。
                    max_num_consecutive_invalid_steps：默认值是5，表示连续无效步长的最大次数。
                    num_threads：默认值是1，表示使用的线程数。
                */

                // 运行优化
                ceres::Solver::Summary summary;
                ceres::Solve(options, &problem, &summary);

                // 打印优化结果
                std::cout << "完整优化报告———————" << summary.FullReport() << std::endl;
                std::cout << "优化结果——————\n";
                //std::cout << "频率估计： " << estimatedFrequency << "\n";
                std::cout << "a, omega, phi, level: " << a << ", " << omega << ", "  << phi << ", " << level << "\n";

                // //将结果图像化
                // vector<double>y;
                // for(int i = 0; i < data2.size(); i++){
                //     y.emplace_back(func<double>(i, a, omega, phi, level));
                // }
                // plotScatter(_time, y, "Fitted data", "time", "y");

                // plt::show();

                //判断优化得出的参数是否合理可使用(优先采用平移)
                max_level = legacy_cfg_.big_fit_max_level * delta_t;//最大平移 (原 1.310)
                max_omega = legacy_cfg_.big_fit_max_omega;//最大角速度 (原 2.000)
                max_a = (2 * (legacy_cfg_.big_angular_speed - legacy_cfg_.big_fit_min_level) *
                         sin(max_omega * delta_t / 2)) / max_omega;//最大振幅
                min_level = legacy_cfg_.big_fit_min_level * delta_t;//最小平移 (原 1.045)
                min_omega = legacy_cfg_.big_fit_min_omega;//最小角速度 (原 1.884)
                min_a = (2 * (legacy_cfg_.big_angular_speed - legacy_cfg_.big_fit_max_level) *
                         sin(min_omega * delta_t / 2)) / min_omega;//最小振幅

                //打印范围
                cout << "max_level:" << max_level << endl;
                cout << "max_omega:" << max_omega << endl;
                cout << "max_a:" << max_a << endl;
                cout << "min_level:" << min_level << endl;
                cout << "min_omega:" << min_omega << endl;
                cout << "min_a:" << min_a << endl;

                //打印定值参数
                cout << "--------------------" << endl;
                cout << "const_level:" << const_level << endl;
                cout << "const_omega:" << const_omega << endl;
                cout << "const_a:" << const_a << endl;

                //判断拟合数据是否合理
                //最先判断相位omega(omega将其视为正值，因为它不影响角度计算)
                if (omega > 0){
                    //不处理omega
                }else{
                    omega = -omega;
                }
                if (omega < min_omega ||  omega > max_omega){//判断相位
                    omega = const_omega;//不符合则使用固定参数
                }        

                //a，将其视为正值，因为它不影响角度计算 
                if (a > 0){
                    //不处理a
                }else{
                    a = -a;
                }
    
                //优先选择平移，其次振幅（level正表示大符逆时针旋转，反之则反）
                if (level > min_level &&  level < max_level)//判断平移,正值逆时针
                {
                    a = (2 * (legacy_cfg_.big_angular_speed - level/delta_t) * sin(omega * delta_t / 2)) / omega;
                    cout << "正基线合理"  << endl;

                }else if (level < -min_level &&  level > -max_level)//判断平移,负值顺时针
                {
                    a = (2 * (legacy_cfg_.big_angular_speed + level/delta_t) * sin(omega * delta_t / 2)) / omega;
                    cout << "负基线合理"  << endl;
                }else if ((a > min_a &&  a < max_a) && big_rotate_state)//判断振幅
                {
                    level = (legacy_cfg_.big_angular_speed - (a*omega/(2*sin(omega * delta_t / 2)))) * delta_t;//level要根据拟合结果赋值
                    cout << "振幅合理，基线不合理" << endl;
                }else if ((a > min_a &&  a < max_a) && !big_rotate_state)//判断振幅
                {
                    level = -(legacy_cfg_.big_angular_speed - (a*omega/(2*sin(omega * delta_t / 2)))) * delta_t;//level要根据拟合结果赋值
                    cout << "振幅合理，基线不合理" << endl;
                }else{//两个都不符合则使用固定参数
                    a = const_a;
                    if (big_rotate_state)//level要根据拟合结果赋值
                    {
                        level = const_level;
                    }else{
                        level = -const_level;
                    }
                    cout << "基线和振幅都不合理" << endl;
                }
///////////////////////////////////////////////////////
	                // 直接使用 Ceres 优化后的 a/omega/level，不做原始数据覆盖
	                // 优化结果已在前面经过范围校验，若不合理已回退到 const 值
	                // （修复前: a=a_temp; omega=2; level=level_temp 覆盖了优化结果）
///////////////////////////////////////////////////
                cout << "估计的振幅：" << a << endl;
                cout << "估计的角速度：" << omega << endl;
                cout << "估计的基线：" << level << endl;
///////////////////////////////////////////////////
                //计算优化所用时间
                std::chrono::steady_clock::time_point ceres_end = std::chrono::steady_clock::now();//优化末时间
                std::chrono::steady_clock::duration ceres_time = ceres_end - ceres_start;//计算时间差
                ceres_time_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(ceres_time).count();//转换时间单位
                //输出优化时间
                std::cout << "优化时间: " << ceres_time_milliseconds << " 毫秒" << std::endl;

            }else{
                //如果不进行优化则清除优化时间计数（现在不使用）
                ceres_time_milliseconds = 0;
            }

            //异常处理
            //逆时针旋转差分角无负值，顺时针则反
            if (big_rotate_state){//顺时针
                if (delta_angle_data.back() < 0){
                    cout << "异常：顺时针角度出现负的差分角" << endl;
                    return 0;
                }
                
            }else{//逆时针
                if (delta_angle_data.back() > 0){
                    cout << "异常：逆时针角度出现正的差分角" << endl;
                    return 0;
                } 
            }
            //除去突变差分角(差分角相邻间隔不超过2.5度)
            if (abs(delta_angle_data.back() - delta_angle_data[delta_angle_data.size()-2]) >
                PI * legacy_cfg_.big_jump_ratio){  // 原 PI*0.014
                return 0;
            }else{
                filter_angle_data.push_back(delta_angle_data.back());
            }

            //初始化函数是上升期还是下降期（不是升就是降）
            if (count_ == 0){
                //判断函数是上升期还是下降期（不是升就是降）
                if (filter_angle_data.size() <
                    static_cast<std::size_t>(legacy_cfg_.big_min_judge_samples)){  // 原 < 21
                    bigbuff_state = true;//逆时针上升为1，反之为零。顺时针则反。
                }else{
                    moving_average_angle_data.clear();
                    moving_average_angle_data = moving_average(
                        filter_angle_data, legacy_cfg_.big_moving_average_window);//滑动窗口

                    bool flag_1 = (moving_average_angle_data[moving_average_angle_data.size() - 1] - moving_average_angle_data[moving_average_angle_data.size() - legacy_cfg_.big_vote_lookback_near]) > 0;
                    bool flag_2 = (moving_average_angle_data[moving_average_angle_data.size() - 1] - moving_average_angle_data[moving_average_angle_data.size() - legacy_cfg_.big_vote_lookback_mid]) > 0;
                    bool flag_3 = (moving_average_angle_data[moving_average_angle_data.size() - 1] - moving_average_angle_data[moving_average_angle_data.size() - legacy_cfg_.big_vote_lookback_far]) > 0;
                    if ((flag_1 && flag_2) || (flag_1 && flag_3) || (flag_2 && flag_3))//三人表决，通过则为上升
                    {
                        bigbuff_state = true;//逆时针上升为1，反之为零。顺时针则反。
                        cout << "上升" << endl;
                    }else{
                        bigbuff_state = false;//逆时针上升为1，反之为零。顺时针则反。
                        cout << "下降" << endl;
                    }

                    count_ = static_cast<int>(fps * legacy_cfg_.big_extreme_wait_seconds);  // 原 fps*0.2

                }  
            }else if (count_ == static_cast<int>(fps * legacy_cfg_.big_extreme_wait_seconds))//过极值点后
            {
                const int smooth_offset =
                    static_cast<int>(fps * legacy_cfg_.big_smooth_lookback_seconds);  // 原 fps*0.10
                double specific_smoothed_date =(filter_angle_data[filter_angle_data.size() - smooth_offset - 0] + 
                                                filter_angle_data[filter_angle_data.size() - smooth_offset - 1] + 
                                                filter_angle_data[filter_angle_data.size() - smooth_offset - 2] + 
                                                filter_angle_data[filter_angle_data.size() - smooth_offset - 3] + 
                                                filter_angle_data[filter_angle_data.size() - smooth_offset - 4]) /
                                               legacy_cfg_.big_smooth_window;  // 原 /5
                if (filter_angle_data.back() - specific_smoothed_date < 0){
                    // if (a < abs(filter_angle_data.back() - level) && abs(filter_angle_data.back() - level) < PI*0.28)//a不大于50度
                    // {
                    //     over_a++;
                    // }else{
                    //     over_a = 0;
                    // }
                    // if(over_a == 4){
                    //     // a = abs(filter_angle_data.back() - level);
                    //     over_a = 0;
                    // }

                    if (bigbuff_state){lag_count++;}else{lag_count = 0;}
                    if (bigbuff_state && (lag_count >= static_cast<int>(fps * legacy_cfg_.big_extreme_wait_seconds)))
                    {
                        //a = abs(filter_angle_data[filter_angle_data.size() - int(fps * 0.07) - 2] - level);
                        count_ = 1;
                        bigbuff_state = false;//逆时针上升为true，反之为零。顺时针则反。
                    }
                    cout << "下降" << endl;
                }else if(filter_angle_data.back() - specific_smoothed_date > 0){
                    // if (a < abs(filter_angle_data.back() - level) && abs(filter_angle_data.back() - level) < PI*0.28)//a不大于50度
                    // {
                    //     over_a++;
                    // }else{
                    //     over_a = 0;
                    // }
                    // if(over_a == 4){
                    //     // a = abs(filter_angle_data.back() - level);
                    //     over_a = 0;
                    // }

                    if (!bigbuff_state){lag_count++;}else{lag_count = 0;}
                    if (!bigbuff_state && (lag_count >= static_cast<int>(fps * legacy_cfg_.big_extreme_wait_seconds)))
                    {
                        //a = abs(filter_angle_data[filter_angle_data.size() - int(fps * 0.07) - 2] - level);
                        count_ = 1;
                        bigbuff_state = true;//逆时针上升为true，反之为零。顺时针则反。
                    }
                    cout << "上升" << endl;
                }
            }else{
                count_++;
            }
        
            if (filter_angle_data.empty()) return 0;
            phase = std::abs(inverse_func(filter_angle_data.back(), a, level));//计算相位

            //获得此刻相位（由于预测角的大小比较是比较绝对值，这是与顺逆有关的，所以根据level判断正负并处理相位）
            if (bigbuff_state) {
                phase = -phase; // 将数据带入反函数获得整个相位
            } else {
                phase = phase; // 将数据带入反函数获得整个相位，并取正
            }
            //cout << "此刻相位" << phase << endl;

            if ((bigbuff_state && !predicted_rectify) ||(!bigbuff_state && predicted_rectify))
            {
            /*计算出最终角度*/
            //cout << "此刻相位" << phase << endl;
                predicted_phase = phase+delta_t*omega;//最终相位
                predicted_angle = func(double(0), a, double(0),predicted_phase , level);//最终角度
                // predicted_angle = 0.8727;//最终角度
                predicted_rectify = bigbuff_state;
                rectify_phase = predicted_phase;
                rectify_time_index = angle_time_data.size()-1;
            }else {
                //计算出最终角度
                //rectify_phase = std::abs(rectify_phase);
                if (!angle_time_data.empty() && rectify_time_index < (int)angle_time_data.size())
                    rectify_phase = rectify_phase+(angle_time_data.back()-angle_time_data[rectify_time_index])*0.001*omega;//最终相位
                predicted_angle = func(double(0), a, double(0),rectify_phase , level);//最终角度
                rectify_time_index = angle_time_data.size()-1;//获取当前做预测的下标
                // cout << "omega:" << omega << endl;
            }

            // //打印预测角度和重要参数
            // cout << "预测角度" << predicted_angle << endl;
            // //cout << "预测相位" << predicted_phase << endl;
            // cout << "a=" << a << endl;
            // cout << "level=" << level << endl;
            // cout << "omega=" << omega << endl;
            // cout << "原始相位" << phase << endl;
            // cout << "预测相位" << predicted_phase << endl;
            // cout << "基础角度" << filter_angle_data.back() << endl;
            // cout << "预测角度" << predicted_angle << endl;

            // //储存预测角
            // predicted_angle_data.push_back(predicted_angle);

            //返回预测角
            // return (predicted_angle+predicted_angle_data[predicted_angle_data.size()-2])*0.5;
            return predicted_angle;         
        }

        if (MoveMode == "small")//小符
        {
            /*小符预测*/
            if (init_a_omega_level)
            {
                cout << "小符初始化" << endl;
                double average = 0;//角度数据的平均数

                //求平均数，只取最近有效帧的平均 (>=10而非==10, 避免丢帧导致永远无法初始化)
                if (delta_angle_data.size() >=
                    static_cast<std::size_t>(legacy_cfg_.small_min_init_samples)){  // 原 >= 10
                    //过滤异常值
                    vector<double> delta_angle_filter_;
                    for (int i = 1; i < delta_angle_data.size(); i++)
                    {
                        //除去突变差分角(角度相邻间隔不超过5度)
                        if (abs(delta_angle_data[i] - delta_angle_data[i-1]) <
                            PI * legacy_cfg_.small_jump_ratio){  // 原 PI*0.028
                            delta_angle_filter_.push_back(delta_angle_data[i]);
                        }
                    }
                    average = mean(delta_angle_filter_);
                    delta_angle_filter_.clear();
                }
            
                //判断小符旋转方向（平均值大于零为逆时针，反之则反）
                {
                    static int init_stuck_frames = 0;  // 超时计数器
                    if (average > 0){
                        smallbuff_state = true;//顺时针
                        init_a_omega_level = false;
                        init_stuck_frames = 0;
                    }else if (average < 0){
                        smallbuff_state = false;//逆时针
                        init_a_omega_level = false;
                        init_stuck_frames = 0;
                    } else if (init_a_omega_level) {
                        // 数据不足, 累计超时
                        init_stuck_frames++;
                        if (init_stuck_frames > legacy_cfg_.small_init_timeout_frames &&
                            delta_angle_data.size() >= 2) {  // 原 > 60
                            double forced_avg = 0;
                            for (size_t k = 0; k < delta_angle_data.size(); k++)
                                forced_avg += delta_angle_data[k];
                            forced_avg /= delta_angle_data.size();
                            smallbuff_state = (forced_avg > 0);
                            init_a_omega_level = false;
                            init_stuck_frames = 0;
                            cout << "小符超时强制初始化, 方向=" << (smallbuff_state?"顺时针":"逆时针") << endl;
                        }
                    }
                }
                // Initialize small buff speed from measured data instead of hardcoded 60°/s
                if (std::abs(average) > 1e-9) {
                    smallbuff_speed = std::abs(average);
                } else {
                    smallbuff_speed = legacy_cfg_.small_default_omega * delta_t; // 保底 60°/s (原 PI/3)
                }
                smallbuff_count = 0;//初始化计数器
            } else {
                smallbuff_count++;

                // 每 small_ema_update_period 帧用最近数据EMA更新速度估计
                if (smallbuff_count >= legacy_cfg_.small_ema_update_period &&
                    smallbuff_count % legacy_cfg_.small_ema_update_period == 0) {  // 原 30
                    vector<double> delta_angle_filter_;
                    int start_idx = max(0, (int)delta_angle_data.size() -
                                           legacy_cfg_.small_recent_window);//取最近 N 帧
                    for (int i = start_idx + 1; i < (int)delta_angle_data.size(); i++)
                    {
                        //除去突变差分角(角度相邻间隔不超过5度)
                        if (abs(delta_angle_data[i] - delta_angle_data[i-1]) <
                            PI * legacy_cfg_.small_jump_ratio){  // 原 PI*0.028
                            delta_angle_filter_.push_back(delta_angle_data[i]);
                        }
                    }
                    if (!delta_angle_filter_.empty()) {
                        double measured_speed = mean(delta_angle_filter_);
                        double alpha = legacy_cfg_.small_speed_ema_alpha;//EMA系数 (原 0.3)
                        smallbuff_speed = alpha * measured_speed + (1.0 - alpha) * smallbuff_speed;
                        cout << "小符速度EMA更新: " << smallbuff_speed / delta_t * 180 / PI << " 度/秒" << endl;
                    }
                }

                //EMA平滑预测输出，抑制逐帧抖动
                if (smallbuff_smooth_pred == 0) smallbuff_smooth_pred = smallbuff_speed;
                smallbuff_smooth_pred = legacy_cfg_.small_smooth_ema_new * smallbuff_speed +
                                        legacy_cfg_.small_smooth_ema_old * smallbuff_smooth_pred;  // 原 0.25/0.75

                //根据旋转方向返回预测角度
                if (smallbuff_state){
                    return smallbuff_smooth_pred;//顺时针
                }else{
                    return (-1)*smallbuff_smooth_pred;//逆时针
                }
            }

        }

        cout << "预测未进入！" << endl;
        return 0;//如果函数出现异常则返回零
        
    }

private:
    // 预测器参数 (默认值 == 迁移前硬编码值)
    buff::LegacyPredictorConfig legacy_cfg_;
    
            bool seres_stop = false;//根据负反馈结果判断是否停止优化

            int ceres_data_count = 0;//优化数据的数量

            //定值函数的参数（函数取值范围的均值）
            double const_level = 0;//平移
            double const_omega = 0;//角速度
            double const_a = 0;//振幅

            //大符的顺逆
            bool big_rotate_state = true;//true逆，反则反

            //优化后函数的参数
            double a = 0;
            double a_temp = 0;
            double omega = 0;
            double level = 0;

            //参数初始化
            bool init_a_omega_level = true;//等于true即未初始化，反之为已初始化

            //平滑数据储存器
            vector<double> moving_average_angle_data;//平滑数据储存器
            vector<double> filter_angle_data;

            //优化结果的范围
            double max_level = 0;//最大平移
            double max_omega = 0;//最大角速度
            double max_a = 0;//最大振幅
            double min_level = 0;//最小平移
            double min_omega = 0;//最小角速度
            double min_a = 0;//最小振幅

            //优化所用时间
            double ceres_time_milliseconds = 0;

            //大符此时处于函数中的状态（上升下降）
            int count_ = 0;//经过极值过后不进行判断的次数
            int lag_count = 0;//相邻重复符合极值条件的次数，经过一定次数才可算是经过极值
            int over_a = 0;//超越振幅重复次数，经过一定次数才能扩大a
            bool bigbuff_state = false;//上下标识符，上升为1，反之为零

            //大符的相位
            double phase = 0;

            //大符预测的相位
            double predicted_phase = 0;

            //矫正预测判断（使用极值作为预测矫正）及其相位及其返回预测角时的下标
            bool predicted_rectify = true;
            double rectify_phase = 0;
            double rectify_time_index = 0;

            //大符预测的角度
            double predicted_angle = 0;

            //储存大符预测的角度和其现实的时间的容器
            vector<double> predicted_angle_data;//角度

            //小符此时的状态（顺时针还是逆时针）
            bool smallbuff_state = false;//true顺false逆标识符
            double smallbuff_speed = 0;//小符速度
            int smallbuff_count = 0;//小符帧计数(用于定时EMA更新)
            double smallbuff_smooth_pred = 0;//小符预测EMA平滑值

};


