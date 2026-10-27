#pragma once

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>

// Persistent rotation direction that survives object re-creation.
// +1 = clockwise (polar angle increases), -1 = counterclockwise, 0 = unset.
extern int g_persistent_rotation_dir;

// Legacy Buff polar-angle processor extracted from Fu.cpp without logic changes.

//整体逻辑：坐标转极角 → 采集样本判断旋转方向 → 角度连续化修正 + 圈数统计 → 数据缓存 / 差分计算 → 点位预测（预留）
class Fu_angleprocess//角度处理器
{
public:	//注：角度是指极坐标系下当前射线与x轴正方向夹角，夹角是指当前射线与后一射线的角度差值
    //参数
    float angle_ = 0.f;
	int loop_count = 0;//当前转动第几圈
	float angle_back = 0;//前一角度
	float angle_now = 0;//后一角度
	float angle_include = 0;//夹角
	float angle_polar_back = 0;//极坐标前一角度
	double last_angle_time_ms = 0;//上次处理的时间戳(ms), 用于检测帧丢失
	float angle_polar_now = 0;//极坐标当前角度
	bool last_frame_polar_valid = false;//本帧极角是否新鲜（图幅内且已刷新），供 RANSAC 采样与旧差分解耦
	float radius_buff = 0;//能量机关的半径
	bool anticlockwise = true; //默认逆时针为正方向
	bool anticlockwise_flag = false;//方向是否判断成功
	bool loop_add_flag = false;//判断是否可以增加圈数
	bool first_angle = true;//传入第一个角度
	cv::Point2f point_pre;//预测出来的二维点
	cv::Point2f point_center_back;//保留预测前的二维点
	vector<float> trun_count;//角度数组(用于判断旋转方向)
    vector<float> angle_date;//角度数组(储存迄今为止所有角度)
    vector<double> angle_time;//角度数组(用于判断旋转方向)
    double angle_diff = 0;//角度差分
    vector<double> angle_diff_vec;//差分角度容器

    void set_image_size(const int width, const int height)
    {
        if (width > 0 && height > 0) {
            image_width_ = width;
            image_height_ = height;
        }
    }

//polar_angle(circle_center, armor_center)
//├─ 计算相对偏移 cartesian_x = 目标X - 圆心X
//├─ 计算相对偏移 cartesian_y = 目标Y - 圆心Y
//├─ 计算两点距离radius，存入全局radius_buff
//├─ 判断x、y正负，标记x_flag、y_flag
//├─ 用反余弦算出基础锐角angle
//├─ 根据所在象限(x_flag,y_flag)换算完整极角theta
//└─ 返回极角theta（弧度，X轴正方向逆时针）
    float polar_angle(cv::Point2f circle_center, cv::Point2f armor_center)
	//自定义函数 polar_angle，输入旋转圆盘中心和装甲板中心两个坐标，输出极坐标角度（射线与 X 轴正方向夹角），同时记录旋转半径。
    {
		bool x_flag = false, y_flag = false;
		float angle = 0;
		float theta = 0;
        //cartesian_x = 目标装甲板中心点 X 方向相对偏移量
		float cartesian_x = armor_center.x - circle_center.x;//笛卡尔坐标系
		float cartesian_y = armor_center.y - circle_center.y;
		float radius = sqrt(cartesian_x * cartesian_x + cartesian_y * cartesian_y);
		this->radius_buff = radius;
        if (!std::isfinite(radius) || radius <= 1e-6f) {
            return std::numeric_limits<float>::quiet_NaN();
        }
		//判断坐标在哪个象限，根据象限加角度
		cartesian_x >= 0 ? (x_flag = true) : (x_flag = false);
		cartesian_y >= 0 ? (y_flag = true) : (y_flag = false);
		//计算出当前直线与x轴的夹角
		const float ratio = std::clamp(abs(cartesian_x) / radius, 0.0f, 1.0f);
		angle = acos(ratio);
		x_flag ? (y_flag ? (theta = angle + 0) : (theta = 1.5 * PI + (0.5 * PI - angle))) : (y_flag ? (theta = 0.5 * PI + (0.5 * PI - angle)) : (theta = angle + 1 * PI));
		//cout << theta * (180 / PI) << endl;
		return std::isfinite(theta)
            ? theta : std::numeric_limits<float>::quiet_NaN();
	}

	//用于自动识别能量机关圆盘整体旋转方向（顺时针 / 逆时针），只在角度缓存积累满 21 组数据时完成一次判断，判定后永久锁定方向不再重复计算。
	//4_27角度判读正确
	void turn(float angle)//放入角度
	{
		//cout << "-----------判断方向--------------" << endl;
		// If a persistent direction was saved from a previous session,
		// pre-seed the direction to avoid wrong initialization.
		if (!this->anticlockwise_flag && g_persistent_rotation_dir != 0) {
			this->anticlockwise = (g_persistent_rotation_dir > 0);
			this->anticlockwise_flag = true;
			this->trun_count.clear();
			std::cout << "方向从持久存储中恢复: " << (this->anticlockwise ? "正转(clockwise)" : "反转(counterclockwise)") << std::endl;
			return;
		}
		if (this->trun_count.size() < 21) this->trun_count.push_back(angle);

		if (this->trun_count.size() == 21 && this->anticlockwise_flag == false)
		{
            vector<double> angle_filter;//角度滤波函数
            for (int i = 1; i < trun_count.size(); i++)
            {
                //除去突变差分角(角度相邻间隔不超过5度)
                if (abs(trun_count[i] - trun_count[i-1]) < PI*0.028){
                    angle_filter.push_back(trun_count[i]);
                }
            }
            
			int positive_count = 0;//正转次数
			int negative_count = 0;//反转次数
			for (int i = 0; i < angle_filter.size(); i++)
			{
				if (i != 0)
				{
					if (abs(angle_filter[i] - angle_filter[i - 1]) >= 300 || abs(angle_filter[i] - angle_filter[i - 1]) == 0) continue;
					else if ((angle_filter[i] - angle_filter[i - 1]) > 0)  positive_count++;
					else if ((angle_filter[i] - angle_filter[i - 1]) < 0)  negative_count++;
				}
			}
			if (positive_count > negative_count)
			{
				//正转
				this->anticlockwise = true;
				this->anticlockwise_flag = true;
					g_persistent_rotation_dir = 1;  // clockwise
				std::cout << "正转，buff顺时针转" << endl;
			}
			else if (negative_count > positive_count)
			{
				//反转
				this->anticlockwise = false;
				this->anticlockwise_flag = true;
					g_persistent_rotation_dir = -1;  // counterclockwise
				std::cout << "反转，buff逆时针转" << endl;
			}
			else
			{
				std::cout << "Not enough data to judge anticlockwise (555row)" << endl;
				this->trun_count.clear();
				return;
			}
		}
		return;
	}

	//根据正反转以及圈数完成累加
	//角度统一修正函数
    //先判断转向：顺时针 → 角度减整圈，转为负区间保证连续；
    //非首帧，计算前后帧角度差；
    //若是逆时针，按角度所在区间分三段处理：
    //0°~30°：修正角度、打开加圈标记；
    //355°~360°：修正角度、关闭加圈标记；
    //中间区间：正常加圈、角度修正 + 检测 360° 跳变并补计圈数；
    //最终输出连续无跳变的累计角度，用于后续滤波、测速、瞄准计算
	float angle2kf(float angle)
	{
		this->angle_now = angle;//后一角度（原先从未被赋值，下方 angle_now==0 条件恒真）
		this->angle_ = angle;//[FIX] 把当前测量写入累积角，使下方圈数/值域修正作用于真实测量
		if (this->angle_now == 0 && !this->anticlockwise && this->anticlockwise_flag)//反转
		{
			angle_ = angle_ - 6.283185;//顺时针旋转时做角度值域修正
		}
		else if (!this->anticlockwise && this->anticlockwise_flag) angle_ = angle_ - 6.283185;
		// cout << "angle " << angle_ << endl;
		if (!this->first_angle)
		{
			this->angle_include = abs(this->angle_polar_now - this->angle_polar_back);//极坐标下的角度差值
		}
		if (this->anticlockwise && this->anticlockwise_flag)//正转
		{
			if (angle_ >= 0 && angle_ <= 0.5235986)//0 <= x <= 30 loop_count
			{
				if (this->first_angle)//是第一个角度，且进入加圈状态
				{
					loop_count--;//配合逆时针旋转的角度跳变规则，修正累计圈数。
					angle_ = angle_ + 6.283185 * (loop_count + 1);//6.283185（一整圈 360°）
					loop_add_flag = true;
				}
				else
				{
					angle_ = angle_ + 6.283185 * (loop_count + 1);
					loop_add_flag = true;
				}
				// cout << "1" << endl;
			}
			else if (angle_ >= 6.195918 && angle_ <= 6.283185)//355 <= x <= 360
			{
				angle_ = angle_ + 6.283185 * loop_count;
				loop_add_flag = false;
				// cout << "2" << endl;
			}
			else if (angle_ > 0.5235986 && angle_ < 6.195918)//30 < x < 355
			{
				if (loop_add_flag)
				{
					loop_count++;
					loop_add_flag = false;
					// cout << "true3" << endl;
				}
				angle_ = angle_ + 6.283185 * loop_count;
				if (this->angle_include >= 1.047197 && this->angle_include <= 5.232986 && this->angle_polar_back > this->angle_polar_now)//若跳到之前的角后，则相当于再转了一圈
				{
					loop_count++;
					angle_ = angle_ + 6.283185 * loop_count;
				}
				// cout << "3" << endl;
			}
		}
		// cout << angle_<< endl;
		//  cout << "loop_count" << loop_count << endl;
		if (!this->anticlockwise && this->anticlockwise_flag)//反转
		{
			if (angle_ <= 0 && angle_ >= -0.5235986)//-30 <= x <=0
			{
				if (this->first_angle)
				{
					loop_count--;
					angle_ = angle_ - 6.283185 * (loop_count + 1);
					loop_add_flag = true;
				}
				else
				{
					angle_ = angle_ - 6.283185 * (loop_count + 1);
					loop_add_flag = true;
				}
			}
			else if (angle_ <= -6.195918 && angle_ >= -6.283185)//-360 <= x <=-355
			{
				angle_ = angle_ - 6.283185 * loop_count;
				loop_add_flag = false;
			}
			else if (angle_ < -0.5235986 && angle_ > -6.195918)//-355 < x < -30
			{
				if (loop_add_flag == true)
				{
					loop_count++;
					loop_add_flag = false;
				}
				angle_ = angle_ - 6.283185 * loop_count;
				if (this->angle_include >= 1.047197 && this->angle_include <= 5.232986 && angle_polar_back < this->angle_polar_now)//若跳到之前的角后，则相当于再转了一圈
				{
					loop_count++;
					angle_ = angle_ - 6.283185 * loop_count;
				}
			}
		}
		this->angle_polar_back = angle;
		this->first_angle = false;//运行过一次之后，证明不再第一个进去的角度
		//cout << "angle " << angle_ << endl;
		// cout << angle_ * 180 / PI << endl;
		// cout << "---------------" << endl;
		return angle_;
	}


    //角度差分函数
    //给定一段预测时长 delta_t，从历史多帧角度时序缓存里，取出距离当前刚好 delta_t 秒前的风车角度，计算角度差；
    //同时做三层校验：数据量校验、时间间隔超时校验、360° 跨圈角度跳变矫正；
    //最终输出「过去 delta_t 时间内风车转过的角度差」，用来计算平均角速度 ω = angle_diff /delta_t，是能量机关旋转预测的核心输入。
    double angle_diff_fun(const double delta_t, const double fps) {
        // Keep the legacy correction rules, but select only indices that have
        // two neighbors on each side for the five-point average.
        if (!std::isfinite(delta_t) || !std::isfinite(fps) ||
            delta_t <= 0.0 || fps <= 0.0 ||
            angle_date.size() != angle_time.size() || angle_time.size() < 5 ||
            fps * delta_t > static_cast<double>(std::numeric_limits<int>::max()) ||
            angle_time.size() < fps * delta_t) {
            return -50;
        }
        const int sample_count = static_cast<int>(angle_time.size());
        const int first_offset = std::max(
            2, static_cast<int>(fps * delta_t * 0.5));
        for (int offset = first_offset; offset <= sample_count - 3; ++offset) {
            const int target_index = sample_count - 1 - offset;
            if (target_index < 2 || target_index + 2 >= sample_count) {
                continue;
            }
            const double elapsed = angle_time.back() - angle_time[target_index];
            if (!std::isfinite(elapsed) || elapsed < 0.0 ||
                elapsed > delta_t * 1000.0 + 80.0) {
                return -50;
            }
            if (elapsed >= delta_t * 1000.0) {
                angle_diff = angle_date.back() -
                    (angle_date[target_index] + angle_date[target_index - 1] +
                     angle_date[target_index + 1] + angle_date[target_index - 2] +
                     angle_date[target_index + 2]) / 5.0;
                if (!std::isfinite(angle_diff)) {
                    return -50;
                }
                if ((std::abs(angle_diff) > 1.169 ||
                     (anticlockwise && angle_diff < 0.0) ||
                     (!anticlockwise && angle_diff > 0.0)) &&
                    anticlockwise_flag) {
                    for (int correction = -6; correction <= 6; ++correction) {
                        const double corrected = angle_diff -
                            2.0 * correction * PI / 5.0;
                        if (anticlockwise && corrected < 2.0 * PI / 6.55 &&
                            corrected > -1e-9) {
                            return corrected;
                        }
                        if (!anticlockwise && corrected > -2.0 * PI / 6.55 &&
                            corrected < 1e-9) {
                            return corrected;
                        }
                    }
                    return -50;
                }
                return angle_diff;
            }
        }
        return -50;

    }
    //传入角度和半径，得到x,y坐标
	cv::Point2f x_y_cal(const double &angle, const double &radius)
	{
		if (!std::isfinite(angle) || !std::isfinite(radius) ||
			!std::isfinite(point_center_back.x) || !std::isfinite(point_center_back.y)) {
			const float nan = std::numeric_limits<float>::quiet_NaN();
			return cv::Point2f(nan, nan);
		}
		Eigen::MatrixXf point2d_(2, 1);//二维像素坐标向量
		Eigen::MatrixXf radius_matrix(2, 2);
		Eigen::MatrixXf angel_matrix(2, 1);//角度+角速度状态向量
		radius_matrix <<
			radius, 0,
			0, radius;
		angel_matrix <<
			cos(angle),
			sin(angle);
		point2d_ = radius_matrix * angel_matrix;
		point_pre = cv::Point2f(point2d_(0, 0) + point_center_back.x, point2d_(1, 0) + point_center_back.y);
		// cout << "calc" << point2d_ << endl;
		// cout << "point:" << point_pre.x << " " << point_pre.y << endl;
		// cout << "pre_angle "  << angle << endl;
		return point_pre;
	}

    //预测前角度处理(circle_center为中心R中心，armor_center为装甲板中心，time为与该角度相对应的时间单位为毫秒,delta_t为德尔塔T，fps为帧率。)
	double pre_predict_angle_process(const cv::Point2f &circle_center, const cv::Point2f &armor_center,const double &time, const double &delta_t, const double &fps)
	{
		float angle = 0;
		if (!std::isfinite(time) || !std::isfinite(delta_t) ||
			!std::isfinite(fps) || delta_t <= 0.0 || fps <= 0.0) {
			this->last_frame_polar_valid = false;
			return -1;
		}
		this->last_frame_polar_valid = false;//默认本帧极角不可用，仅在图幅内且完成刷新后置真
		//判断数据是否丢帧,点的值是否都是在图幅内
		if (
			(circle_center.x > 0 && circle_center.x <= image_width_) &&
			(circle_center.y > 0 && circle_center.y <= image_height_) &&
			(armor_center.x > 0 && armor_center.x <= image_width_) &&
			(armor_center.y > 0 && armor_center.y <= image_height_)
			)
		{
			//保留了圆心点
			//cout << "-----------------" << endl;
			// Detect frame drops: if >100ms since last processing, reset continuity
			// to prevent stale angle_polar_back from corrupting loop counting.
			if (this->last_angle_time_ms > 0 && (time - this->last_angle_time_ms) > 100.0) {
				this->first_angle = true;
				this->angle_polar_back = 0;
			}
			this->point_center_back.x = circle_center.x;
			this->point_center_back.y = circle_center.y;
			if (!std::isfinite(circle_center.x) || !std::isfinite(circle_center.y) ||
				!std::isfinite(armor_center.x) || !std::isfinite(armor_center.y)) {
				return -1;
			}
			angle = this->polar_angle(circle_center, armor_center);//极坐标系下角度
			this->angle_polar_now = angle; // 保存当前极角，供预测+叉乘检测使用
			if (!std::isfinite(angle) || !std::isfinite(this->radius_buff)) {
				this->last_frame_polar_valid = false;
				return -1;
			}
			this->last_angle_time_ms = time;
			this->last_frame_polar_valid = true;//本帧极角已刷新，RANSAC 可采样
			this->turn(angle);//判断正反转
			angle = this->angle2kf(angle);//得到可以累加的角度
			//cout << "累加的角度：" << angle/PI << "PI" << endl;;
            this->angle_date.push_back(angle);//储存角度
            this->angle_time.push_back(time);//储存对应时间
            // Keep the two legacy buffers paired. If an external caller
            // changed one public buffer, the old `&&` trim condition could
            // leave a permanent length mismatch and invalidate future
            // angle_diff_fun calls.
            if (this->angle_date.size() > this->angle_time.size()) {
                this->angle_date.erase(
                    this->angle_date.begin(),
                    this->angle_date.begin() + static_cast<std::ptrdiff_t>(
                        this->angle_date.size() - this->angle_time.size()));
            } else if (this->angle_time.size() > this->angle_date.size()) {
                this->angle_time.erase(
                    this->angle_time.begin(),
                    this->angle_time.begin() + static_cast<std::ptrdiff_t>(
                        this->angle_time.size() - this->angle_date.size()));
            }
            while (this->angle_date.size() > 300) {
                this->angle_date.erase(this->angle_date.begin());
                this->angle_time.erase(this->angle_time.begin());
            }
            this->angle_diff = angle_diff_fun(delta_t, fps);//获得差分角
			//cout << "差分的角度：" << this->angle_diff << endl;;
            if (this->angle_diff != -50){//判断差分角是否正确
                // [JITTER-ANGLE_DIFF] delta tracking
                {
                    static double prev_diff_log = 0;
                    double diff_delta = this->angle_diff - prev_diff_log;
                    if (prev_diff_log != 0 && std::abs(diff_delta) > 0.05) {
                        std::cout << "[JITTER-ANGLE_DIFF] d=" << diff_delta * 180.0 / M_PI
                                  << "deg cur=" << this->angle_diff * 180.0 / M_PI
                                  << "deg prev=" << prev_diff_log * 180.0 / M_PI
                                  << "deg polar=" << this->angle_polar_now * 180.0 / M_PI
                                  << "deg loop=" << this->loop_count << std::endl;
                    }
                    prev_diff_log = this->angle_diff;
                }
                return this->angle_diff;
            }else{
                // cout << "角度差分错误！" << endl;
		        return -1;
            }
		}else{
            // cout << "角度差分错误！" << endl;
		    return -1;
        }

	}

    //预测后角度处理
    cv::Point2f post_predict_angle_process(const double &angle, const double &radius)
	{
        //传入完整的预测角度和半径，得到x,y坐标
        return x_y_cal(angle, radius);;
	}

private:
    int image_width_ = 1280;
    int image_height_ = 1024;
};

