#pragma once
#include<chrono>

using namespace std;

//一个跨平台的时间处理工具模块提供了时间戳获取休眠控制和时间格式转换功能
typedef long long Timestamp;	// 时间戳类型
typedef long long TimeSpan;		// 时间间隔类型
namespace Tools {
	// 休眠函数宏
	#define Sleep(x) (std::this_thread::sleep_for(chrono::milliseconds(long(x))))

	// 获取当前时间戳
	Timestamp GetTimestamp() {
		return chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
	}

	// 时间戳转日期时间字符串
	std::string Timestamp2HMS(long long timestamp, int timearea) {
		static int Mouth2Day[13] = { 0,31,28,31,30,31,30,31,31,30,31,30,31 };
		static auto Append = [](std::string& src, long value, char end) {
			if (value < 10)
				src += '0' + std::to_string(value);
			else
				src += std::to_string(value);
			if (end)
				src += end;
		};
		std::string result = "";
		long long totalSecond = timestamp / 1000 + timearea * 3600;
		long second = totalSecond % 60, minute = (totalSecond / 60), hour = (minute / 60), day = hour / 24, tday = day + 1, year = 1970, mouth = 1;
		minute %= 60;
		hour %= 24;
		while (tday > 365) {
			if (!(year % 4) || !(year % 400)) {//闰年减366
				tday -= 366;
			}
			else {
				tday -= 365;
			}
			year += 1;
		}
		result += std::to_string(year) + '/';
		bool runYear = !(year % 4) || !(year % 400);
		if (runYear)
			Mouth2Day[2] = 29;
		else
			Mouth2Day[2] = 28;
		while (tday > Mouth2Day[mouth]) {
			tday -= Mouth2Day[mouth];
			mouth += 1;
		}
		Append(result, mouth, '/');
		Append(result, tday, ' ');
		Append(result, hour, ':');
		Append(result, minute, ':');
		Append(result, second, '\0');
		return result;
	}
}