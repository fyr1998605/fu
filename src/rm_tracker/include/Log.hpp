#pragma once
#ifndef LOG
#define LOG
#include "UTools.hpp"
#include <iostream>
// 老登遗留: 日志打印工具,目前未使用,未经过测试
namespace Tools {
	enum LogLevel {
		Debug,
		Info,
		Warn,
		Error,
		Fatal
	};
	class Log {
	public:
		typedef LogLevel Level;
		static const LogLevel MinimumLogLevel = LogLevel::Info; // ��־�����ͼ���
		static void Logging(std::string info, Level level); // ��־���(����̨����������ļ�)
		static void Debug(std::string info); // ��־���(����̨����������ļ�)
		static void Info(std::string info); // ��־���(����̨����������ļ�)
		static void Warn(std::string info); // ��־���(����̨����������ļ�)
		static void Error(std::string info); // ��־���(����̨����������ļ�)
		static void Fatal(std::string info); // ��־���(����̨����������ļ�)
	};
	void Log::Logging(std::string info, Level level) {
		long long currentTime = GetTimestamp();
		//�����������ͼ�������
		if(level >= MinimumLogLevel) {
			std::string print = "";
			switch (level) {
			case LogLevel::Debug:
				print += "\033[36m[DEBUG][";
				break;
			case LogLevel::Info:
				print += "\033[36m[INFO ][";
				break;
			case LogLevel::Warn:
				print += "\033[33m[WARN ][";
				break;
			case LogLevel::Error:
				print += "\033[31m[ERROR][";
				break;
			case LogLevel::Fatal:
				print += "\033[1;31m[FATAL][";
				break;
			}
			print += Timestamp2HMS(currentTime,8) + "]\t" + info + "\033[0m";
			cout << print << endl;
		}
	}
	void Log::Debug(std::string info) {
		Logging(info, LogLevel::Debug);
	}
	void Log::Info(std::string info) {
		Logging(info, LogLevel::Info);
	}
	void Log::Warn(std::string info) {
		Logging(info, LogLevel::Warn);
	}
	void Log::Error(std::string info) {
		Logging(info, LogLevel::Error);
	}
	void Log::Fatal(std::string info) {
		Logging(info, LogLevel::Fatal);
	}
}
#endif