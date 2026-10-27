#pragma once
#include <thread>
#include <exception>
#include <vector>
#include <mutex>
#include <queue>
#include "UTools.hpp"
#ifndef __linux__
#include <windows.h>
SYSTEM_INFO sysInfo;
#else
#include "unistd.h"
#include "sys/sysinfo.h"
#endif
using namespace std;
using namespace Tools;

// 跨平台的C++线程池库
// 老登遗留半成品
// 未经过使用和测试
// 仅供参考
namespace Tools {
	
	// 定义一个函数指针类型别名
	// 定义线程任务函数的类型,便于统一管理回调函数 
	typedef void (*Func_Ptr)(void*);

	// 线程池默认线程数量限制
	#define THREAD_DEFAULT_LIMIT 10

	// 主模板声明
	template<typename...An>
	class Meta {};
	
	// 单参数特化
	template<typename T>
	class Meta<T> {
	public:
		T value;
		static const int length = sizeof(T);
		static const int count = 1;
		Meta(void* ptr, T v) :value(std::move(v)) {
			// std::cout << "Address:\t" << (long long)ptr << "\t -> \t" << v << std::endl;
			*((T*)ptr) = v;
		};
	};
	
	// 多参数递归特化
	template<typename T, typename...An>
	class Meta<T, An...> :public Meta<An...>
	{
	public:
		typedef Meta<An...> base;
		T value;
		static const int length = base::length + sizeof(T);
		static const int count = base::count + 1;
		Meta(void* ptr, T v, An...args) :base(((char*)ptr + sizeof(T)), args...), value(std::move(v)) {
			// std::cout << "Address:\t" << (long long)ptr << "\t -> \t" << v << std::endl;
			*((T*)ptr) = v;
		}
	};
	
	// 编译器类型查询系统,用于Meta模板中获取特定位姿的类型信息和偏移量
	// 主模板声明
	template<int N, class T, class...An>
	class MetaInfo {

	};
	
	// 索引0的特化(递归基)
	template<class T, class...An>
	class MetaInfo<0, Meta<T, An...>> {
	public:
		typedef T Type;
		static const int Offset = 0;
		MetaInfo(void* ptr) {

		}
	};
	
	// 递归特化(N>0)
	template<int N, class T, class...An>
	class MetaInfo<N, Meta<T, An...>> {
	public:
		typedef MetaInfo<N - 1, Meta<An...>> base;
		typedef class base::Type Type;
		static const int Offset = base::Offset + sizeof(T);
		MetaInfo(void* ptr) :MetaInfo<N - 1, Meta<An...>>(ptr) {

		}
	};
	
	// 线程任务的状态机
	enum class TaskState {   
		free,	   // 空闲状态
		operating, // 执行中状态
		completed, // 已完成状态
		error,	   // 错误状态
		unknow	   // 未知状态
	};

	// 线程池的任务封装类
	// 它将一个待执行的任务,相关参数,回调函数和执行状态打包成一个完整的任务单元
	class ThreadTask {      
	public:
		unsigned int tid;							// 执行任务的线程ID
		std::function<void(void*)> _FUNC_;			// 要执行的主要任务函数
		void (*_CALL_BACK_)(ThreadTask&);			// 任务成功完成后的回调
		void (*_ERROR_CALL_BACK_)(std::exception&);	// 任务出错时的回调
		void* _PARAMETERS_;							// 传递给任务函数的参数
		TaskState _TASK_STATE_;						// 当前任务状态
		std::exception _ERROR_INFO_;				// 存储异常通信信息
		long _START_TIME_STRAMP_;					// 任务开始时间戳
		long _END_TIME_STRAMP_;						// 任务结束时间戳

		// 默认构造函数: 创建一个空任务或未初始化的任务
		ThreadTask()
		{
			this->tid = 0;							// 线程ID
			this->_FUNC_ = 0;						// 空函数
			this->_CALL_BACK_ = 0;					// 空回调
			this->_ERROR_CALL_BACK_ = 0;			// 空错误回调
			this->_PARAMETERS_ = 0;					// 空参数
			this->_TASK_STATE_ = TaskState::unknow;	// 未知状态
			this->_START_TIME_STRAMP_ = -1;			// 无效时间戳
		}

		// 带时间戳的构造函数
		// 允许指定精确的开始时间戳
		ThreadTask(std::function<void(void*)> func, void (*callback)(ThreadTask&), void (*error_callback)(std::exception&), void* parameters, TaskState task_state, long timestramp) {
			this->tid = 0;
			this->_FUNC_ = func;
			this->_CALL_BACK_ = callback;
			this->_ERROR_CALL_BACK_ = error_callback;
			this->_PARAMETERS_ = parameters;
			this->_TASK_STATE_ = task_state;
			this->_START_TIME_STRAMP_ = timestramp;
		}

		// 不带时间戳的构造函数
		// 使用默认时间戳
		ThreadTask(std::function<void(void*)> func, void (*callback)(ThreadTask&), void (*error_callback)(std::exception&), void* parameters, TaskState task_state) {
			this->tid = 0;
			this->_FUNC_ = func;
			this->_CALL_BACK_ = callback;
			this->_ERROR_CALL_BACK_ = error_callback;
			this->_PARAMETERS_ = parameters;
			this->_TASK_STATE_ = task_state;
			this->_START_TIME_STRAMP_ = 0;
		}

		// 检查任务是否为空
		bool Empty() {
			if (!this)
				return true;
			return (this->_FUNC_ == NULL || this->_START_TIME_STRAMP_ < 0 || _TASK_STATE_ == TaskState::unknow);
		}

		// 静态工厂方法创建空任务
		static ThreadTask Zero() {
			return ThreadTask(NULL, NULL, NULL, NULL, TaskState::unknow, -1);
		}

		// 修改任务状态
		void ChangeState(TaskState state) {
			this->_TASK_STATE_ = state;
		}
	};
	
	// 线程池类型配置枚举,用于指定线程池的工作模式和线程数量策略
	enum class PoolType {
		CPUDense = 0,	// 最大化CPU利用率
		CPUOnly = 1,	// 每个物理核心一个线程
		IODense = 2		// 大量线程处理I/O等待
	};
	
	class ThreadPool;  
	
	// 线程池中的工作线程疯转
	// 封装了一个实际的执行线程,负责接收任务,执行任务并管理自身状态
	// 每个Thread对象对应一个物理/逻辑线程
	class Thread {     
	private:
		unsigned int id;		 // 线程唯一标识符
		thread main;			 // 实际的线程库对象
		ThreadTask* task = NULL; // 指向当前要执行的任务的指针
		bool free = true;	     // 线程是否空闲
		bool function = true;	 // 线程是否在运行
	public:
		Thread(ThreadPool&);
		Thread();
		bool Init(ThreadPool&);
		bool GiveTask(ThreadTask&);
		bool Handle(ThreadTask&);
		bool Handle(ThreadTask**);
		bool Function();
		bool Free() {
			return free;
		}
		bool Done() {
			task = NULL;
			free = true;
			return free;
		}
		bool Shutdown();
		unsigned int ID();
	};
	
	// 线程池的核心管理器,实现了生产者-消费者模式的异步任务执行框架
	class ThreadPool {
	private:
		// 最大线程数限制,实际创建的线程数,当前正在执行任务的线程数
		int limit, count, inuse;
		// 指向线程数组的起始指针,指向线程数组末尾的指针
		Thread* list,*list_end;
		// 空闲线程队列(FIFO)
		queue<Thread*> free;
		// 正在执行的任务指针列表
		vector<ThreadTask*> inUse;
		// 等待执行的任务队列(FIFO)
		queue<ThreadTask> tasks;
		// 管理线程,负责调度任务
		thread manager;
		// 线程池类型(决定线程数量策略)
		PoolType type;
		// 同步机制: 参数锁,任务队列锁,空闲队列锁,使用计数锁
		mutex lock_parameters, lock_queue, lock_free, lock_inuse;
		// 运行标志
		bool function = true;
	public:

		// 初始化线程池,创建所有线程
		ThreadPool(PoolType type, int limit = -1) {
			std::cout<<"Thread pool start initialize"<<std::endl;
			if (limit != 0 && limit >= -1)
				this->limit = limit;
			else
				this->limit = THREAD_DEFAULT_LIMIT;
			if (~this->limit)
				this->count = limit;
			else
			{
				this->type = type;
#ifndef __linux__
				if (type == PoolType::CPUDense)
				{
					GetSystemInfo(&sysInfo);
					this->count = 2 * sysInfo.dwNumberOfProcessors;
				}
				else if (type == PoolType::CPUOnly) {
					GetSystemInfo(&sysInfo);
					this->count = sysInfo.dwNumberOfProcessors;
				}
				else if (type == PoolType::IODense) {
					GetSystemInfo(&sysInfo);
					this->count = 10 * sysInfo.dwNumberOfProcessors - 1;
				}
				else
					this->count = 4;
				this->limit = this->count;
#else
#endif
			}
			Thread* list = new Thread[limit];
			list_end = list + limit;
			lock_parameters.lock();
			for (int i = 0; i < this->count; i++) {
				(list + i)->Init(*this);
				// std::cout<<"Thread: " << (list + i)->ID() <<std::endl;
			}
			lock_parameters.unlock();
			manager = thread(&ThreadPool::Manager, this);
			manager.detach();
			switch (type)
			{
			case PoolType::CPUDense:
				std::cout<<"Thread pool was initialized!(type:CPU Dense|limit:" << limit << ')'<<std::endl;
				break;
			case PoolType::CPUOnly:
				std::cout<<"Thread pool was initialized!(type:CPU Only|limit:" << limit << ')'<<std::endl;
				break;
			case PoolType::IODense:
				std::cout<<"Thread pool was initialized!(type:IO Dense|limit:" << limit << ')'<<std::endl;
				break;
			default:
				std::cout<<"Thread pool was initialized!(type:Unknow|limit:Unknow)"<<std::endl;
				break;
			}
		}
		
		~ThreadPool() {

		}

		// 工作线程主循环
		// 每个工作线程的执行入口
		bool CheckSignal = false;
		void Tasker(Thread* param) {
			unsigned int id = static_cast<unsigned int>(std::hash<std::thread::id>()(std::this_thread::get_id()));
			lock_free.lock();
			free.push(param);
			// std::cout << "Thread:" << param.ID() << " in queue!" << std::endl;
			lock_free.unlock();
			ThreadTask* task = NULL;
			while (param->Function()) {
				if (CheckSignal) {
					param->Done();
					lock_free.lock();
					//std::cout << std::this_thread::get_id() << " Check!" << std::endl;
					free.push(param);
					lock_free.unlock();
				}
				if (!param->Handle(&task) || !task) {
					// std::this_thread::sleep_for(chrono::milliseconds(5));
					continue;
				}
				task->tid = id;
				task->_TASK_STATE_ = TaskState::operating;
				try {
					task->_FUNC_(task->_PARAMETERS_);
					if (task->_CALL_BACK_)
						task->_CALL_BACK_(*task);
					task->ChangeState(TaskState::completed);
				}
				catch (std::exception e) {
					try {
						if (!task->_ERROR_CALL_BACK_)
							task->_ERROR_CALL_BACK_(e);
					}
					catch (std::exception e) {}
					task->_TASK_STATE_ = TaskState::error;
				}
				param->Done();
				lock_inuse.lock();
				inuse -= 1;
				lock_inuse.unlock();
				lock_free.lock();
				//std::cout << std::this_thread::get_id() << " Free" << std::endl;
				free.push(param);
				lock_free.unlock();

				// std::this_thread::sleep_for(chrono::milliseconds(5));
				Sleep(2);
			}
		}
		
		// 获取等待任务数
		// 返回当前等待执行的任务数量
		// 用于用户监控线程池负载情况
		int GetTaskCount()
		{
			lock_queue.lock();
			int result = tasks.size();
			// std::cout << " Task:" << tasks.size() << std::endl;
			lock_queue.unlock();
			return result;
		}

		// 检查正在执行的任务
		void CheckInUse() {
			// lock_free.lock();
			// // std::cout << "Free thread count:" << free.size();
			// // std::cout << " Inuse:" << inuse;
			// // std::cout << " Task:" << tasks.size() << "    " << '\r';
			// lock_free.unlock();
			// if (inUse.size() == 0)
			// 	return;
			//vector<vector<ThreadTask*>::iterator> removes;
			//for (auto i = inUse.begin(); i != inUse.end(); i++) {
			//	if ((*i)->_TASK_STATE_ == TaskState::error || (*i)->_TASK_STATE_ == TaskState::completed)
			//		removes.push_back(i);
			//		//inUse.erase(i);
			//}
			//for (auto& i : removes) {
			//	try {
			//		if ((*i)->Empty())
			//			continue;
			//		inUse.erase(i);
			//	}
			//	catch (std::runtime_error e) {
			//		std::cout << "Exception to erase task:" << e.what() << std::endl;
			//	}
			//}
			//removes.clear();
		}

		// 打印状态信息
		void INFO(){
			lock_free.lock();
			lock_queue.lock();
			std::cout << "FTC:" << free.size();
			std::cout << " Inuse:" << inuse;
			std::cout << " Task:" << tasks.size() << std::endl;
			lock_queue.unlock();
			lock_free.unlock();
		}

		// 管理主线程
		// 作用: 任务调度器,连接任务队列和空闲线程
		const int ManagerTimespan = 0;
		void Manager() {
			lock_inuse.lock();
			inuse = 0;
			lock_inuse.unlock();
			ThreadTask task;
			Thread* th;
			while (function) {
				std::this_thread::sleep_for(chrono::milliseconds(ManagerTimespan));
				CheckInUse();
				if (inuse >= count || tasks.size() == 0) {
					continue;
				}
				lock_free.lock();
				//if (inuse + free.size() != count) {
				//	CheckSignal = true;
				//	std::cout << "List error start to check!" << std::endl;
				//	while (free.size() != 0)
				//		free.pop();
				//}
				//else {
				//	CheckSignal = false;
				//}
				lock_free.unlock();
				lock_queue.lock();
				task = tasks.front();
				tasks.pop();
				lock_queue.unlock();
				if (task.Empty()) {
					std::this_thread::sleep_for(chrono::milliseconds(ManagerTimespan));
					continue;
				}
				while (free.size() == 0) {
					CheckInUse();
					std::this_thread::sleep_for(chrono::milliseconds(ManagerTimespan));
				}
				lock_free.lock();
				th = free.front();
				free.pop();
				lock_free.unlock();
				if (!th->Function() || !th->Free()) {
					std::this_thread::sleep_for(chrono::milliseconds(ManagerTimespan));
					continue;
				}
				lock_inuse.lock();
				inuse += 1;
				lock_inuse.unlock();
				th->GiveTask(task);
				inUse.push_back(&task);

				std::this_thread::sleep_for(chrono::milliseconds(ManagerTimespan));
			}
		}
		
		// 任务添加函数: 用户接口,向线程池提交任务
		void AddTask(ThreadTask task) {
				lock_queue.lock();
				tasks.push(task);
				lock_queue.unlock();
		}

		void AddTask(std::function<void(void*)> func, void* parameters = NULL) {
				lock_queue.lock();
				tasks.push(ThreadTask(func, NULL, NULL, parameters, TaskState::free));
				lock_queue.unlock();
		}
		template<class Fc, class...An>
		void AddTask() {

		}
	};


	Thread::Thread(ThreadPool& pool) {
		// main = thread(&ThreadPool::Tasker,this, &pool);
		// id = static_cast<unsigned int>(std::hash<std::thread::id>()(main.get_id()));
		// main.detach();
	}

	Thread::Thread()
	{}
	
	// 线程初始化
	bool Thread::Init(ThreadPool& pool){
		auto t = &ThreadPool::Tasker;
		main = thread([this,&pool](){pool.Tasker(this);});
		id = static_cast<unsigned int>(std::hash<std::thread::id>()(main.get_id()));
		main.detach();
		return true;
	}
	
	// 给线程分配一个任务
	bool Thread::GiveTask(ThreadTask& task) {
		if (!free || task.Empty())
			return false;
		this->task = &task;
		return true;
	}
	
	// 获取任务
	// 通过引用返回
	bool Thread::Handle(ThreadTask& task) {
		task = ThreadTask::Zero();
		if (!free || this->task->Empty())
			return false;
		free = false;
		task = *(this->task);
		this->task = NULL;
		return true;
	}
	
	// 获取任务
	// 通过指针返回
	bool Thread::Handle(ThreadTask** task) {
		*task = NULL;
		if (!free || !this->task || this->task->Empty())
			return false;
		free = false;
		*task = this->task;
		this->task = NULL;
		return true;
	}
	
	// 检查线程是否运行
	bool Thread::Function() {
		return function;
	}
	
	// 关闭线程
	bool Thread::Shutdown() {
		function = false;
		return function;
	}
	
	// 获取线程ID
	unsigned int Thread::ID() {
		return id;
	}
}
