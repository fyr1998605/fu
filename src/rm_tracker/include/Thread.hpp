#pragma once
#include <thread>

// 老登遗留: 未经过使用和测试
// 仅供参考
class Thread {
private:
	thread main;							
	ThreadTask task = ThreadTask::Zero();	
	bool free = true;						
	bool function = true;					
public:
	Thread(ThreadPool& pool) {
		main = thread(&ThreadPool::Tasker, &pool);
		main.detach();
	}
	~Thread() {

	}

	bool GiveTask(ThreadTask& task) {
		if (!free || task.Empty())
			return false;
		this->task = task;
		return true;
	}

	bool Handle(ThreadTask& task) {
		if (!free || task.Empty())
			return false;
		task = this->task;
		return true;
	}

	bool Function() {
		return function;
	}

	bool Shutdown() {
		function = false;
		return function;
	}

	thread::id ID() {
		return main.get_id();
	}
};