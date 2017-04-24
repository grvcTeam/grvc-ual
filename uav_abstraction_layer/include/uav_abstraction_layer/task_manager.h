#ifndef UAV_ABSTRACTION_LAYER_TASK_MANAGER_H
#define UAV_ABSTRACTION_LAYER_TASK_MANAGER_H

#include <thread>
#include <list>
#include <mutex>
#include <chrono>
#include <uav_abstraction_layer/task.h>

namespace grvc { namespace ual {

class TaskManager {
public:

    unsigned int registerTask() {
        Task task;
        task.id_ = getNewId();
        task.state_ = Task::State::WAITING;
        pushTask(task);
        // Wait until task fate is decided
        while (true) {
            updateTasks();
            task = getTask(task.id_);
            if (task.state_ == Task::State::WAITING) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                break;
            }
        }
        return task.id_;
    }

    Task::State getState(unsigned int _id) {
        Task task = getTask(_id);
        return task.state_;
    }

    Task::State deregisterTask(unsigned int _id) {
        Task task = getTask(_id);
        switch (task.state_) {
            case Task::State::WAITING:
            case Task::State::ABORTING:
                task.state_ = Task::State::ABORTED;
                break;
            case Task::State::RUNNING:
                task.state_ = Task::State::FINISHED;
                break;
            case Task::State::ABORTED:
            case Task::State::FINISHED:
            case Task::State::NONE:
                break;
            default:
                assert(false);            
        }
        setTask(task);
        return task.state_;
    }

protected:

    unsigned int getNewId() {
        id_mutex_.lock();
        unsigned int id = id_counter_++;
        id_mutex_.unlock();
        return id;
    }

    void pushTask(const Task& _task) {
        tasks_mutex_.lock();
        tasks_.push_back(_task);
        tasks_mutex_.unlock();
    }

    Task getTask(unsigned int _id) {
        Task task;
        bool task_found = false;
        // Look for task
        tasks_mutex_.lock();
        for (auto t : tasks_) {
            if (t.id_ == _id) {
                task_found = true;
                task = t;
                break;
            }
        }
        tasks_mutex_.unlock();
        // In case task not found
        if (!task_found) {
            task.id_ = _id;
            task.state_ = Task::State::NONE;
        }
        return task;
    }

    bool setTask(const Task& _task) {
        bool task_found = false;
        // Look for task
        tasks_mutex_.lock();
        for (auto &t : tasks_) {
            if (t.id_ == _task.id_) {
                task_found = true;
                t = _task;
                break;
            }
        }
        tasks_mutex_.unlock();
        return task_found;
    }

    void updateTasks() {
        std::cout << std::endl << "Updating tasks:" << std::endl;  // debug!
        bool found_waiting_task = false;
        tasks_mutex_.lock();
        // Last task has more priority
        for (auto rit = tasks_.rbegin(); rit != tasks_.rend(); ++rit) {
            std::cout << *rit;  // debug!
            switch (rit->state_) {
                case Task::State::WAITING:
                    if (!found_waiting_task) {
                        found_waiting_task = true;
                    } else {
                        rit->state_ = Task::State::ABORTING;
                    }
                    break;
                case Task::State::RUNNING:
                    if (found_waiting_task) {
                        rit->state_ = Task::State::ABORTING;
                    }
                    break;
                case Task::State::ABORTING:
                    break;
                case Task::State::ABORTED:
                case Task::State::FINISHED:
                    // Set state for next deletion
                    rit->state_ = Task::State::NONE;
                    break;
                case Task::State::NONE:
                    break;
                default:
                    assert(false);
            }
        }
        // Delete all none-state taks
        tasks_.remove_if([](const Task& _task) {
            return _task.state_ == Task::State::NONE;
        });
        // Only if there's one task waiting, let it run!
        if (tasks_.size() == 1 && found_waiting_task) {
            tasks_.front().state_ = Task::State::RUNNING;
        }
        tasks_mutex_.unlock();
    }

    // Id counter and its mutex
    unsigned int id_counter_ = 0;
    std::mutex id_mutex_;
    // Tasks list and mutex
    std::list<Task> tasks_;
    std::mutex tasks_mutex_;
};

}}  // namespace grvc::ual

#endif  // UAV_ABSTRACTION_LAYER_TASK_MANAGER_H
