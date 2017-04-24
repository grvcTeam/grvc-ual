#ifndef UAV_ABSTRACTION_LAYER_TASK_H
#define UAV_ABSTRACTION_LAYER_TASK_H

#include <ctime>
#include <chrono>
#include <iostream>
#include <assert.h>

namespace grvc { namespace ual {

class Task {
public:
    enum State { WAITING, RUNNING, ABORTING, ABORTED, FINISHED, NONE };

    Task() {
        id_ = 0;
        state_ = State::NONE;
        creation_time_ = std::chrono::system_clock::now();
    }

    bool operator<(const Task& _task) { return creation_time_ < _task.creation_time_; }

    static std::string stateToString(const Task::State& _state) {
        std::string out;
        switch (_state) {
            case State::WAITING:
                out = "waiting";
                break;
            case State::RUNNING:
                out = "running";
                break;
            case State::ABORTING:
                out = "aborting";
                break;
            case State::ABORTED:
                out = "aborted";
                break;
            case State::FINISHED:
                out = "finished";
                break;
            case State::NONE:
                out = "none";
                break;
            default:
                assert(false);
        }
        return out;
    }

    unsigned int id_;
    State state_;
    std::chrono::time_point<std::chrono::system_clock> creation_time_;
};

std::ostream& operator<<(std::ostream& _os, const Task& _task) {
    std::time_t creation = std::chrono::system_clock::to_time_t(_task.creation_time_);
    _os << '[' << _task.id_ << "]: " << Task::stateToString(_task.state_) << ", created in " << std::ctime(&creation);
    return _os;
}

}}  // namespace grvc::ual

#endif  // UAV_ABSTRACTION_LAYER_TASK_H
