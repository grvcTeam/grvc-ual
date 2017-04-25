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

    Task(unsigned int _id, State _state) : id_(_id), state_(_state) {}
    unsigned int id_;
    State state_;

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
};

std::ostream& operator<<(std::ostream& _os, const Task& _task) {
    _os << '[' << _task.id_ << "]: " << Task::stateToString(_task.state_) << std::endl;
    return _os;
}

}}  // namespace grvc::ual

#endif  // UAV_ABSTRACTION_LAYER_TASK_H
