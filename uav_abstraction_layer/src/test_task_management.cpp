//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#include <stdlib.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>

struct CountToTenParameter {
    std::string who_;
    int index_;
};

class BackendTest {
public:

    template <typename Callable, typename ... Args>
    bool safeCall(Callable&& _fn, Args&& ... _args) {
        if (!goToRunning()) { return false; }
        std::bind(_fn, this, std::forward<Args>(_args)...)();
        goToIdle();
        return true;
    }

    void voidFunction() {
        std::cout << "Void!" << std::endl;
    }

    void countToTen(const CountToTenParameter& _in) {
        int count = 0;
        // Check that task is not aborted every control loop
        while (!abort_) {
            count++;
            std::cout << std::endl << "-------------------------------------------------- " << \
                "[" << _in.index_ << "] " << _in.who_ << ": " << count << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (count >= 10) { break; }
        }
    }

    void criticalCountToTen(const std::string& _who, int _index) {
        // As it's critical, abort_ is not considered
        for (int count = 1; count <= 10; count++) {
            std::cout << std::endl << "-------------------------------------------------- " << \
                "[" << _index << "] " << _who << " (critically): " << count << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void abortCurrentTask() {
        running_mutex_.lock();
        if (running_task_) {
            abort_ = true;
        }
        running_mutex_.unlock();
    }

protected:
    bool running_task_ = false;
    std::mutex running_mutex_;
    bool abort_ = false;

    bool goToRunning() {
        std::lock_guard<std::mutex> lock_guard(running_mutex_);
        if (!running_task_) {
            running_task_ = true;
            return true;
        } else {
            return false;
        }
    }

    void goToIdle() {
        std::lock_guard<std::mutex> lock_guard(running_mutex_);
        running_task_ = false;
        abort_ = false;
    }
};

int main(int, char**) {
    // Initialize random seed
    srand(time(NULL));

    BackendTest test;
    test.safeCall(&BackendTest::voidFunction);

    std::string people[5] = { "Alberto", "Barbara", "Carlos", "Diego", "Enrique"};
    for (int i = 0; i < 25; i++) {
        std::string who = people[i%5];
        int odds = rand()%100 + 1;
        if (odds > 50) {
            CountToTenParameter param;
            param.who_ = who;
            param.index_ = i;
            std::thread ([param, &test]() {
                if (!test.safeCall(&BackendTest::countToTen, param)) {
                    std::cout << "[" << param.index_ << "] " << param.who_ << " can't count!" << std::endl;
                }
            }).detach();
        } else {
            std::thread ([who, i, &test]() {
                test.abortCurrentTask();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (!test.safeCall(&BackendTest::criticalCountToTen, who, i)){
                    std::cout << "[" << i << "] " << who << " can't count (critically)!" << std::endl;
                }
            }).detach();
        }
        std::this_thread::sleep_for(std::chrono::seconds(rand()%5 + 1));
    }
    std::this_thread::sleep_for(std::chrono::seconds(250));
    return 0;
}
