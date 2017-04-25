#include <stdlib.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
//#include <uav_abstraction_layer/task_board.h>

struct CountToTenParameter {
    std::string who_;
    int index_;
};

class BackendTest {
public:

    template <typename Callable, typename ... Args>
    bool safeCall(Callable&& _fn, Args&& ... _args) {
        if (!idle()) { return false; }
        std::bind(_fn, this, std::forward<Args>(_args)...)();
        goToIdle();
        return true;
    }

//std::function<void(const CountToTenParameter&)> count_to_ten = std::bind(&BackendTest::countToTen, &test, count_param);
//if (!test.safeCall(count_to_ten, count_param))

    // template<typename T_>
    // bool safeCall(std::function<void(const T_&)>& _f, const T_& _parameter) {
    //     if (!idle()) { return false; }
    //     _f(_parameter);
    //     goToIdle();
    //     return true;
    // }

    void testVoidFunction() {
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
        // while (running_task_) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
        // Wait outside this function!
    }

protected:
    bool running_task_ = false;
    std::mutex running_mutex_;

    bool abort_ = false;

    bool idle() {
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
    BackendTest test;
    test.safeCall(&BackendTest::testVoidFunction);
    //grvc::ual::TaskBoard task_board;
    //--------------
    /*std::thread ta([&](){
        test.countToTen("Ana");
    });
    std::thread tb([&](){
        test.countToTen("Barbara");
    });
    std::this_thread::sleep_for(std::chrono::seconds(3));
    test.abortCurrentTask();

    std::thread tc([&](){
        test.criticalCountToTen("Barbara");
    });
    std::thread td([&](){
        test.countToTen("Ana");
    });
    test.abortCurrentTask();


    std::this_thread::sleep_for(std::chrono::seconds(1000));*/
    //--------------
    // std::thread first_count([&]() {
    //     test.countToTen("Me", -1);
    // });
    std::string people[5] = { "Alberto", "Barbara", "Carlos", "Diego", "Enrique"};

    // Initialize random seed
    srand(time(NULL));

    for (int i = 0; i < 25; i++) {
        std::string who = people[i%5];
        CountToTenParameter count_param;
        count_param.who_ = who;
        count_param.index_ = i;
        int odds = rand()%100 + 1;
        std::thread count_thread;
        if (odds > 50) {
            std::thread ([count_param, &test]() {
                if (!test.safeCall(&BackendTest::countToTen, count_param)) {
                    std::cout << "[" << count_param.index_ << "] " << count_param.who_ << " can't count!" << std::endl;
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

    //first_count.join();
    std::this_thread::sleep_for(std::chrono::seconds(250));
    return 0;
}
