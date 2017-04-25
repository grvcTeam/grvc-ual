#include <stdlib.h>
#include <time.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>


class BackendTest {
public:

    bool countToTen(const std::string& _who, int _index) {
        // TODO: Wrap it!
        if (!idle()) { return false; }

        int count = 0;
        // Check that task is not aborted every control loop
        while (!abort_) {
            count++;
            std::cout << std::endl << "-------------------------------------------------- " << \
                "[" << _index << "] " << _who << ": " << count << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (count >= 10) { break; }
        }

        // TODO: Wrap it!
        abort_=false;
        running_task_ = false;
        return true;
    }

    bool criticalCountToTen(const std::string& _who, int _index) {
        // TODO: Wrap it!
        if (!idle()) { return false; }

        for (int count = 1; count <= 10; count++) {
            std::cout << std::endl << "-------------------------------------------------- " << \
                "[" << _index << "] " << _who << " (critically): " << count << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // TODO: Wrap it!
        abort_ = false;
        running_task_ = false;
        return true;
    }

    void abortCurrentTask(){
        abort_ = true;
        while (running_task_) { std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
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
};

int main(int, char**) {
    BackendTest test;
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
    std::thread first_count([&]() {
        test.countToTen("Me", -1);
    });
    std::string people[5] = { "Alberto", "Barbara", "Carlos", "Diego", "Enrique"};

    // Initialize random seed
    srand(time(NULL));

    for (int i = 0; i < 25; i++) {
        std::this_thread::sleep_for(std::chrono::seconds(rand()%5 + 1));
        std::string who = people[i%5];
        int odds = rand()%100 + 1;
        std::thread count_thread;
        if (odds > 50) {
            std::thread ([who, i, &test]() {
                if (!test.countToTen(who, i)){
                    std::cout << "[" << i << "] " << who << " can't count!" << std::endl;
                }
            }).detach();
        } else {
            std::thread ([who, i, &test]() {
                test.abortCurrentTask();
                if (!test.criticalCountToTen(who, i)){
                    std::cout << "[" << i << "] " << who << " can't count (critically)!" << std::endl;
                }
            }).detach();
        }
    }

    first_count.join();
    std::this_thread::sleep_for(std::chrono::seconds(250));
    return 0;
}
