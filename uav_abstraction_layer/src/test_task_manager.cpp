#include <uav_abstraction_layer/task_manager.h>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace grvc::ual;

class BackendTest {
public:

    // Important: pass argument by value!
    void countToTen(const std::string _who) {
        int count = 0;
        unsigned int id = task_manager_.registerTask();
        std::cout << std::endl << _who << " registered a new task with id = " << id << std::endl;
        // Check that task should be running every control loop
        while (task_manager_.getState(id) == Task::State::RUNNING) {
            count++;
            std::cout << std::endl << "-------------------------------------------------- " << \
                _who << ": " << count << std::endl;
            this_thread::sleep_for(chrono::seconds(1));
            if (count >= 10) { break; }
        }
        task_manager_.deregisterTask(id);
    }

    // Important: pass argument by value!
    void criticalCountToTen(const std::string _who) {
        unsigned int id = task_manager_.registerTask();
        std::cout << std::endl << _who << " registered a new task with id = " << id << std::endl;
        // Check at least once that task should be running
        if (task_manager_.getState(id) == Task::State::RUNNING) {
            for (int count = 1; count <= 10; count++) {
                std::cout << std::endl << "-------------------------------------------------- " << \
                    _who << " (critically): " << count << std::endl;
                this_thread::sleep_for(chrono::seconds(1));
            }
        }
        task_manager_.deregisterTask(id);
    }

protected:
    TaskManager task_manager_;
};

int main(int, char**) {
    BackendTest test;
    
    std::thread first_count([&]() {
        test.countToTen("Me");
    });
    std::string people[5] = { "Alberto", "Barbara", "Carlos", "Diego", "Enrique" };
 
    // Initialize random seed
    srand(time(NULL));

    for (int i = 0; i < 25; i++) {
        this_thread::sleep_for(chrono::seconds(rand()%13 + 1));
        std::string who = people[i%5];
        int odds = rand()%100 + 1;
        std::thread count_thread;
        if (odds > 20) {
            std::thread ([&]() {
                test.countToTen(who);
            }).detach();
        } else {
            std::thread ([&]() {
                test.criticalCountToTen(who);
            }).detach();
        }
    }

    first_count.join();
    return 0;
}
