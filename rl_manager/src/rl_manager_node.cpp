#include <rl_manager/rl_manager.h>


int main(int argc, char **argv) {
    // initialize ros
    ros::init(argc, argv, "rl_manager_node");
    RL_manager* rl_man_ptr = new RL_manager();
    return 0;
}
