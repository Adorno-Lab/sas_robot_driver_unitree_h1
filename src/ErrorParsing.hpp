#include <cstdint>
#include <string>
#include <unordered_map>

static const std::unordered_map<int32_t, std::string> get_readable_error = {
    // error.hpp
    {-1, "Unknown error"},
    {0, "Success"},
    {100, "Invalid parameter "},
    {1001, "common error"},
    {1002, "Bad cast error"},
    {1003, "Future error"},
    {1004, "Future fault error"},
    {1005, "Json data error"},
    {1006, "System error"},
    {1007, "File operation error"},
    {1008, "Socket operation error"},
    {1009, "IO operation error"},
    {1010, "Lock operation error"},
    {1011, "Network error"},
    {1012, "Timeout error"},
    // dds_error.hpp
    {2001, "dds error"},
    // sport_error.hpp
    {4101, "point path error."},
    {4201, "server overtime."},
    {4205, "server function not init."},
    // robot_state_error.hpp
    {5201, "service switch error."},
    {5202, "service is protected."},
    {5203, "low power switch error."},
    {5204, "low power state error."},
    // motion_switcher_error.hpp
    {7001, "parameter is invalid."},
    {7002, "switcher is busy."},
    {7003, "event is invalid."},
    {7004, "name or alias is invalid."},
    {7005, "name or alias is invalid."},
    {7006, "check cmd execute error."},
    {7007, "select cmd execute error."},
    {7008, "release cmd execute error."},
    {7009, "save customize data error."},
    // g1_arm_action_error.hpp
    {7400, "The topic rt/armsdk is occupied."},
    {7401, "The arm is holding. Expecting release action(99) or the same last action id."},
    {7402, "Invalid action id."},
    {7404, "Invalid fsm id."},
    // r1_loco_error.hpp
    {7304, "FSM ID return denied."},
    // g1_loco_error.hpp
    {7301, "LocoState not available."},
    {7302, "Invalid fsm id."},
    {7303, "Invalid task id."},
    // h1_loco_error.hpp
    {8301, "LocoState not available."},
    {8302, "Invalid fsm id."},
    {8303, "OdomState not available."},
    {8304, "Invalid task id."},
    // internal_error.hpp
    {3001, "Unknown error."},
    {3102, "Send request error."},
    {3103, "Api is not registered."},
    {3104, "Call api timeout error."},
    {3105, "Response api not match error."},
    {3106, "Response data error."},
    {3107, "Lease is invalid."},
    {3201, "Send response error."},
    {3202, "Server internal error."},
    {3203, "Api not implement error."},
    {3204, "Api parameter error."},
    {3205, "Request denied by lease."},
    {3206, "Lease not exist in server cache."},
    {3207, "Lease is already exist in server cache."},
    // config_error.hpp
    {8201, "parameter error."},
    {8202, "config name is not found."},
    {8203, "name is invalid."},
    {8204, "name/content length limited."},
    {8205, "lock error."},
    {8206, "load meta error."},
    {8207, "save meta error."},
    {8208, "save meta temp error."},
    {8209, "formalize meta error."},
    {8210, "remove meta error."},
    {8211, "load data error."},
    {8212, "save data error."},
    {8213, "save data temp error."},
    {8214, "formalize data error."},
    {8215, "remove data error."},
    // g1_agv_error.hpp
    {9101, "Module not initialized."},
    {9102, "Failed to execute move command."},
    {9103, "Failed to execute height adjust command."},
};
