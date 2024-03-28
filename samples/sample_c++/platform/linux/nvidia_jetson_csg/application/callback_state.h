#ifndef _CALLBACK_STATE_
#define _CALLBACK_STATE_

constexpr int callback_success_code = 1001;
constexpr int callback_unknown_error = 4000;
constexpr int callback_aircraft_error = 4001;
constexpr int callback_json_error = 4002;
constexpr int callback_database_error = 4003;
constexpr int callback_device_unconn = 4004;
constexpr int callback_port_error = 4005;
constexpr int callback_param_error = 4006;
constexpr int callback_hangar_unavailable = 9001;
constexpr int callback_task_non_existent = 9002;
constexpr int callback_task_cancel = 9003;

#endif
