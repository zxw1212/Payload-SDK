#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "application.hpp"
#include "dji_logger.h"
#include "dji_platform.h"
#include "forward_status_mqtt.h"
#include "consume_command_mqtt.h"
#include "subscribe_helper.h"

// some flag
int i_Mode = 0;
std::string modeFly;
int i_CAMERA_MODE = 0;
bool bool_RTMPFlag;
bool kSubscriptionStart = false;

// psdk handler
T_DjiTaskHandle s_flightInitTaskHandle;
T_DjiTaskHandle s_flightStatusForwardTaskHandle;
T_DjiTaskHandle s_consumeUserCommandTaskHandle;

#define DJI_TEST_FLIGHT_INIT_TASK_STACK_SIZE 2048
#define DJI_TEST_FLIGHT_STATUS_FORWARD_TASK_STACK_SIZE 2048
#define DJI_TEST_CONSUME_USER_COMMAND_TASK_STACK_SIZE 2048

int main(int argc, char** argv) {
  Application application(argc, argv);
  // start more thread tasks
  T_DjiReturnCode returnCode;
  T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
  if (!osalHandler) {
    USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
    return -1;
  }

  modeFly = "INIT";
  // start a thread to init flight ctrl and subscribe the topic list
  returnCode = osalHandler->TaskCreate("DjiUser_ModuleInitAndSubscribeTopics",
                                       DjiUser_ModuleInitAndSubscribeTopics,
                                       DJI_TEST_FLIGHT_INIT_TASK_STACK_SIZE,
                                       NULL, &s_flightInitTaskHandle);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "DjiUser_ModuleInitAndSubscribeTopics task failed, errno = 0x%08llX",
        returnCode);
    goto Deinit;
  }
  kSubscriptionStart = true;
  osalHandler->TaskSleepMs(5000);

  // start a thread to forwarding pub the topics via MQTT
  returnCode =
      osalHandler->TaskCreate("status_forward_task", ForwardStatusMqtt_Task,
                              DJI_TEST_FLIGHT_STATUS_FORWARD_TASK_STACK_SIZE,
                              NULL, &s_flightStatusForwardTaskHandle);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("status_forward_task task failed, errno = 0x%08llX",
                   returnCode);
    goto Deinit;
  }

  osalHandler->TaskSleepMs(5000);

  // start a thread to get user command via MQTT
  returnCode = osalHandler->TaskCreate(
      "consume_user_command_task", ConsumeUserCommand_Task,
      DJI_TEST_CONSUME_USER_COMMAND_TASK_STACK_SIZE, NULL,
      &s_consumeUserCommandTaskHandle);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("consume_user_command_task task failed, errno = 0x%08llX",
                   returnCode);
    goto Deinit;
  }
  
  // keep running
  USER_LOG_INFO("App keep running....\n");
  while (1) {
    osalHandler->TaskSleepMs(5000);
  }

Deinit:
  returnCode = DjiUser_ModuleDeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit Flight Control sample failed,error code:0x%08llX",
                   returnCode);
    return returnCode;
  }

  return 0;
}
