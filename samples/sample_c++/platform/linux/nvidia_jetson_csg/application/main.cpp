#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "application.hpp"
#include "dji_logger.h"
#include "dji_platform.h"
#include "subscribe_helper.h"

// some flag
std::string modeFly;
bool kSubscriptionStart = false;

// psdk handler
T_DjiTaskHandle s_flightInitTaskHandle;

#define DJI_TEST_FLIGHT_INIT_TASK_STACK_SIZE 2048

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

  osalHandler->TaskSleepMs(1000);

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
