#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "camera_stream_inference_rtmp.h"
#include "consume_command_mqtt.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "forward_status_mqtt.h"
#include "mqtt/async_client.h"
#include "subscribe_helper.h"
#include <json/json.h>
#include <stdio>

// mqtt client and topics
mqtt::topic* topic_cb;
mqtt::async_client* cli;
std::string s_CLIENT_ID;
std::string s_RTMP_URI;
std::string s_HTTP_URI;
std::string s_TOPIC_MISSION;
std::string s_TOPIC_GENERAL;
std::string s_TOPIC_RC;
std::string s_TOPIC_CAMERA;
std::string s_TOPIC_V3;
std::string s_TOPIC_FAN;

// some flag
int i_Mode = 0;
std::string modeFly;
int i_CAMERA_MODE = 0;
bool bool_RTMPFlag;
bool kSubscriptionStart = false;

// time info
char timeDay[80];
char timeSec[80];

// mqtt param
constexpr int kQOS = 1;
constexpr int kMAX_BUFFERED_MESSAGES = 1200;

// psdk handler
T_DjiTaskHandle s_rpiRtmpThread;
T_DjiTaskHandle s_flightInitTaskHandle;
T_DjiTaskHandle s_flightStatusForwardTaskHandle;
T_DjiTaskHandle s_consumeUserCommandTaskHandle;

#define RPI_RTMP_TASK_STACK_SIZE 2048
#define DJI_TEST_FLIGHT_INIT_TASK_STACK_SIZE 2048
#define DJI_TEST_FLIGHT_STATUS_FORWARD_TASK_STACK_SIZE 2048
#define DJI_TEST_CONSUME_USER_COMMAND_TASK_STACK_SIZE 2048

int main(int argc, char** argv) {
  // setup param of MQTT
  std::ifstream ifs2;
  ifs2.open("/sys/firmware/devicetree/base/serial-number", ios::in);
  if (!ifs2.is_open()) {
    std::cout << "open file /sys/firmware/devicetree/base/serial-number error!"
              << std::endl;
    return -1;
  }
  char cpuid[128];
  memset(cpuid, 0, 128);
  while (ifs2 >> cpuid) {
    std::cout << "cpu id is: " << cpuid << std::endl;
  }
  std::ifstream ifs;
  ifs.open("/home/rer/mybin/bin/config.json");
  if (!ifs.is_open()) {
    std::cout << "open file /home/rer/mybin/bin/config.json error!"
              << std::endl;
    return -1;
  }
  Json::Reader reader;
  Json::Value root;
  if (!reader.parse(ifs, root, false)) {
    std::cout << "parse json file error!" << std::endl;
    return -1;
  }
  std::string mqtt_url = root["mqtt_url"].asString();
  std::string mqtt_username = root["mqtt_username"].asString();
  std::string mqtt_passwd = root["mqtt_passwd"].asString();
  s_CLIENT_ID = cpuid;
  s_CLIENT_ID = "/" + s_CLIENT_ID;
  s_CLIENT_ID = "/dongguan-sai-001";
  s_RTMP_URI = root["rtmp_url"].asString();
  s_HTTP_URI = root["http_url"].asString();
  i_Mode = root["MODE"].asInt();
  std::cout << "s_CLIENT_ID: " << s_CLIENT_ID << ", s_RTMP_URI: " << s_RTMP_URI
            << std::endl;
  s_TOPIC_MISSION = s_CLIENT_ID + "/sys/mission";
  s_TOPIC_GENERAL = s_CLIENT_ID + "/sys/general";
  s_TOPIC_RC = s_CLIENT_ID + "/sys/rc";
  s_TOPIC_CAMERA = s_CLIENT_ID + "/sys/camera";
  s_TOPIC_V3 = s_CLIENT_ID + "/v3/sys/general";
  s_TOPIC_FAN = s_CLIENT_ID + "/sys/fanBase";
  std::string s_TOPIC_CB = s_CLIENT_ID + "/callback";

  // init the flight control instance and setup subscription.
  // TBD：make sure if subscription should be setup each loop.
  // DjiUser_FlightControllerCommandFlyingTask vs DjiTest_FlightControlRunSample
  modeFly = "INIT";
  constexpr bool kSubcribeTopicInThreadTask = true;
  if (!kSubcribeTopicInThreadTask) {
    T_DjiReturnCode returnCode = DjiTest_FlightControlInitAndSubscribeTopics();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      USER_LOG_ERROR("Init flight Control sample failed,error code:0x%08llX",
                     returnCode);
      return -1;
    }
    kSubscriptionStart = true;
  }

  modeFly = "STANDBY";
  // Todo{zengxw} start dji liveview thread task.
  bool_RTMPFlag = root["RTMPFlag"].asBool();
  T_DjiReturnCode returnCode;
  T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
  if (!osalHandler) {
    USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
    return -1;
  }
  returnCode =
      osalHandler->TaskCreate("MyRtmp_Task", MyRtmp_Task,
                              RPI_RTMP_TASK_STACK_SIZE, NULL, &s_rpiRtmpThread);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("MyRtmp_Task failed, errno = 0x%08llX", returnCode);
    return -1;
  }

  // setup MQTTT
  auto createOpts = mqtt::create_options_builder()
                        .send_while_disconnected(true, true)
                        .kMAX_BUFFERED_MESSAGES(kMAX_BUFFERED_MESSAGES)
                        .delete_oldest_messages()
                        .finalize();
  mqtt::async_client client(mqtt_url, s_CLIENT_ID, createOpts);
  cli = &client;
  cli->set_connected_handler([&client](const string&) {
    std::cout << "*** MQTT Connected ***" << std::endl;
  });
  cli->set_connection_lost_handler([&client](const string&) {
    std::cout << "*** MQTT Connection Lost ***" << std::endl;
  });
  auto willMsg =
      mqtt::message("/events", "Time publisher disconnected", 1, true);
  auto connOpts = mqtt::connect_options_builder()
                      .user_name("dkyuser")
                      .password("Ycjc@dky1409")
                      .clean_session()
                      .will(willMsg)
                      .automatic_reconnect(seconds(1), seconds(10))
                      .finalize();
  auto topic_callback = mqtt::topic(*cli, s_TOPIC_CB, kQOS);
  topic_cb = &topic_callback;

  // start more thread tasks
  try {
    // start a thread to init flight ctrl and subscribe the topic list
    if (kSubcribeTopicInThreadTask) {
      returnCode = osalHandler->TaskCreate(
          "flight_init_task", DjiTest_FlightControlInitAndSubscribeTopics,
          DJI_TEST_FLIGHT_INIT_TASK_STACK_SIZE, NULL, &s_flightInitTaskHandle);
      if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("flight_init_task task failed, errno = 0x%08llX",
                       returnCode);
        return -1;
      }
      kSubscriptionStart = true;
    }

    osalHandler->TaskSleepMs(1000);
    cli->start_consuming();
    std::cout << s_TOPIC_MISSION << std::endl;
    auto tok = cli->connect(connOpts);
    auto rsp = tok->get_connect_response();
    if (!rsp.is_session_present()) {
      cli->subscribe(s_TOPIC_MISSION, kQOS)->wait();
      cli->subscribe(s_TOPIC_GENERAL, kQOS)->wait();
      cli->subscribe(s_TOPIC_RC, kQOS)->wait();
      cli->subscribe(s_TOPIC_CAMERA, kQOS)->wait();
      cli->subscribe(s_TOPIC_V3, kQOS)->wait();
      cli->subscribe(s_TOPIC_FAN, kQOS)->wait();
    }

    // start a thread to forwarding pub the topics via MQTT
    returnCode =
        osalHandler->TaskCreate("status_forward_task", ForwardStatusMqtt_Task,
                                DJI_TEST_FLIGHT_STATUS_FORWARD_TASK_STACK_SIZE,
                                NULL, &s_flightStatusForwardTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      USER_LOG_ERROR("status_forward_task task failed, errno = 0x%08llX",
                     returnCode);
      return -1;
    }

    // start a thread to get user command via MQTT
    returnCode = osalHandler->TaskCreate(
        "consume_user_command_task", ConsumeUserCommand_Task,
        DJI_TEST_CONSUME_USER_COMMAND_TASK_STACK_SIZE, NULL,
        &s_consumeUserCommandTaskHandle);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      USER_LOG_ERROR("consume_user_command_task task failed, errno = 0x%08llX",
                     returnCode);
      return -1;
    }

  } catch (const mqtt::exception& exc) {
    std::cerr << exc.what() << '\n';
    return -1;
  }

  // keep running
  while (1) {
    sleep(5);
  }
  return 0;
}