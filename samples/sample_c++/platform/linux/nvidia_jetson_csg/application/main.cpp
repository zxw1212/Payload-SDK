#include <stdio.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "application.hpp"
#include "consume_command_mqtt.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "forward_status_mqtt.h"
#include "mqtt/async_client.h"
#include "mqtt/topic.h"
#include "subscribe_helper.h"
#include <json/json.h>

namespace {
#define EnableMQTT
}

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
constexpr int kQOS = 1;
constexpr int kMAX_BUFFERED_MESSAGES = 1200;

// some flag
int i_Mode = 0;
std::string modeFly;
int i_CAMERA_MODE = 0;
bool bool_RTMPFlag;
bool kSubscriptionStart = false;

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
#ifdef EnableMQTT
  // setup param of MQTT
  system("pwd");
  ifstream ifs;
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
  std::string SERVER_ADDRESS = root["SERVER_ADDRESS"].asString();
  s_CLIENT_ID = root["CLIENT_ID"].asString();
  s_RTMP_URI = root["RTMP_URI"].asString();
  s_HTTP_URI = root["HTTP_URI"].asString();

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

  // setup MQTTT
  auto createOpts = mqtt::create_options_builder()
                        .send_while_disconnected(true, true)
                        .max_buffered_messages(kMAX_BUFFERED_MESSAGES)
                        .delete_oldest_messages()
                        .finalize();
  mqtt::async_client client(SERVER_ADDRESS, s_CLIENT_ID, createOpts);
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
                      .automatic_reconnect(std::chrono::seconds(1),
                                           std::chrono::seconds(10))
                      .finalize();
  auto topic_callback = mqtt::topic(*cli, s_TOPIC_CB, kQOS);
  topic_cb = &topic_callback;

  // subscribe mqtt topic
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

  // // add to timer to check connection every 10 seconds
  // mqtt::timer ping_timer(*cli);
  // ping_timer.set_interval(seconds(10));

  // ping_timer.set_callback([&client]() {
  //   if (!cli->is_connected()) {
  //     cout << "*** Connection Lost ***" << endl;
  //     try {
  //       cli->reconnect();
  //     } catch (const mqtt::exception& exc) {
  //       cerr << "Reconnect failed: " << exc.what() << endl;
  //     }
  //   }
  // });
  // ping_timer.start();

#endif

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

  // modeFly = "STANDBY";
  // // Todo{zengxw} start dji liveview thread task.
  // bool_RTMPFlag = root["RTMPFlag"].asBool();
  // returnCode =
  //     osalHandler->TaskCreate("MyRtmp_Task", MyRtmp_Task,
  //                             RPI_RTMP_TASK_STACK_SIZE, NULL, &s_rpiRtmpThread);
  // if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
  //   USER_LOG_ERROR("MyRtmp_Task failed, errno = 0x%08llX", returnCode);
  //   goto Deinit;
  // }

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
