#include "forward_status_mqtt.h"

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "subscribe_helper.h"

#include <json/json.h>
#include "mqtt/async_client.h"
#include "mqtt/topic.h"

namespace {
#define EnableMQTT
}

constexpr double DEG2RAD = 0.01745329252;

// The QoS for sending data
constexpr int kNum_QOS = 1;
// Forwarding frequence
constexpr int waitTimeMs = 1000;
Json::Value jv_waypoint;

extern std::string modeFly;
extern int i_CAMERA_MODE;
extern bool kSubscriptionStart;
extern std::string s_CLIENT_ID;
extern bool rtkAvailable;
extern mqtt::async_client* cli;
// --------------------------------------------------------------------------
void* ForwardStatusMqtt_Task(void* arg) {
  T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
  if (!osalHandler) {
    USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
    return NULL;
  }

  // All the subscribe data
  T_DjiFcSubscriptionAltitudeFused altitudeFused;    // 0  无人机高度
  T_DjiFcSubscriptionHeightFusion relativeAltitude;  // 1  无人机相对高度
  // 2  初始机头方向
  // 3  下行信号强度
  // 4  上行信号强度
  T_DjiFcSubscriptionFlightStatus flightStatus;  // 5  飞行时间
  // 6  返航高度 homeAltitude
  T_DjiFcSubscriptionVelocity velocity;  // 7  速度
  // 8  水平距离
  // 9  机头方向 yaw
  T_DjiFcSubscriptionPositionFused positionFused;  // 10 无人机纬度
  // 11 无人机经度
  T_DjiFcSubscriptionQuaternion quaternion;
  // 12 俯仰 eulerAngle.x: pitch
  // 13 横滚 eulerAngle.y: roll
  // 14 偏航 eulerAngle.z: yaw
  // 15 垂直速度
  // 16 图传信号
  T_DjiFcSubscriptionDisplaymode displayMode;  // 17 飞行模式
  // 18 返航状态
  T_DjiFcSubscriptionAltitudeOfHomePoint
      homePointAltitude;                           // 19 起飞/返航点海拔
  T_DjiFcSubscriptionHomePointInfo homePointInfo;  // 20 起飞/返航点纬度
  // 21 起飞/返航点经度
  T_DjiFcSubscriptionCompass compass;  // 22 磁罗盘是否异常
  // 23 卫星数
  T_DjiFcSubscriptionMotorStartError areMotorsOn;  // 24 电机是否启动
  // T_DjiFcSubscriptionAvoidData avoidData; // TODO{zengxw} is this para need?
  T_DjiFcSubscriptionFlightAnomaly flyAnomaly;  // 25 异常返回值
  T_DjiFcSubscriptionHeightFusion
      ultrasonicHeight;  // 26 超声波高度返回 // TODO{zengxw} same to
                         // relativeAltitude?
  // 27 是否正在推流
  // 28 推流信息

  T_DjiFcSubscriptionRC rc;
  T_DjiFcSubscriptionRtkPosition rtk;
  T_DjiFcSubscriptionRtkPositionInfo rtk_posi_info;
  T_DjiFcSubscriptionRtkVelocity rtk_velocity;
  T_DjiFcSubscriptionRtkYaw rtk_yaw;
  T_DjiFcSubscriptionRtkYawInfo rtk_yaw_info;
  T_DjiFcSubscriptionRTKConnectStatus rtk_connect_status;

  T_DjiFcSubscriptionGimbalAngles gimbalAngles;
  // T_DjiFcSubscriptionThreeGimbalData threeGimbaData; // TODO{zengxw} is this
  // param need?

  double initYaw = 361.0;
  double mag = 0.0;
  bool compassError = false;
  Vector3f eulerAngle;

  // Battery data
  int countSecond = 0;
  T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo1;
  T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo2;

  int elapsedTimeInSecond = 0;

  try {
    auto top_base = mqtt::topic(*cli, s_CLIENT_ID + "/status/base",
    kNum_QOS);
    auto top_aircraft =
        mqtt::topic(*cli, s_CLIENT_ID + "/status/aircraft", kNum_QOS);
    auto top_rtk = mqtt::topic(*cli, s_CLIENT_ID + "/status/rtk", kNum_QOS);
    auto top_battery =
        mqtt::topic(*cli, s_CLIENT_ID + "/status/aircraft_battery",
        kNum_QOS);
    auto top_gimbal = mqtt::topic(*cli, s_CLIENT_ID + "/status/gimbal",
    kNum_QOS);
    auto top_waypoint =
        mqtt::topic(*cli, s_CLIENT_ID + "/status/waypoint", kNum_QOS);
    auto top_camera =
        mqtt::topic(*cli, s_CLIENT_ID + "/v3/status/camera", kNum_QOS);
    auto top_avoid =
        mqtt::topic(*cli, s_CLIENT_ID + "/v3/status/avoidSystem", kNum_QOS);

    while (true) {
      if (!kSubscriptionStart) {
        continue;
      }

      altitudeFused = DjiUser_FlightControlGetValueOfAltitudeFused();
      relativeAltitude =
          DjiUser_FlightControlGetValueOfHeightFused();  // TODO{zengxw} make
                                                         // sure what is this
                                                         // param's purpose.
      flightStatus = DjiUser_FlightControlGetValueOfFlightStatus();
      velocity = DjiUser_FlightControlGetValueOfVelocity();
      positionFused = DjiUser_FlightControlGetValueOfPositionFused();
      quaternion = DjiUser_FlightControlGetValueOfRawQuat();
      displayMode = DjiUser_FlightControlGetValueOfDisplaymode();
      homePointAltitude = DjiUser_FlightControlGetValueOfHomePointAltitude();
      homePointInfo = DjiUser_FlightControlGetValueOfHomePointInfo();
      compass = DjiUser_FlightControlGetValueOfCompass();
      areMotorsOn = DjiUser_FlightControlGetValueOfMotorStatusError();
      flyAnomaly = DjiUser_FlightControlGetValueOfFlightAnomaly();
      ultrasonicHeight =
          DjiUser_FlightControlGetValueOfHeightFused();  // TODO{zengxw} the
                                                         // same to
                                                         // relativeAltitude??

      rc = DjiUser_FlightControlGetValueOfRC();

      rtk = DjiUser_FlightControlGetValueOfRtkPosition();
      rtk_posi_info = DjiUser_FlightControlGetValueOfRtkPositionInfo();
      rtk_velocity = DjiUser_FlightControlGetValueOfRtkVelocity();
      rtk_yaw = DjiUser_FlightControlGetValueOfRtkYaw();
      rtk_yaw_info = DjiUser_FlightControlGetValueOfRtkYawInfo();
      rtk_connect_status = DjiUser_FlightControlGetValueOfRTKConnectStatus();

      gimbalAngles = DjiUser_FlightControlGetValueOfGimbalAngles();

      if (flightStatus == E_DjiFcSubscriptionFlightStatus::
                              DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        elapsedTimeInSecond += (waitTimeMs / 1000);
      }

      mag = std::sqrt(compass.x * compass.x + compass.y * compass.y +
                      compass.z * compass.z);
      compassError = (1000 < mag) && (mag < 2000);

      double horizontalSpeed = std::sqrt(velocity.data.x * velocity.data.x +
                                         +velocity.data.y * velocity.data.y);
      // x:pitch y:roll z:yaw
      eulerAngle = quaternionToEulerAngle(quaternion);
      // init yaw
      if (initYaw > 360) {
        initYaw = eulerAngle.z;
      }

      // 5.3.2 [/status/aircraft]
      Json::Value jv_base;
      jv_base["aircraftStateConstant"] = "READY_TO_GO";  // 无人机状态
      jv_base["isAircraftConnected"] = true;          // 无人机是否连接
      jv_base["nestStateConstant"] = modeFly;         // 机库状态
      jv_base["isRemoteControllerConnected"] = true;  // 遥控器是否连接
      jv_base["nestLocationAltitude"] = 0.0;          // 经纬度
      jv_base["nestLocationLatitude"] = 0.0;
      jv_base["nestLocationLongitude"] = 0.0;
      jv_base["nestControllerState"] = 1;  // 机库控制端是否在线

      // 5.3.2 [/status/aircraft]
      Json::Value jv_aircraft;
      jv_aircraft["aircraftAltitude"] = std::to_string(altitudeFused);
      jv_aircraft["relativeAltitude"] = std::to_string(relativeAltitude);
      jv_aircraft["aircraftBaseHeadDirection"] = std::to_string(initYaw);
      // TODO jv_aircraft["aircraftDownLinkSignal"] = std::to_string(0);
      // TODO jv_aircraft["aircraftUpLinkSignal"] = std::to_string(0);
      jv_aircraft["aircraftFlyInSecond"] = std::to_string(elapsedTimeInSecond);
      jv_aircraft["aircraftGoHomeHeight"] = std::to_string(homePointAltitude);
      jv_aircraft["aircraftHSpeed"] = std::to_string(horizontalSpeed);
      jv_aircraft["aircraftHeadDirection"] = std::to_string(eulerAngle.z);
      jv_aircraft["aircraftLocationLatitude"] =
          std::to_string(positionFused.latitude / DEG2RAD);
      jv_aircraft["aircraftLocationLongitude"] =
          std::to_string(positionFused.longitude / DEG2RAD);
      jv_aircraft["aircraftPitch"] = std::to_string(eulerAngle.x);
      jv_aircraft["aircraftRoll"] = std::to_string(eulerAngle.y);
      jv_aircraft["aircraftYaw"] = std::to_string(eulerAngle.z);
      jv_aircraft["aircraftVSpeed"] = std::to_string(velocity.data.z);

      std::ofstream ofs;
      ofs.open("log.txt", std::ios::app);  // 采取追加的方式写入文件
      ofs << std::to_string(positionFused.latitude / DEG2RAD) << ",";
      ofs << std::to_string(positionFused.longitude / DEG2RAD) << ",";
      ofs << std::to_string(altitudeFused) << ",";
      ofs << std::to_string(eulerAngle.x) << ",";
      ofs << std::to_string(eulerAngle.y) << ",";
      ofs << std::to_string(eulerAngle.z) << ",";
      ofs << std::to_string(velocity.data.z) << std::endl;
      ofs.close();

      // TODO jv_aircraft["aircraftVideoSignal"] = std::to_string(0);
      jv_aircraft["flightMode"] = displayModeToString(displayMode);
      // TODO jv_aircraft["goHomeState"] = std::to_string(0);
      jv_aircraft["homeAltitude"] = std::to_string(homePointAltitude);
      jv_aircraft["isCompassError"] = (compassError ? "false" : "true");
      jv_aircraft["satelliteCount"] =
          std::to_string(positionFused.visibleSatelliteNumber);
      jv_aircraft["areMotorsOn"] = (areMotorsOn ? "false" : "true");
      jv_aircraft["diagnostics"] = getFlightAnomaly(flyAnomaly);
      jv_aircraft["aircraftUltrasonicHeightInMeters"] = std::to_string(ultrasonicHeight);
      // TODO jv_aircraft["isLiveStreaming"] = std::to_string(0);
      // TODO jv_aircraft["liveStreamInfo"] = std::to_string(0);
      if (homePointInfo.longitude > -3.2 && homePointInfo.longitude < 3.2) {
        jv_aircraft["homeLocationLatitude"] =
            std::to_string(homePointInfo.latitude / DEG2RAD);
        jv_aircraft["homeLocationLongitude"] =
            std::to_string(homePointInfo.longitude / DEG2RAD);
        jv_aircraft["aircraftDistToHomePoint"] = std::to_string(
            getDistanceByRad(positionFused.latitude, positionFused.longitude,
                             homePointInfo.latitude, homePointInfo.longitude));
      }

      // 5.3.3 [/status/rtk]
      Json::Value jv_rtk;
      if (rtkAvailable) {
        jv_rtk["aircraftRtkAltitude"] = std::to_string(rtk.hfsl);
        jv_rtk["aircraftRtkLatitude"] = std::to_string(rtk.latitude);
        jv_rtk["aircraftRtkLongitude"] = std::to_string(rtk.longitude);
        jv_rtk["aircraftRtkYaw"] = std::to_string(rtk_yaw);
        // TODO  jv_rtk["aircraftSatelliteCount"] = std::to_string(0);
        // TODO  jv_rtk["distanceToHomePoint"] = std::to_string(0);
        // TODO  jv_rtk["homePointLatitude"] = std::to_string(0);
        // TODO  jv_rtk["homePointLongitude"] = std::to_string(0);
        // TODO  jv_rtk["isRtkEnable"] = std::to_string(0);
        // TODO  jv_rtk["isRtkReady"] = std::to_string(0);
        jv_rtk["connectionInfo"] =
            (rtk_connect_status.rtkConnected ? "success" : "failed");
        // TODO  jv_rtk["rtkType"] = std::to_string(0);
        // TODO  jv_rtk["baseInfo"] = std::to_string(0);
        // TODO  jv_rtk["mobileAltitude"] = std::to_string(0);
        jv_rtk["positioningSolution"] =
            getPositionSolutionString(rtk_posi_info);
        // TODO  jv_rtk["takeOffAltitude"] = std::to_string(0);
        // TODO  jv_rtk["networkChannelMsg"] = std::to_string(0);
      }

      // 5.3.4 [/status/aircraft_battery]
      Json::Value jv_battery;
      if (countSecond++ % 2 == 0) {
        singleBatteryInfo1 = DjiUser_FlightControlGetValueOfBattery1();
        singleBatteryInfo2 = DjiUser_FlightControlGetValueOfBattery2();
        // TODO{zengxw} is singleBatteryInfo2 not need?
        jv_battery["aircraftBatteryChargeInPercent"] =
            singleBatteryInfo1.batteryCapacityPercent;
        jv_battery["aircraftBatteryCurrentConsumption"] =
            singleBatteryInfo1.cellCount;
        jv_battery["aircraftBatteryCurrentTemperature"] =
            singleBatteryInfo1.batteryTemperature / 10;
        jv_battery["aircraftBatteryCurrentVoltage"] =
            singleBatteryInfo1.currentVoltage;
        jv_battery["isBatteryConnected"] =
            (singleBatteryInfo1.batteryState.isBatteryEmbed ? "false" : "true");
        jv_battery["isCellDamaged"] =
            (singleBatteryInfo1.batteryState.cellBreak ? "true" : "false");
        jv_battery["isLowCellVoltageDetected"] = true;
        jv_battery["batterySerailNo"] = "SN001";   
      }

      // 5.3.6 [/status/gimbal]
      Json::Value jv_gimbal;
      jv_gimbal["gimbalPitch"] = std::to_string(gimbalAngles.x);
      jv_gimbal["gimbalYaw"] = std::to_string(gimbalAngles.z);
      jv_gimbal["gimbalRoll"] = std::to_string(gimbalAngles.y);

      // 5.3.7 [/status/waypoint]

      // 5.3.13 [/v3/status/camera]
      Json::Value jv_camera;
      // TODO{zengxw} make sure the camera mount position,
      // base_info.mountPosition is E-port.
      E_DjiCameraType camera_type = DjiUser_FlightControlGetValueOfCameraType(
          E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
      jv_camera["cameraType"] = cameraTypeEnumToString(camera_type);
      // jv_camera["cameraMode"] = std::to_string(i_CAMERA_MODE);

      // 5.3.14 [/v3/status/avoidSystem]
      Json::Value jv_avoid;
      E_DjiFlightControllerObstacleAvoidanceEnableStatus
          horizontal_avoid_status =
              DjiUser_FlightControlGetValueOfCollisionAvoidance(
                  COLLISION_AVOIDANCE_DIRECTION::HORIZONTAL,
                  COLLISION_AVOIDANCE_TYPE::VISUAL);
      E_DjiFlightControllerObstacleAvoidanceEnableStatus upwards_avoid_status =
          DjiUser_FlightControlGetValueOfCollisionAvoidance(
              COLLISION_AVOIDANCE_DIRECTION::UP,
              COLLISION_AVOIDANCE_TYPE::VISUAL);
      E_DjiFlightControllerObstacleAvoidanceEnableStatus
          downwards_avoid_status =
              DjiUser_FlightControlGetValueOfCollisionAvoidance(
                  COLLISION_AVOIDANCE_DIRECTION::DOWN,
                  COLLISION_AVOIDANCE_TYPE::VISUAL);
      int avoid_status = 1;
      if (horizontal_avoid_status ==
              E_DjiFlightControllerObstacleAvoidanceEnableStatus::
                  DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE ||
          upwards_avoid_status ==
              E_DjiFlightControllerObstacleAvoidanceEnableStatus::
                  DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE ||
          downwards_avoid_status ==
              E_DjiFlightControllerObstacleAvoidanceEnableStatus::
                  DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE) {
        avoid_status = 0;
      }
      T_DjiAircraftInfoBaseInfo base_info =
          DjiUser_FlightControlGetValueOfAircraftBaseInfo();
      jv_avoid["aircraftType"] = base_info.aircraftType;
      jv_avoid["avoidSwitch"] = std::to_string(avoid_status);

#ifdef EnableMQTT
      top_base.publish(jv_base.toStyledString());
      top_aircraft.publish(jv_aircraft.toStyledString());
      if (rtkAvailable) top_rtk.publish(jv_rtk.toStyledString());
      if ((countSecond - 1) % 2 == 0) top_battery.publish(jv_battery.toStyledString());
      top_gimbal.publish(jv_gimbal.toStyledString());
      top_waypoint.publish(jv_waypoint.toStyledString());
      top_camera.publish(jv_camera.toStyledString());
      top_avoid.publish(jv_avoid.toStyledString());
#endif
      osalHandler->TaskSleepMs(waitTimeMs);
    }
  } catch (std::exception& e) {
    USER_LOG_ERROR("Subscrption error module!\n");
    return 0;
  }

  return 0;
}
