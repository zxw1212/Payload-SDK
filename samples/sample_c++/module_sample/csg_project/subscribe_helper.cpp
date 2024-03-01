
#include "subscribe_helper.h"

#include <stdio.h>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "dji_camera_manager.h"
#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "dji_platform.h"

void* DjiTest_FlightControlInitAndSubscribeTopics(void* arg) {
  T_DjiReturnCode returnCode;

  T_DjiFlightControllerRidInfo ridInfo = {0};
  ridInfo.latitude = 22.542812;
  ridInfo.longitude = 113.958902;
  ridInfo.altitude = 10;

  returnCode = DjiFlightController_Init(ridInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX",
                   returnCode);
    return NULL;
  }

  returnCode = DjiFcSubscription_Init();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX",
                   returnCode);
    return NULL;
  }

  std::vector<E_DjiFcSubscriptionTopic> topic_list{
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,  // like OSDK
                                                     // TOPIC_GPS_FUSED
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY,

      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RC,

      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,

      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_THREE_GIMBAL_DATA  // For M300 RTK and M350
                                                       // RTK.
  };
  // string copy of topi list
  std::vector<std::string> topic_list_string{
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_"
      "FUSED",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_"
      "FUSION",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_"
      "FLIGHT",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_POSITION_"
      "FUSED",  // like OSDK TOPIC_GPS_FUSED
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_"
      "DISPLAYMODE",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_"
      "HOMEPOINT",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_"
      "INFO",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_COMPASS",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_"
      "MOTOR_START_ERROR",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_"
      "ANOMALY",

      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RC",

      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_"
      "POSITION",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_"
      "INFO",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_"
      "POSITION_INFO",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_"
      "VELOCITY",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_"
      "STATUS",

      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_"
      "ANGLES",
      "E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_THREE_"
      "GIMBAL_DATA"  // For M300 RTK and M350 RTK.
  };
  if (topic_list.size() != topic_list_string.size()) {
    std::cerr << "Error: Vector lengths are not equal." << std::endl;
    return NULL;
  }

  // subscribe topic list
  int topic_index = 0;
  for (const auto& topic : topic_list) {
    returnCode = DjiFcSubscription_SubscribeTopic(
        topic, E_DjiDataSubscriptionTopicFreq::DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
        NULL);
    if (returnCode != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      std::cerr << "Subscribe topic " << topic_list_string[topic_index]
                << std::endl;
      USER_LOG_ERROR("failed, error code:0x%08llX", returnCode);
      return NULL;
    }
    topic_index++;
  }
}

T_DjiFcSubscriptionAltitudeFused
DjiUser_FlightControlGetValueOfAltitudeFused() {
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timeStamp = {0};
  T_DjiFcSubscriptionAltitudeFused altitudeFused;

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
      (uint8_t*)&altitudeFused, sizeof(T_DjiFcSubscriptionAltitudeFused),
      &timeStamp);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic altitudeFused error, error code: 0x%08X",
                   djiStat);
  }
  return altitudeFused;
}

T_DjiFcSubscriptionHeightFusion DjiUser_FlightControlGetValueOfHeightFused() {
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timeStamp = {0};
  T_DjiFcSubscriptionHeightFusion heightFused;

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
      (uint8_t*)&heightFused, sizeof(T_DjiFcSubscriptionHeightFusion),
      &timeStamp);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic heightFused error, error code: 0x%08X",
                   djiStat);
  }
  return heightFused;
}

T_DjiFcSubscriptionFlightStatus DjiUser_FlightControlGetValueOfFlightStatus() {
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timeStamp = {0};
  T_DjiFcSubscriptionFlightStatus flightStatus;

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
      (uint8_t*)&flightStatus, sizeof(T_DjiFcSubscriptionFlightStatus),
      &timeStamp);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic flightStatus error, error code: 0x%08X",
                   djiStat);
  }
  return flightStatus;
}

T_DjiFcSubscriptionVelocity DjiUser_FlightControlGetValueOfVelocity() {
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timeStamp = {0};
  T_DjiFcSubscriptionVelocity velocity;

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
      (uint8_t*)&velocity, sizeof(T_DjiFcSubscriptionVelocity), &timeStamp);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic velocity error, error code: 0x%08X",
                   djiStat);
  }
  return velocity;
}

T_DjiFcSubscriptionPositionFused
DjiUser_FlightControlGetValueOfPositionFused() {
  T_DjiReturnCode djiStat;
  T_DjiDataTimestamp timeStamp = {0};
  T_DjiFcSubscriptionPositionFused positionFused;

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
      (uint8_t*)&positionFused, sizeof(T_DjiFcSubscriptionPositionFused),
      &timeStamp);
  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic positionFused error, error code: 0x%08X",
                   djiStat);
  }
  return positionFused;
}

T_DjiFcSubscriptionQuaternion DjiUser_FlightControlGetValueOfRawQuat() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionQuaternion quaternion = {0};
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
      (uint8_t*)&quaternion, sizeof(T_DjiFcSubscriptionQuaternion), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X",
                   djiStat);
  }

  return quaternion;
}

T_DjiFcSubscriptionDisplaymode DjiUser_FlightControlGetValueOfDisplaymode() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionDisplaymode displayMode;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
      (uint8_t*)&displayMode, sizeof(T_DjiFcSubscriptionDisplaymode),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic displayMode error, error code: 0x%08X",
                   djiStat);
  }

  return displayMode;
}

T_DjiFcSubscriptionAltitudeOfHomePoint
DjiUser_FlightControlGetValueOfHomePointAltitude() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
      (uint8_t*)&homePointAltitude,
      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get value of topic homePointAltitude error, error code: 0x%08X",
        djiStat);
  }

  return homePointAltitude;
}

T_DjiFcSubscriptionHomePointInfo
DjiUser_FlightControlGetValueOfHomePointInfo() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionHomePointInfo homePointInfo;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
      (uint8_t*)&homePointInfo, sizeof(T_DjiFcSubscriptionHomePointInfo),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic homePointInfo error, error code: 0x%08X",
                   djiStat);
  }

  return homePointInfo;
}

T_DjiFcSubscriptionCompass DjiUser_FlightControlGetValueOfCompass() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionCompass compass;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_COMPASS,
      (uint8_t*)&compass, sizeof(T_DjiFcSubscriptionCompass), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic compass error, error code: 0x%08X",
                   djiStat);
  }

  return compass;
}

T_DjiFcSubscriptionMotorStartError
DjiUser_FlightControlGetValueOfMotorStatusError() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionMotorStartError areMotorsOn;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_STATUS_MOTOR_START_ERROR,
      (uint8_t*)&areMotorsOn, sizeof(T_DjiFcSubscriptionMotorStartError),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get value of topic MotorStartError error, error code: 0x%08X",
        djiStat);
  }

  return areMotorsOn;
}

T_DjiFcSubscriptionFlightAnomaly
DjiUser_FlightControlGetValueOfFlightAnomaly() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionFlightAnomaly flyAnomaly;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_FLIGHT_ANOMALY,
      (uint8_t*)&flyAnomaly, sizeof(T_DjiFcSubscriptionFlightAnomaly),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic FlightAnomaly error, error code: 0x%08X",
                   djiStat);
  }

  return flyAnomaly;
}

T_DjiFcSubscriptionRC DjiUser_FlightControlGetValueOfRC() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRC rc;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RC, (uint8_t*)&rc,
      sizeof(T_DjiFcSubscriptionRC), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic RC error, error code: 0x%08X", djiStat);
  }

  return rc;
}

T_DjiFcSubscriptionRtkPosition DjiUser_FlightControlGetValueOfRtkPosition() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRtkPosition rtk;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
      (uint8_t*)&rtk, sizeof(T_DjiFcSubscriptionRtkPosition), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic RtkPosition error, error code: 0x%08X",
                   djiStat);
  }

  return rtk;
}

T_DjiFcSubscriptionRtkPositionInfo
DjiUser_FlightControlGetValueOfRtkPositionInfo() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRtkPositionInfo rtk_info;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
      (uint8_t*)&rtk_info, sizeof(T_DjiFcSubscriptionRtkPositionInfo),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get value of topic RtkPositionInfo error, error code: 0x%08X",
        djiStat);
  }

  return rtk_info;
}

T_DjiFcSubscriptionRtkVelocity DjiUser_FlightControlGetValueOfRtkVelocity() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRtkVelocity rtk_velocity;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
      (uint8_t*)&rtk_velocity, sizeof(T_DjiFcSubscriptionRtkVelocity),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic RtkVelocity error, error code: 0x%08X",
                   djiStat);
  }

  return rtk_velocity;
}

T_DjiFcSubscriptionRtkYaw DjiUser_FlightControlGetValueOfRtkYaw() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRtkYaw rtk_yaw;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
      (uint8_t*)&rtk_yaw, sizeof(T_DjiFcSubscriptionRtkYaw), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic RtkYaw error, error code: 0x%08X",
                   djiStat);
  }

  return rtk_yaw;
}

T_DjiFcSubscriptionRtkYawInfo DjiUser_FlightControlGetValueOfRtkYawInfo() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRtkYawInfo rtk_yaw_info;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW_INFO,
      (uint8_t*)&rtk_yaw_info, sizeof(T_DjiFcSubscriptionRtkYawInfo),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic RtkYawInfo error, error code: 0x%08X",
                   djiStat);
  }

  return rtk_yaw_info;
}

T_DjiFcSubscriptionRTKConnectStatus
DjiUser_FlightControlGetValueOfRTKConnectStatus() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionRTKConnectStatus rtk_connect_status;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,
      (uint8_t*)&rtk_connect_status,
      sizeof(T_DjiFcSubscriptionRTKConnectStatus), &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get value of topic RTKConnectStatus error, error code: 0x%08X",
        djiStat);
  }

  return rtk_connect_status;
}

T_DjiFcSubscriptionGimbalAngles DjiUser_FlightControlGetValueOfGimbalAngles() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionGimbalAngles gimbalAngles;
  T_DjiDataTimestamp timeStamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
      (uint8_t*)&gimbalAngles, sizeof(T_DjiFcSubscriptionGimbalAngles),
      &timeStamp);

  if (djiStat != DjiErrorCode::DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic GimbalAngles error, error code: 0x%08X",
                   djiStat);
  }

  return gimbalAngles;
}

T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;
  T_DjiDataTimestamp timestamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX1,
      (uint8_t*)&singleBatteryInfo,
      sizeof(T_DjiFcSubscriptionSingleBatteryInfo), &timestamp);

  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic battery1 error, error code: 0x%08X",
                   djiStat);
  }

  return singleBatteryInfo;
}

T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2() {
  T_DjiReturnCode djiStat;
  T_DjiFcSubscriptionSingleBatteryInfo singleBatteryInfo;
  T_DjiDataTimestamp timestamp = {0};

  djiStat = DjiFcSubscription_GetLatestValueOfTopic(
      E_DjiFcSubscriptionTopic::
          DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_SINGLE_INFO_INDEX2,
      (uint8_t*)&singleBatteryInfo,
      sizeof(T_DjiFcSubscriptionSingleBatteryInfo), &timestamp);

  if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get value of topic battery2 error, error code: 0x%08X",
                   djiStat);
  }

  return singleBatteryInfo;
}

E_DjiCameraType DjiUser_FlightControlGetValueOfCameraType(
    E_DjiMountPosition mountPosition) {
  E_DjiCameraType cameraType;
  T_DjiReturnCode returnCode;

  returnCode = DjiCameraManager_GetCameraType(mountPosition, &cameraType);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR(
        "Get mounted position %d camera's type failed, error code: 0x%08X\r\n",
        mountPosition, returnCode);
  }
  return cameraType;
}

T_DjiAircraftInfoBaseInfo DjiUser_FlightControlGetValueOfAircraftBaseInfo() {
  T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
  T_DjiReturnCode returnCode;
  returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("get aircraft base info error");
  }
  return aircraftInfoBaseInfo;
}

E_DjiFlightControllerObstacleAvoidanceEnableStatus
DjiUser_FlightControlGetValueOfCollisionAvoidance(
    COLLISION_AVOIDANCE_DIRECTION direction, COLLISION_AVOIDANCE_TYPE type) {
  E_DjiFlightControllerObstacleAvoidanceEnableStatus
      collision_avoid_enable_status;
  T_DjiReturnCode returnCode;

  switch (direction) {
    case COLLISION_AVOIDANCE_DIRECTION::HORIZONTAL:
      switch (type) {
        case COLLISION_AVOIDANCE_TYPE::RADAR:
          returnCode =
              DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
                  &collision_avoid_enable_status);
          if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR(
                "Get horizontal radar obstacle avoidance failed, error code: "
                "0x%08X",
                returnCode);
          }
          return collision_avoid_enable_status;
        case COLLISION_AVOIDANCE_TYPE::VISUAL:
          returnCode =
              DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
                  &collision_avoid_enable_status);
          if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR(
                "Get horizontal visual obstacle avoidance failed, error code: "
                "0x%08X",
                returnCode);
          }
          return collision_avoid_enable_status;
      }
    case COLLISION_AVOIDANCE_DIRECTION::UP:
      switch (type) {
        case COLLISION_AVOIDANCE_TYPE::RADAR:
          returnCode =
              DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
                  &collision_avoid_enable_status);
          if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR(
                "Get upwards radar obstacle avoidance failed, error code: "
                "0x%08X",
                returnCode);
          }
          return collision_avoid_enable_status;
        case COLLISION_AVOIDANCE_TYPE::VISUAL:
          returnCode =
              DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
                  &collision_avoid_enable_status);
          if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR(
                "Get upwards visual obstacle avoidance failed, error code: "
                "0x%08X",
                returnCode);
          }
          return collision_avoid_enable_status;
      }
    case COLLISION_AVOIDANCE_DIRECTION::DOWN:
      switch (type) {
        case COLLISION_AVOIDANCE_TYPE::RADAR:
          USER_LOG_ERROR(
              "Ladar does not support collision avoidance in downwards "
              "direction!");
        case COLLISION_AVOIDANCE_TYPE::VISUAL:
          returnCode =
              DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
                  &collision_avoid_enable_status);
          if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR(
                "Get downwards visual obstacle avoidance failed, error code: "
                "0x%08X",
                returnCode);
          }
          return collision_avoid_enable_status;
      }
  }
}

T_DjiVector3f quaternionToEulerAngle(
    const T_DjiFcSubscriptionQuaternion& quat) {
  T_DjiVector3f eulerAngle;
  constexpr double DEG2RAD = 0.01745329252;
  double q2sqr = quat.q2 * quat.q2;
  double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
  double t1 = 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
  double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
  double t3 = 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
  double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
  t2 = (t2 > 1.0) ? 1.0 : t2;
  t2 = (t2 < -1.0) ? -1.0 : t2;
  eulerAngle.x = std::asin(t2) / DEG2RAD;       // in degree
  eulerAngle.y = std::atan2(t3, t4) / DEG2RAD;  // in degree
  eulerAngle.z = std::atan2(t1, t0) / DEG2RAD;  // in degree
  return eulerAngle;
}

std::string getFlightAnomaly(
    const T_DjiFcSubscriptionFlightAnomaly& flyAnomaly) {
  std::string ret = "";

  if (!(flyAnomaly.impactInAir || flyAnomaly.randomFly ||
        flyAnomaly.heightCtrlFail || flyAnomaly.rollPitchCtrlFail ||
        flyAnomaly.yawCtrlFail || flyAnomaly.aircraftIsFalling ||
        flyAnomaly.strongWindLevel1 || flyAnomaly.strongWindLevel2 ||
        flyAnomaly.compassInstallationError ||
        flyAnomaly.imuInstallationError || flyAnomaly.escTemperatureHigh ||
        flyAnomaly.atLeastOneEscDisconnected || flyAnomaly.gpsYawError)) {
    ret = "normal";
    return ret;
  } else {
    if (flyAnomaly.impactInAir) {
      ret += "impactInAir.";
    }
    if (flyAnomaly.randomFly) {
      ret += "randomFly.";
    }
    if (flyAnomaly.heightCtrlFail) {
      ret += "heightCtrlFail.";
    }
    if (flyAnomaly.rollPitchCtrlFail) {
      ret += "rollPitchCtrlFail.";
    }
    if (flyAnomaly.yawCtrlFail) {
      ret += "yawCtrlFail.";
    }
    if (flyAnomaly.aircraftIsFalling) {
      ret += "aircraftIsFalling.";
    }
    if (flyAnomaly.strongWindLevel1) {
      ret += "strongWindLevel1.";
    }
    if (flyAnomaly.strongWindLevel2) {
      ret += "strongWindLevel2.";
    }
    if (flyAnomaly.compassInstallationError) {
      ret += "compassInstallationError.";
    }
    if (flyAnomaly.imuInstallationError) {
      ret += "imuInstallationError.";
    }
    if (flyAnomaly.escTemperatureHigh) {
      ret += "escTemperatureHigh.";
    }
    if (flyAnomaly.atLeastOneEscDisconnected) {
      ret += "atLeastOneEscDisconnected.";
    }
    if (flyAnomaly.gpsYawError) {
      ret += "gpsYawError.";
    }
  }

  return ret;
}

// Calculate the Earth surface distance between two latitude and longitude
// coordinates
double getDistanceByRad(double radLat1, double radLng1, double radLat2,
                        double radLng2) {
  double a = radLat1 - radLat2;
  double b = radLng1 - radLng2;
  double s = 2 * std::asin(std::sqrt(pow(std::sin(a / 2), 2) +
                                     std::cos(radLat1) * std::cos(radLat2) *
                                         std::pow(sin(b / 2), 2)));
  constexpr double EARTH_RADIUS = 6378137.0;
  s = s * EARTH_RADIUS;
  s = std::round(s * 10000) / 10000;
  return s;
}

std::string getPositionSolutionString(int posInfo) {
  std::string posInfoStr;
  switch (posInfo) {
    case 0:
      posInfoStr = "NONE";
      break;
    case 1:
      posInfoStr = "FIXED_POINT";
      break;
    case 2:
      posInfoStr = "FIXED_HEIGHT/AUTO";
      break;
    case 16:
      posInfoStr = "SINGLE_POINT";
      break;
    case 32:
    case 33:
    case 34:
      posInfoStr = "FLOAT";
      break;
    case 48:
    case 49:
    case 50:
      posInfoStr = "INTEGER";
      break;
    default:
      posInfoStr = "UNKNOWN";
      break;
  }
  return posInfoStr;
}

std::string cameraTypeEnumToString(const E_DjiCameraType& camera_type) {
  std::unordered_map<E_DjiCameraType, std::string> cameraTypeMap = {
      {E_DjiCameraType::DJI_CAMERA_TYPE_UNKNOWN, "Unknown"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_Z30, "Zenmuse Z30"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_XT2, "Zenmuse XT2"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_PSDK, "Payload Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_XTS, "Zenmuse XTS"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_H20, "Zenmuse H20"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_H20T, "Zenmuse H20T"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_P1, "Zenmuse P1"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_L1, "Zenmuse L1"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_L2, "Zenmuse L2"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_H20N, "Zenmuse H20N"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M30, "M30 Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M30T, "M30T Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M3E, "M3E Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M3T, "M3T Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M3D, "M3D Camera"},
      {E_DjiCameraType::DJI_CAMERA_TYPE_M3TD, "M3TD Camera"},
  };

  return cameraTypeMap[camera_type];
}

std::string cameraWorkModeEnumToString(const E_DjiCameraManagerWorkMode& camera_work_mode) {
  std::unordered_map<E_DjiCameraManagerWorkMode, std::string> cameraWorkModeMap = {
      {DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO, "SHOOT_PHOTO"},
      {DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO, "RECORD_VIDEO"},
      {DJI_CAMERA_MANAGER_WORK_MODE_PLAYBACK, "PLAYBACK"},
      {DJI_CAMERA_MANAGER_WORK_MODE_MEDIA_DOWNLOAD, "MEDIA_DOWNLOAD"},
      {DJI_CAMERA_MANAGER_WORK_MODE_BROADCAST, "BROADCAST"},
      {DJI_CAMERA_MANAGER_WORK_MODE_WORK_MODE_UNKNOWN, "UNKNOWN"}
  };

  return cameraWorkModeMap[camera_work_mode];
}