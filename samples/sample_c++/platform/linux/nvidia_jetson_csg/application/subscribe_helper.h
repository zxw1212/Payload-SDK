
#ifndef _SUBSCRIBE_HELPER_H_
#define _SUBSCRIBE_HELPER_H_

#include <string>

#include "dji_aircraft_info.h"
#include "dji_camera_manager.h"
#include "dji_fc_subscription.h"
#include "dji_flight_controller.h"
#include "dji_typedef.h"

void* DjiUser_ModuleInitAndSubscribeTopics(void* arg);
T_DjiReturnCode DjiUser_ModuleDeInit(void);

T_DjiReturnCode DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);

// some helper function to get subscription value

// raletive to sea level
T_DjiFcSubscriptionAltitudeFused DjiUser_FlightControlGetValueOfAltitudeFused();
// raletive to land
T_DjiFcSubscriptionHeightFusion DjiUser_FlightControlGetValueOfHeightFused();
T_DjiFcSubscriptionFlightStatus DjiUser_FlightControlGetValueOfFlightStatus();
T_DjiFcSubscriptionVelocity DjiUser_FlightControlGetValueOfVelocity();
T_DjiFcSubscriptionPositionFused DjiUser_FlightControlGetValueOfPositionFused();
T_DjiFcSubscriptionQuaternion DjiUser_FlightControlGetValueOfRawQuat();
T_DjiFcSubscriptionDisplaymode DjiUser_FlightControlGetValueOfDisplaymode();
T_DjiFcSubscriptionAltitudeOfHomePoint
DjiUser_FlightControlGetValueOfHomePointAltitude();
T_DjiFcSubscriptionHomePointInfo DjiUser_FlightControlGetValueOfHomePointInfo();
T_DjiFcSubscriptionCompass DjiUser_FlightControlGetValueOfCompass();
T_DjiFcSubscriptionMotorStartError
DjiUser_FlightControlGetValueOfMotorStatusError();
T_DjiFcSubscriptionFlightAnomaly DjiUser_FlightControlGetValueOfFlightAnomaly();
T_DjiFcSubscriptionRC DjiUser_FlightControlGetValueOfRC();
T_DjiFcSubscriptionRtkPosition DjiUser_FlightControlGetValueOfRtkPosition();
T_DjiFcSubscriptionRtkPositionInfo
DjiUser_FlightControlGetValueOfRtkPositionInfo();
T_DjiFcSubscriptionRtkVelocity DjiUser_FlightControlGetValueOfRtkVelocity();
T_DjiFcSubscriptionRtkYaw DjiUser_FlightControlGetValueOfRtkYaw();
T_DjiFcSubscriptionRtkYawInfo DjiUser_FlightControlGetValueOfRtkYawInfo();
T_DjiFcSubscriptionRTKConnectStatus
DjiUser_FlightControlGetValueOfRTKConnectStatus();
T_DjiFcSubscriptionGimbalAngles DjiUser_FlightControlGetValueOfGimbalAngles();

T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1();
T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2();

T_DjiAircraftInfoBaseInfo DjiUser_FlightControlGetValueOfAircraftBaseInfo();

// TODO{zengxw} no need to init camera??
E_DjiCameraType DjiUser_FlightControlGetValueOfCameraType(
    E_DjiMountPosition mountPosition);

enum class COLLISION_AVOIDANCE_DIRECTION { HORIZONTAL = 0, UP, DOWN };
enum class COLLISION_AVOIDANCE_TYPE { RADAR = 0, VISUAL };
E_DjiFlightControllerObstacleAvoidanceEnableStatus
DjiUser_FlightControlGetValueOfCollisionAvoidance(
    COLLISION_AVOIDANCE_DIRECTION direction, COLLISION_AVOIDANCE_TYPE type);

T_DjiVector3f quaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion& quat);
std::string getFlightAnomaly(
    const T_DjiFcSubscriptionFlightAnomaly& flyAnomaly);
double getDistanceByRad(double radLat1, double radLng1, double radLat2,
                        double radLng2);
std::string getPositionSolutionString(int posInfo);
std::string cameraTypeEnumToString(const E_DjiCameraType& camera_type);
std::string cameraWorkModeEnumToString(
    const E_DjiCameraManagerWorkMode& camera_work_mode);

#endif  // _SUBSCRIBE_HELPER_H_
