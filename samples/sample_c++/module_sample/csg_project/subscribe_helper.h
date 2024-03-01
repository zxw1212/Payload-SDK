
#ifndef _SUBSCRIBE_HELPER_H_
#define _SUBSCRIBE_HELPER_H_

#include <string>

#include "dji_fc_subscription.h"
#include "dji_typedef.h"
#include <dji_aircraft_info.h>

// TODO{zengxw} add ConfigUpdate and JoystickCtrlAuthorityEventCallback
// TODO{zengxw} Init camera and gimbal here
// TODO{zengxw} add deinit function
void* DjiTest_FlightControlInitAndSubscribeTopics(void* arg);

// some helper function to get subscription value

// 融合高度,相对于海平面
T_DjiFcSubscriptionAltitudeFused DjiUser_FlightControlGetValueOfAltitudeFused();
// 融合相对地面高度
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

// TODO{zengxw} make sure if flight can return battery info without calling
// subscription.
T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery1();
T_DjiFcSubscriptionSingleBatteryInfo DjiUser_FlightControlGetValueOfBattery2();

T_DjiAircraftInfoBaseInfo DjiUser_FlightControlGetValueOfAircraftBaseInfo();

E_DjiCameraType DjiUser_FlightControlGetValueOfCameraType(
    E_DjiMountPosition mountPosition);

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

enum class COLLISION_AVOIDANCE_DIRECTION { HORIZONTAL = 0, UP, DOWN };
enum class COLLISION_AVOIDANCE_TYPE { RADAR = 0, VISUAL };
#endif  // _SUBSCRIBE_HELPER_H_
