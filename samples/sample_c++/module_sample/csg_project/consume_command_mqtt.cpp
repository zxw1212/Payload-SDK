#include <iostream>
#include <string>
#include <chrono>
#include <cstdlib>
#include <time.h>
#include <json/json.h>
#include <vector>
#include "consume_command_mqtt.h"
#include "subscribe_helper.h"
#include "callback_state.h"

#include "dji_fc_subscription.h"
#include "dji_flight_controller.h"
#include "dji_aircraft_info.h"
#include "dji_platform.h"
#include "dji_typedef.h"
#include "dji_logger.h"
#include "dji_liveview.h"
#include "dji_waypoint_v2.h"
#include "dji_waypoint_v2_type.h"
#include "dji_gimbal_manager.h"
#include "dji_camera_manager.h"

#include "windsp.h"

// task handler
T_DjiTaskHandle s_myGoHomeThread;
T_DjiTaskHandle s_myLandingThread;
T_DjiTaskHandle s_myGoHomeAndConfirmLandingThread;
T_DjiTaskHandle s_myLandingAndConfirmLandingThread;
T_DjiTaskHandle s_myJoyStickThread;


#define GO_HOME_TASK_STACK_SIZE 2048
#define LANDING_TASK_STACK_SIZE 2048
#define GO_HOME_AND_CONFIRM_LANDING_TASK_STACK_SIZE 2048
#define LANDING_AND_CONFIRM_LANDING_TASK_STACK_SIZE 2048
#define MY_JOYSTICK_TASK_STACK_SIZE 2048


extern string modeFly;
extern Json::Value jv_waypoint;
extern mqtt::topic* topic_cb;

extern int i_CAMERA_MODE;
extern bool bool_RTMPFlag;
extern int i_Mode;

extern std::string s_CLIENT_ID;
extern std::string s_RTMP_URI;
extern std::string s_HTTP_URI;
extern std::string s_TOPIC_MISSION;
extern std::string s_TOPIC_GENERAL;
extern std::string s_TOPIC_RC;
extern std::string s_TOPIC_CAMERA;
extern std::string s_TOPIC_V3;
extern std::string s_TOPIC_FAN;

int fanStep = 0;
int fileNum = 0;

#define TEST_CAMERA_MANAGER_MEDIA_FILE_NAME_MAX_SIZE             256
FILE *s_downloadMediaFile = NULL;
T_DjiCameraManagerFileList s_meidaFileList;
uint32_t downloadStartMs = 0;
uint32_t downloadEndMs = 0;
char downloadFileName[TEST_CAMERA_MANAGER_MEDIA_FILE_NAME_MAX_SIZE] = {0};
uint32_t s_nextDownloadFileIndex = 0;

typedef struct {
    uint8_t missionState;
    char *stateStr;
} T_DjiTestWaypointV2StateStr;
const T_DjiTestWaypointV2StateStr s_waypointV2StateStr[] = {
    {.missionState = 0x00, .stateStr = "Ground station not start"},
    {.missionState = 0x01, .stateStr = "Mission prepared"},
    {.missionState = 0x02, .stateStr = "Enter mission"},
    {.missionState = 0x03, .stateStr = "Execute mission"},
    {.missionState = 0x04, .stateStr = "Pause Mission"},
    {.missionState = 0x05, .stateStr = "Enter mission after ending pause"},
    {.missionState = 0x06, .stateStr = "Exit mission"},
    {.missionState = 0xFF, .stateStr = "Unknown"}
};

typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiTestFlightControlDisplayModeStr;

const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};
uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;
    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }
    return i;
}

bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}

    int actionNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiUser_FlightControlGetValueOfDisplaymode() != mode && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr,
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                           DjiUser_FlightControlGetValueOfDisplaymode())].displayModeStr);
        return false;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                      s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr);
        return true;
    }
}

bool DjiTest_FlightControlMotorStartedCheck(void)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}

    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiUser_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
           DjiUser_FlightControlGetValueOfDisplaymode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        osalHandler->TaskSleepMs(100);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
}

bool DjiTest_FlightControlTakeOffInAirCheck(void)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}

    int stillOnGround = 0;
    int timeoutCycles = 110;

    while (DjiUser_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           (DjiUser_FlightControlGetValueOfDisplaymode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            DjiUser_FlightControlGetValueOfDisplaymode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        osalHandler->TaskSleepMs(100);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

void takeoffFinishedCheck(void)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}
    while (DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
           DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
        osalHandler->TaskSleepMs(1000);
    }

    return (DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

// TODO{zengxw} go home action may contains start landing step.
bool startGoHome(void){
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return false;
	}
	
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    aircraftInfoBaseInfo = DjiUser_FlightControlGetValueOfAircraftBaseInfo();
	
	USER_LOG_INFO("Start go home action");
	T_DjiReturnCode djiStat = DjiFlightController_StartGoHome();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start to go home failed, error code: 0x%08X", djiStat);
        return false;
    }

    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)) {
        return NULL;
    } else {
        while (DjiUser_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }

	return true;
}

bool isLandingToSafeHeight(){
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return false;
	}
	T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    aircraftInfoBaseInfo = DjiUser_FlightControlGetValueOfAircraftBaseInfo();

	USER_LOG_INFO("Start landing to safe height action");
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action");
        return false;
    }
	while (DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
			DjiUser_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
		T_DjiFcSubscriptionHeightFusion heightFusion = DjiUser_FlightControlGetValueOfHeightFused();
		osalHandler->TaskSleepMs(1000);
		// to_safe_heighth
		if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3E ||
			aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3T ||
			aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3D ||
			aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3TD) {
			if ((dji_f64_t) 0.45 < heightFusion && heightFusion < (dji_f64_t) 0.55) {
				break;
			}
		} else {
			if ((dji_f64_t) 0.65 < heightFusion && heightFusion < (dji_f64_t) 0.75) {
				break;
			}
		}
	}
    
	return true;
}

bool startLandingToSafeHeight(){
	T_DjiReturnCode djiStat = DjiFlightController_StartLanding();
    if(!isLandingToSafeHeight()) return false;

	return true;
}

bool isLandingToGround(){
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return false;
	}

	USER_LOG_INFO("Start landing to ground action");
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action");
        return false;
    }
	while (DjiUser_FlightControlGetValueOfDisplaymode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
			DjiUser_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
		osalHandler->TaskSleepMs(1000);
	}
    
	return true;
}

bool startLandingToGround(){
	T_DjiReturnCode djiStat = DjiFlightController_StartLanding();
    if(!isLandingToGround()) return false;

	return true;
}

// after arrived safe height, confirm landing to groud.
bool confirmLanding(void)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return false;
	}

	T_DjiReturnCode djiStat;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus enableStatus;
	djiStat = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&enableStatus);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get downwards visual obstacle avoidance enable status error");
		return false;
    }

	USER_LOG_INFO("Start confirm Landing to ground action");
    djiStat = DjiFlightController_StartConfirmLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", djiStat);
        return false;
    }

    if (enableStatus == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
        if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)) {
			USER_LOG_ERROR("Downwards collision enable, can only land on DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING mode");
            return false;
        } else {
            while (DjiUser_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
                   DjiTest_FlightControlGetValueOfDisplayMode() ==
                   DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
                osalHandler->TaskSleepMs(1000);
            }
        }
    } else {
        while (DjiUser_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() ==
               DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
            osalHandler->TaskSleepMs(1000);
        }
    }
	return true;
}

bool landingFinishedCheck(void) {
	if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
		return true;
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        return false;
    }
}

// Go home, landing to safe height, then confirm landing
void* MyGoHomeAndConfirmLanding_Task(void* arg)
{
    /*! Step 1: Start go home */
	if(!startGoHome()) return NULL;

    /*! Step 2: Start landing */
    if(!isLandingToSafeHeight()) return NULL;

    /*! Step 4: Confirm Landing */
    if(!confirmLanding()) return NULL;

    /*! Step 5: Landing finished check*/
    if(!landingFinishedCheck()) return NULL;

	return NULL;
}

// Landing to safe height, then confirm landing
void* MyLandingAndConfirmLanding_Task(void* arg)
{
   /*! Step 1: Verify and setup the subscription */
  //Subscription by main thread

  /*! Step 2: Start landing */
	if(!startLandingToSafeHeight()) return NULL;

  /*! Step 3: Confirm Landing */
    if(!confirmLanding()) return NULL;

  /*! Step 4: Landing finished check*/
    if(!landingFinishedCheck()) return NULL;

  return NULL;
}

// Go home, landing to safe height
void* MyGoHome_Task(void* arg)
{
	/*! Step 1: Start go home */
	if(!startGoHome()) return NULL;

	/*! Step 2: Start landing */
	if(!isLandingToSafeHeight()) return NULL;

	/*! Confirm Landing by Remote-Control */
}

// Land directly to ground without confirming
void* MyLanding_Task(void* arg)
{
	// start landing to ground
	USER_LOG_INFO("Start landing action");
	if(!startLandingToGround()) return NULL;

	// landing finished check
	if(!landingFinishedCheck()) return NULL;
    
	return NULL;
}

std::time_t  now_t;
Json::Reader reader;
Json::Value  jsonValue_msg;
std::string  sCode, sParam;
int          iCode, iParam;
float        fx, fy, fParam;
Json::Value  jsonValue_param, jsonValue_cb;
bool         bool_JoystickCtrlAuthority = true;

std::string  uploadUrl;

typedef struct
{
	std::string missionID;
	std::string Name;
	double yaw;
	double bladeLen;
	double towerHeight;
	double cenLat;
	double cenLon;
	double cenAlt;
	double speedForCheck;
	double distanceForCheck;
	double speedForWork;
	double distanceForWork;
	int    flightAlt;
	int    returnAlt;
	double relativeAlt;
	int    shootThickness;

	std::string fanId;
	std::string airportId;
	std::string remark;
	std::string delFlag;
	std::string airportName;
	std::string substationName;
	std::string updateBy;
	std::string id;
	std::string creator;
	float 		bladeLength;
	std::string updateTime;
	std::string createTime;
	std::string name;

}FanData;


FanData fd;

void MQTT_Mission();
void MQTT_RC();
void MQTT_Camera();
void MQTT_Gen();
void MQTT_V3();
void MQTT_FAN();
void TaskFan();
void TaskFan2(std::string fileName);


T_DjiReturnCode DjiTest_CameraManagerDownloadFileDataCallback(T_DjiDownloadFilePacketInfo packetInfo,
                                                                     const uint8_t *data, uint16_t len)
{
    int32_t i;
    float downloadSpeed = 0.0f;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();

    if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START) {
        for (i = 0; i < s_meidaFileList.totalCount; ++i) {
            if (s_meidaFileList.fileListInfo[i].fileIndex == packetInfo.fileIndex) {
                s_nextDownloadFileIndex = i + 1;
                break;
            }
        }
        osalHandler->GetTimeMs(&downloadStartMs);

        memset(downloadFileName, 0, sizeof(downloadFileName));
        snprintf(downloadFileName, sizeof(downloadFileName), "%s", s_meidaFileList.fileListInfo[i].fileName);
        USER_LOG_INFO("Start download media file, index : %d, next download media file, index: %d", i,
                      s_nextDownloadFileIndex);
        s_downloadMediaFile = fopen(downloadFileName, "wb+");
        if (s_downloadMediaFile == NULL) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
        fwrite(data, 1, len, s_downloadMediaFile);
    } else if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_TRANSFER) {
        if (s_downloadMediaFile != NULL) {
            fwrite(data, 1, len, s_downloadMediaFile);
        }
        printf("\033[1;32;40m ### [Complete rate : %0.1f%%] (%s), size: %u, fileIndex: %d\033[0m\r\n",
               packetInfo.progressInPercent, downloadFileName, packetInfo.fileSize, packetInfo.fileIndex);
        printf("\033[1A");
        USER_LOG_DEBUG("Transfer download media file data, len: %d, percent: %.1f", len, packetInfo.progressInPercent);
    } else if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END) {
        if (s_downloadMediaFile == NULL) {
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }

        fwrite(data, 1, len, s_downloadMediaFile);

        osalHandler->GetTimeMs(&downloadEndMs);

        downloadSpeed = (float) packetInfo.fileSize / (float) (downloadEndMs - downloadStartMs);
        printf("\033[1;32;40m ### [Complete rate : %0.1f%%] (%s), size: %u, fileIndex: %d\033[0m\r\n",
               packetInfo.progressInPercent, downloadFileName, packetInfo.fileSize, packetInfo.fileIndex);
        printf("\033[1A");
        printf("\r\n");
        USER_LOG_INFO("End download media file, Download Speed %.2f KB/S\r\n\r\n", downloadSpeed);
        fclose(s_downloadMediaFile);
        s_downloadMediaFile = NULL;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

typedef struct {
	int download;
	int downloadFail;
	int upload;
	int uploadFail;
	int uploadTotal;
}FileSend;

FileSend fs;

void UpdateStatus(string pCode)
{
	Json::Value jo;
	jo["code"] = "1001";
	jo["pCode"] = pCode;
	jo["date"] = (int)time(0);

	if (pCode == "00000")
	{
		jo["msg"] = "未定义";
	}
	else if (pCode == "00001")
	{
		jo["msg"] = "任务上传";
	}
	else if (pCode == "00002")
	{
		jo["msg"] = jv_waypoint["execMissionID"];
	}
	else if (pCode == "00003")
	{
		jo["msg"] = "任务开始失败";
	}
	else if (pCode == "00004")
	{
		jo["msg"] = "设置 Home 点位置";
	}
	else if (pCode == "00005")
	{
		jo["msg"] = "任务完成";
	}
	else if (pCode == "00006")
	{
		jo["msg"] = "下载/上传任务照片";
		jo["param"]["download"] = fs.download;
		jo["param"]["downloadFail"] = fs.downloadFail;
		jo["param"]["upload"] = fs.upload;
		jo["param"]["uploadFail"] = fs.uploadFail;
		jo["param"]["uploadTotal"] = fs.uploadTotal;

	}
	else if (pCode == "00007")
	{
		jo["msg"] = "机库回收完成";
	}
	else if (pCode == "00008")
	{
		jo["msg"] = "任务（简易机库等待确认时）取消";
	}
	else if (pCode == "00009")
	{
		jo["msg"] = "任务等待确认时，即已经收到任务并处于等待确认";
	}
	else if (pCode == "00010")
	{
		jo["msg"] = "任务处于断点状态";
	}
	else if (pCode == "00011")
	{
		jo["msg"] = "任务开始续飞";
	}

	std::cout << "UpdateStatus " << jo.toStyledString() << std:endl;
	topic_cb->publish(jo.toStyledString());
}

void executeCMD(const char* cmd, char* result)
{
	printf("executeCMD %s\r\n", result);

	char buf_ps[1024];
	char ps[1024] = { 0 };
	FILE* ptr;
	strcpy(ps, cmd);
	if ((ptr = popen(ps, "r")) != NULL)
	{
		while (fgets(buf_ps, 1024, ptr) != NULL)
		{
			strcat(result, buf_ps);
			if (strlen(result) > 1024)
				break;
		}
		pclose(ptr);
		ptr = NULL;
	}
	else
	{
		printf("popen %s error\n", ps);
	}
}

//{
//	"execMissionID": "1681397602",
//		"waypointIndex" : 1,
//		"latitude" : 20.60670661,
//		"longitude" : 110.48439074,
//		"altitude" : 50.24,
//		"height" : 90.68,
//		"mediaType" : 0
//}
///home/rer/mybin/bin/hyPost 1681397602 2 20.60670661 110.48439074 50.24 90.68 1 052.jpg 2>&1
int SendFile(string execMissionID, int waypointIndex, double latitude, double longitude, double altitude, double height, int mediaType, string filePath, string urls)
{
	char bufSend[256];
	char bufRec[256];
	memset(bufSend, 0, 256);
	memset(bufRec, 0, 256);

	printf("sendfile\r\n");

	sprintf(bufSend, "/home/rer/mybin/bin/hyPost %s %d %lf %lf %lf %lf %d %s %s 2>&1",
		execMissionID.c_str(), waypointIndex, latitude, longitude, altitude, height, mediaType, filePath.c_str(),urls.c_str());

	printf("%s\r\n", bufSend);

	executeCMD(bufSend, bufRec);
	std::cout << bufRec << std::endl;
	return 1;
}

void* MySendFile_Task(void* arg)
{

	if (jv_waypoint["missionState"] == "FINISH")
	{
		std::cout << "下载任务已经开始" << std::endl;
	}
	else
	{
		jv_waypoint["missionState"] = "FINISH";
		std::cout << "开始下载图片" << std::endl;

		char path[80] = { 0 };
		char cmdStr[80] = { 0 };
		sprintf(path, "./task/%s", jv_waypoint["execMissionID"].asCString());
		sprintf(cmdStr, "mkdir -p %s",path);
		printf("%s", cmdStr);
		std::system(cmdStr);

		UpdateStatus("00007");

		T_DjiReturnCode returnCode;
		T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
		uint16_t downloadCount = 0;

		s_nextDownloadFileIndex = 0;
		returnCode = DjiCameraManager_RegDownloadFileDataCallback(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DjiTest_CameraManagerDownloadFileDataCallback);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Register download file data callback failed, error code: 0x%08X.", returnCode);
			return returnCode;
		}

		returnCode = DjiCameraManager_ObtainDownloaderRights(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Obtain downloader rights failed, error code: 0x%08X.", returnCode);
		}
		
		returnCode = DjiCameraManager_DownloadFileList(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &s_meidaFileList);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Download file list failed, error code: 0x%08X.", returnCode);
			return returnCode;
		}

		if (s_meidaFileList.totalCount > 0) {
			downloadCount = s_meidaFileList.totalCount;
			printf(
				"\033[1;33;40m -> Download file list finished, total file count is %d, the following %d is list details: \033[0m\r\n",
				s_meidaFileList.totalCount, downloadCount);
			for (int i = 0; i < downloadCount; ++i) {
				if (s_meidaFileList.fileListInfo[i].fileSize < 1 * 1024 * 1024) {
					printf(
						"\033[1;32;40m ### Media file_%03d name: %s, index: %d, time:%04d-%02d-%02d_%02d:%02d:%02d, size: %.2f KB, type: %d \033[0m\r\n",
						i, s_meidaFileList.fileListInfo[i].fileName,
						s_meidaFileList.fileListInfo[i].fileIndex,
						s_meidaFileList.fileListInfo[i].createTime.year,
						s_meidaFileList.fileListInfo[i].createTime.month,
						s_meidaFileList.fileListInfo[i].createTime.day,
						s_meidaFileList.fileListInfo[i].createTime.hour,
						s_meidaFileList.fileListInfo[i].createTime.minute,
						s_meidaFileList.fileListInfo[i].createTime.second,
						(dji_f32_t) s_meidaFileList.fileListInfo[i].fileSize / 1024,
						s_meidaFileList.fileListInfo[i].type);
				} else {
					printf(
						"\033[1;32;40m ### Media file_%03d name: %s, index: %d, time:%04d-%02d-%02d_%02d:%02d:%02d, size: %.2f MB, type: %d \033[0m\r\n",
						i, s_meidaFileList.fileListInfo[i].fileName,
						s_meidaFileList.fileListInfo[i].fileIndex,
						s_meidaFileList.fileListInfo[i].createTime.year,
						s_meidaFileList.fileListInfo[i].createTime.month,
						s_meidaFileList.fileListInfo[i].createTime.day,
						s_meidaFileList.fileListInfo[i].createTime.hour,
						s_meidaFileList.fileListInfo[i].createTime.minute,
						s_meidaFileList.fileListInfo[i].createTime.second,
						(dji_f32_t) s_meidaFileList.fileListInfo[i].fileSize / (1024 * 1024),
						s_meidaFileList.fileListInfo[i].type);
				}
			}
			printf("\r\n");

			osalHandler->TaskSleepMs(1000);

			downloadCount =  s_meidaFileList.totalCount;

			for (int i = 0; i < downloadCount; ++i) {
			redownload:
				if (i != s_nextDownloadFileIndex) {
					i = s_nextDownloadFileIndex;
				}

				returnCode = DjiCameraManager_DownloadFileByIndex(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, s_meidaFileList.fileListInfo[i].fileIndex);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("Download media file by index failed, error code: 0x%08X.", returnCode);
					s_nextDownloadFileIndex--;
					goto redownload;
				}
			}

			osalHandler->TaskSleepMs(1000);
		} else {
			USER_LOG_WARN("Media file is not existed in sdcard.\r\n");
		}

		std::string  lastPathNFilename = "";
		int mediaFileListSize = s_meidaFileList.totalCount;
		for (int i = 0; i < fileNum; i++)
		{
			std::string last_path_N_file_name = s_meidaFileList.fileListInfo[mediaFileListSize - 1 - i * 2].fileName;
			std::cout << "last " << last_path_N_file_name << std::endl;

			fs.download = i + 1;
			fs.downloadFail = 0;
			fs.upload = i;
			fs.uploadFail = 0;
			fs.uploadTotal = fileNum;
			UpdateStatus("00006");


			std::size_t found = last_path_N_file_name.find_last_of("/");
			std::string fileName;
			if (found != std::string::npos) {
				fileName = last_path_N_file_name.substr(found + 1);
			} else {
				fileName = last_path_N_file_name;
			}
			std::string s2 = "/";
		    lastPathNFilename = path + s2 + fileName;

			std::string copyCommand = "cp " + last_path_N_file_name + " " + lastPathNFilename;
			int result = std::system(copyCommand.c_str());
			if (result != 0) {
				std::cout << "文件拷贝失败！" << std::endl;
				return NULL;
			}
			
			if (fanStep == 1)
			{
				;
			}
			else if (fanStep == 2)
			{
				//SendFile(jv_waypoint["execMissionID"].asString(), 0, 20.607123, 110.221142, 80.25, 50.21, 0, lastPathNFilename);

				fs.download = i + 1;
				fs.downloadFail = 0;
				fs.upload = i + 1;
				fs.uploadFail = 0;
				fs.uploadTotal = fileNum;
				UpdateStatus("00006");

			}
		}
		if (fanStep == 1)
		{
			if (i_Mode == 0)
			{
				TaskFan2("/task/test.jpg");
			}
			else
			{
				TaskFan2(lastPathNFilename);
			}
		}
		else if (fanStep == 2)
		{


			UpdateStatus("00005");


			char bufSend[256];
			char bufRec[256];
			memset(bufSend, 0, 256);
			memset(bufRec, 0, 256);

			printf("sendfile\r\n");

			sprintf(bufSend, "/home/rer/mybin/bin/hyPost %s  %s 2>&1", jv_waypoint["execMissionID"].asCString(), uploadUrl.c_str());

			printf("%s\r\n", bufSend);

			executeCMD(bufSend, bufRec);
			std::cout << bufRec << std::endl;
		}

	}
}


// joystick task
int joyTime = 0;
float joyX = 0.0;
float joyY = 0.0;
float joyZ = 0.0;
float joyW = 0.0;
void* MyJoytick_Task(void* arg)
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}

	T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
	DjiFlightController_SetJoystickMode(joystickMode);

	while (true)
	{
		static float joyX_old, joyY_old, joyZ_old, joyW_old = 0.0; // TODO{zengxw} what is these used for?
		if (joyTime > 50)
		{
			joyTime = joyTime - 50;
			if (joyX_old == joyX && joyY_old == joyY && joyZ_old == joyZ && joyW_old == joyW)
			{
				cout << "joytime contniue " << joyTime << endl;
			}
			else
			{
				cout << "joytime joystickAction " << joyTime << endl;
			}

			static T_DjiFlightControllerJoystickCommand joystickCommand = {joyX, joyY, joyZ,joyW};
			T_DjiReturnCode returnCode;
			returnCode = DjiFlightController_ExecuteJoystickAction(joystickCommand);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
				return NULL;
			}

			joyX_old = joyX;
			joyY_old = joyY;
			joyZ_old = joyZ;
			joyW_old = joyW;
		}
		else
		{
			//flightSample->emergencyBrake();
			//cout << "joytime emergencyBrake " << joyTime << endl;
		}
		osalHandler->TaskSleepMs(50);
	}
}


T_DjiReturnCode DjiTest_WaypointV2EventCallback(T_DjiWaypointV2MissionEventPush eventData)
{
    if (eventData.event == 0x01) {
        USER_LOG_INFO("[%s]: Mission interrupted reason is 0x%x",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.interruptReason);
    } else if (eventData.event == 0x02) {
        USER_LOG_INFO("[%s]: Mission recover reason is 0x%x",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.recoverProcess);
    } else if (eventData.event == 0x03) {
        USER_LOG_INFO("[%s]: Mission exit reason is 0x%x",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.exitReason);
    } else if (eventData.event == 0x10) {
        USER_LOG_INFO("[%s]: Current waypoint index is %d",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.waypointIndex);
    } else if (eventData.event == 0x11) {
        USER_LOG_INFO("[%s]: Current mission execute times is %d",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.T_DjiWaypointV2MissionExecEvent.currentMissionExecTimes);
    } else if (eventData.event == 0x12) {
        USER_LOG_INFO("[%s]: avoid obstacle state:%d",
                      s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
                      eventData.data.avoidState);
    } else if (eventData.event == 0x30) {
        USER_LOG_INFO(
            "[%s]: action id:%d, pre actuator state:%d, current actuator state:%d, result:0x%08llX",
            s_waypointV2EventStr[DJiTest_WaypointV2GetMissionEventIndex(eventData.event)].eventStr,
            eventData.data.T_DjiWaypointV2ActionExecEvent.actionId,
            eventData.data.T_DjiWaypointV2ActionExecEvent.preActuatorState,
            eventData.data.T_DjiWaypointV2ActionExecEvent.curActuatorState,
            eventData.data.T_DjiWaypointV2ActionExecEvent.result
        );
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

uint8_t DjiTest_WaypointV2GetMissionStateIndex(uint8_t state)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_waypointV2StateStr) / sizeof(T_DjiTestWaypointV2StateStr); i++) {
        if (s_waypointV2StateStr[i].missionState == state) {
            return i;
        }
    }

    return i;
}

T_DjiReturnCode DjiTest_WaypointV2StateCallback(T_DjiWaypointV2MissionStatePush stateData)
{
    static uint32_t curMs = 0;
    static uint32_t preMs = 0;
    osalHandler->GetTimeMs(&curMs);
    if (curMs - preMs >= 1000) {
        preMs = curMs;
        USER_LOG_INFO("[Waypoint Index:%d]: State: %s, velocity:%.2f m/s",
                      stateData.curWaypointIndex,
                      s_waypointV2StateStr[DjiTest_WaypointV2GetMissionStateIndex(stateData.state)].stateStr,
                      (dji_f32_t) stateData.velocity / 100);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void* ConsumeUserCommand_Task(void* arg)
{
	// Add joystick task
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    returnCode = osalHandler->TaskCreate("MyJoytick_Task", MyJoytick_Task,
                                         MY_JOYSTICK_TASK_STACK_SIZE, NULL,
                                         &s_myJoyStickThread);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Create MyJoytick_Task failed, errno = 0x%08llX", returnCode);
        return NULL;
    }


	// Waypoint V2 init
	returnCode = DjiWaypointV2_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init waypoint V2 module error, stat:0x%08llX", returnCode);
        return NULL;
    }
	osalHandler->TaskSleepMs(1000);

	// WaypointV2 register call back
    returnCode = DjiWaypointV2_RegisterMissionEventCallback(DjiTest_WaypointV2EventCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register waypoint V2 event failed, error code: 0x%08X", returnCode);
        goto out;
    }
    returnCode = DjiWaypointV2_RegisterMissionStateCallback(DjiTest_WaypointV2StateCallback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register waypoint V2 state failed, error code: 0x%08X", returnCode);
        goto out;
    }
    osalHandler->TaskSleepMs(1000);


	returnCode =  DjiFlightController_ReleaseJoystickCtrlAuthority();
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
    }
	returnCode = DjiFlightController_ObtainJoystickCtrlAuthority(); //TODO{zengxw} For what?
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    }

	// camera init
	returnCode = DjiCameraManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init camera manager failed, error code: 0x%08X\r\n", returnCode);
    }

	// gimble init
	returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init gimble manager failed, error code: 0x%08X\r\n", returnCode);
    }

	UpdateStatus("00000");

	try
	{
		while (true)
		{
			now_t = std::time(NULL);

			jsonValue_cb.clear();
			jsonValue_cb["date"] = (int)now_t;
			auto msg = cli->consume_message();
			if (!msg)
			{
				break;
			}
			if (!(reader.parse(msg->to_string(), jsonValue_msg)))
			{
				jsonValue_cb["pCode"] = "UNKNOWN";
				jsonValue_cb["code"] = callback_json_error;
				jsonValue_cb["msg"] = "json parse error";
			}
			else
			{
				if (!(jsonValue_msg.isMember("code") && jsonValue_msg["code"].isString()))
				{
					jsonValue_cb["pCode"] = "UNKNOWN";
					jsonValue_cb["code"] = callback_param_error;
					jsonValue_cb["msg"] = "param error without code";
				}
				else
				{
					sCode = jsonValue_msg["code"].asString();
					printf("code is %s\r\n", sCode.c_str());

					jsonValue_cb["pCode"] = sCode;
					iCode = std::stoi(sCode);

					if (msg->get_topic() == s_TOPIC_MISSION)
					{
						// Mission task
						MQTT_Mission();
					}
					else if (msg->get_topic() == s_TOPIC_CAMERA)
					{
						// Camera task
						MQTT_Camera();
					}
					else if (msg->get_topic() == s_TOPIC_GENERAL)
					{
						// General task
						MQTT_Gen();
					}
					else if (msg->get_topic() == s_TOPIC_V3)
					{
						// ... 
						MQTT_V3();
					}
					else if (msg->get_topic() == s_TOPIC_RC)
					{
						// Remote control task
						MQTT_RC();
					}
					else if (msg->get_topic() == s_TOPIC_FAN)
					{
						// Fan task
						MQTT_FAN();
					}
				}
			}
			std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
			topic_cb->publish(jsonValue_cb.toStyledString());
		}

	}
	catch (const mqtt::exception& exc)
	{
		std::cerr << "\n  " << exc << std::endl;
	}
	
out:
    returnCode = DjiTest_WaypointV2DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit waypoint V2 sample failed, error code: 0x%08X", returnCode);
		return NULL;
    }
}

typedef struct MissionTask
{
	std::string wayPointActionParam;
	std::string wayPointAction;

	double wayPointLongitude;
	double wayPointLatitude;
	double wayPointAltitude;

	double wayPointSpeed;
	double shootPhotoTimeInterval;
	double shootPhotoDistanceInterval;
} MissionTask;

// print WaypointV2 location
void fun(WaypointV2 val)
{
	std::cout.precision(15);
	std::cout << val.latitude << ' ' << val.longitude << ' ' << std::endl;
}

// This function splits the input string str according to the specified delimiter pattern and returns a vector resVec composed of the split substrings.
std::vector<std::string> splitWithStl(const std::string& str, const std::string& pattern)
{
	std::vector<std::string> resVec;
	if ("" == str)
	{
		return resVec;
	}
	//方便截取最后一段数据
	std::string strs = str + pattern;

	size_t pos = strs.find(pattern);
	size_t size = strs.size();
	while (pos != std::string::npos)
	{
		std::string x = strs.substr(0, pos);
		resVec.push_back(x);
		strs = strs.substr(pos + 1, size);
		pos = strs.find(pattern);
	}

	return resVec;
}


// print WaypointV2 actionId and action details
void printActionDetails(T_DJIWaypointV2Action val)
{
	std::cout.precision(15);

	std::cout << " actionId " << val.actionId;
	std::cout <<  " Type " << val.actuator.actuatorType;
	std::cout <<  " Index " << val.actuator.actuatorIndex;
	std::cout << std::endl;

	if (val.actuator.actuatorType == E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA)
	{
		std::cout << " actionId " << val.actionId << " operationType "  << val.actuator.cameraActuatorParam.operationType << "  " << std::endl;
	}
	else if (val.actuator.actuatorType == E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL)
	{
		std::cout << " actionId " << val.actionId << " rotation "<< val.actuator.gimbalActuatorParam.rotation.y << "  " << std::endl;
	}
	else if (val.actuator.actuatorType == E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL)
	{
		std::cout << " actionId " << val.actionId << " yawRotatingParam " << val.actuator.aircraftControlActuatorParam.yawRotatingParam.yaw << "  " << std::endl;
	}
	
}

/*
missonTasks:		Task1(1 waypoint and some actions1.1, 1.2...), 				Task2     ...		TaskN
actionVector:		Action_start1  {ac1.1, ac1.2...}  Action_end1				Action_start2   ...
*/
std::vector<T_DJIWaypointV2Action> generateWaypointActions3(MissionTask missonTasks[], uint16_t actionNum)
{
	std::vector<T_DJIWaypointV2Action> actionVector;

	int actionId = 0;
	for (uint16_t i = 0; i < actionNum; i++)
	{
		std::vector<std::string> actions = splitWithStl(missonTasks[i].wayPointAction, ",");
		std::vector<std::string> pars = splitWithStl(missonTasks[i].wayPointActionParam, ",");

    	T_DJIWaypointV2Trigger trigger;
		trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_TYPE_SAMPLE_REACH_POINT;
		trigger.sampleReachPointTriggerParam.waypointIndex = i;
		trigger.sampleReachPointTriggerParam.terminateNum = 0;

		T_DJIWaypointV2Actuator actuator;
		actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
		// for UAV with multiple camera or gimbal
        actuator.actuatorIndex = 0;
        actuator.aircraftControlActuatorParam.operationType = E_DJIWaypointV2ActionActuatorAircraftControlOperationType::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
		actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 0;

		T_DJIWaypointV2Action action;
		action.actionId = actionId++;
        memcpy(&action.actuator, &actuator, sizeof(actuator));
        memcpy(&action.trigger, &trigger, sizeof(trigger));
		actionVector.push_back(action);


		for (int j = 0; j < actions.size(); j++)
		{
			std::string act = actions[j];
			std::string par = pars[j];
			std::cout << "action " << act << ", param " << par << std::endl;

			T_DJIWaypointV2AssociateTriggerParam associateTriggerParam;
			associateTriggerParam.actionAssociatedType = E_DJIWaypointV2TriggerAssociatedTimingType::DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISHED;
			associateTriggerParam.actionIdAssociated = actionId - 1;

			if (act == "0")
			{
				associateTriggerParam.waitingTime = atoi(par.c_str());
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;
			}
			else if (act == "1")
			{
				//photo
				T_DJIWaypointV2Actuator actuator;
				actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
				actuator.actuatorIndex = 0;
				actuator.cameraActuatorParam.operationType = E_DJIWaypointV2ActionActuatorCameraOperationType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO;

				associateTriggerParam.waitingTime = 3;
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;

				T_DJIWaypointV2Action action;
				action.actionId = actionId++;
				memcpy(&action.actuator, &actuator, sizeof(actuator));
				memcpy(&action.trigger, &trigger, sizeof(trigger));
				actionVector.push_back(action);
				std::cout << "1  actionVector.push_back " << std::endl;
			}
			else if (act == "2")
			{
				//record
				// TODO{zengxw} why record pairs to DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_TAKE_PHOTO in osdk project?????
				T_DJIWaypointV2Actuator actuator;
				actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
				actuator.actuatorIndex = 0;
				actuator.cameraActuatorParam.operationType = E_DJIWaypointV2ActionActuatorCameraOperationType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_START_RECORD_VIDEO;

				associateTriggerParam.waitingTime = 0;
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;

				T_DJIWaypointV2Action action;
				action.actionId = actionId++;
				memcpy(&action.actuator, &actuator, sizeof(actuator));
				memcpy(&action.trigger, &trigger, sizeof(trigger));
				actionVector.push_back(action);
				cout << "2  actionVector.push_back " << endl;
			}
			else if (act == "3")
			{
				//stop record
				T_DJIWaypointV2Actuator actuator;
				actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_CAMERA;
				actuator.actuatorIndex = 0;
				actuator.cameraActuatorParam.operationType = E_DJIWaypointV2ActionActuatorCameraOperationType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_CAMERA_OPERATION_TYPE_STOP_RECORD_VIDEO;

				associateTriggerParam.waitingTime = 0;
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;

				T_DJIWaypointV2Action action;
				action.actionId = actionId++;
				memcpy(&action.actuator, &actuator, sizeof(actuator));
				memcpy(&action.trigger, &trigger, sizeof(trigger));
				actionVector.push_back(action);
				cout << "3  actionVector.push_back " << actuator.cameraActuatorParam.operationType << endl;
			}
			else if (act == "4")
			{
				//yaw
				T_DJIWaypointV2Actuator actuator;
				actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
				actuator.actuatorIndex = 0;
				actuator.aircraftControlActuatorParam.operationType = E_DJIWaypointV2ActionActuatorAircraftControlOperationType::DJIWaypointV2ActionActuatorAircraftControlOperationTypeRotateYaw;
				actuator.aircraftControlActuatorParam.yawRotatingParam.isRelative = 0;
				actuator.aircraftControlActuatorParam.yawRotatingParam.reserved = 7;
				actuator.aircraftControlActuatorParam.yawRotatingParam.yaw = atof(par.c_str());

				associateTriggerParam.waitingTime = 0;
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;

				T_DJIWaypointV2Action action;
				action.actionId = actionId++;
				memcpy(&action.actuator, &actuator, sizeof(actuator));
				memcpy(&action.trigger, &trigger, sizeof(trigger));
				actionVector.push_back(action);
				cout << "4  actionVector.push_back " << actuator.aircraftControlActuatorParam.yawRotatingParam.yaw << endl;
			}
			else if (act == "5")
			{
				//gimbal
				T_DJIWaypointV2Actuator actuator;
				actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_GIMBAL;
				actuator.actuatorIndex = 0;
				actuator.gimbalActuatorParam.operationType = E_DJIWaypointV2ActionActuatorGimbalOperationType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_GIMBAL_OPERATION_TYPE_ROTATE_GIMBAL;
				actuator.gimbalActuatorParam.rotation.x = 0;
				actuator.gimbalActuatorParam.rotation.y = (int)(atof(par.c_str()) * 10);
				actuator.gimbalActuatorParam.rotation.z = 0;
				actuator.gimbalActuatorParam.rotation.ctrl_mode = 0;
				actuator.gimbalActuatorParam.rotation.rollCmdIgnore = 1;
				actuator.gimbalActuatorParam.rotation.pitchCmdIgnore = 0;
				actuator.gimbalActuatorParam.rotation.yawCmdIgnore = 1;
				actuator.gimbalActuatorParam.rotation.absYawModeRef = 0;
				actuator.gimbalActuatorParam.rotation.reserved = 3;
				actuator.gimbalActuatorParam.rotation.durationTime = 20;

				associateTriggerParam.waitingTime = 0;
				T_DJIWaypointV2Trigger trigger;
				trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
				trigger.associateTriggerParam = associateTriggerParam;

				T_DJIWaypointV2Action action;
				action.actionId = actionId++;
				memcpy(&action.actuator, &actuator, sizeof(actuator));
				memcpy(&action.trigger, &trigger, sizeof(trigger));
				actionVector.push_back(action);
				cout << "5 actionVector.push_back " << endl;
			}
		}


		// TODO{zengxw} For what? since the next loop starts with an isStartFlying=0 action.
		T_DJIWaypointV2Trigger trigger;
		trigger.actionTriggerType = E_DJIWaypointV2ActionTriggerType::DJI_WAYPOINT_V2_ACTION_TRIGGER_ACTION_ASSOCIATED;
		trigger.associateTriggerParam.actionAssociatedType = E_DJIWaypointV2TriggerAssociatedTimingType::DJI_WAYPOINT_V2_TRIGGER_ASSOCIATED_TIMING_TYPE_AFTER_FINISHED;
		trigger.associateTriggerParam.actionIdAssociated = actionId - 1;
		trigger.associateTriggerParam.waitingTime = 1;

		T_DJIWaypointV2Actuator actuator;
		actuator.actuatorType = E_DJIWaypointV2ActionActuatorType::DJI_WAYPOINT_V2_ACTION_ACTUATOR_TYPE_AIRCRAFT_CONTROL;
        actuator.actuatorIndex = 0;
        actuator.aircraftControlActuatorParam.operationType = E_DJIWaypointV2ActionActuatorAircraftControlOperationType::DJIWaypointV2ActionActuatorAircraftControlOperationTypeFlyingControl;
		actuator.aircraftControlActuatorParam.flyControlParam.isStartFlying = 1;

		T_DJIWaypointV2Action action;
		action.actionId = actionId++;
        memcpy(&action.actuator, &actuator, sizeof(actuator));
        memcpy(&action.trigger, &trigger, sizeof(trigger));
		actionVector.push_back(action);
	}

	for_each(actionVector.begin(), actionVector.end(), printActionDetails);

	return actionVector;
}

std::vector<T_DjiWaypointV2> generatePolygonWaypoints3(MissionTask missonTasks[], int nums)
{
	for (int i = 0; i < nums; i++)
	{
		std::cout.precision(15);
		std::cout << "[" << i << "]";
		std::cout << " lat:" << missonTasks[i].wayPointLatitude;
		std::cout << " lng:" << missonTasks[i].wayPointLongitude;
		std::cout << " alt:" << missonTasks[i].wayPointAltitude;

		std::cout << " param:" << missonTasks[i].wayPointActionParam;
		std::cout << " action:" << missonTasks[i].wayPointAction;

		std::cout << " speed:" << missonTasks[i].wayPointSpeed;

		std::cout << " shooting time interval:" << missonTasks[i].shootPhotoTimeInterval;
		std::cout << " shooting distance interval:" << missonTasks[i].shootPhotoDistanceInterval;
		std::cout << std::endl;
	}

	std::vector<T_DjiWaypointV2> waypointList;
	T_DjiWaypointV2              startPoint = {};
	T_DjiWaypointV2              waypointV2 = {};

	startPoint.latitude = missonTasks[0].wayPointLatitude * 3.141592653589 / 180;
	startPoint.longitude = missonTasks[0].wayPointLongitude * 3.141592653589 / 180;
	startPoint.relativeHeight = missonTasks[0].wayPointAltitude;

	std::cout << "startPoint latitude:" << startPoint.latitude << "startPoint longitude:" << startPoint.longitude << std::endl;

	startPoint.waypointType = E_DJIWaypointV2FlightPathMode::DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
	startPoint.headingMode = E_DJIWaypointV2HeadingMode::DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
	startPoint.config.useLocalCruiseVel = 0;
	startPoint.config.useLocalMaxVel = 0;

	startPoint.dampingDistance = 40;
	startPoint.heading = 0;
	startPoint.turnMode = E_DJIWaypointV2TurnMode::DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

	startPoint.pointOfInterest.positionX = 0;
	startPoint.pointOfInterest.positionY = 0;
	startPoint.pointOfInterest.positionZ = 0;
	startPoint.maxFlightSpeed = 5;
	startPoint.autoFlightSpeed = missonTasks[0].wayPointSpeed;

	waypointList.push_back(startPoint);

	for (int i = 1; i < nums; i++)
	{
		waypointV2.waypointType = E_DJIWaypointV2FlightPathMode::DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_IN_STRAIGHT_AND_STOP;
		waypointV2.headingMode = E_DJIWaypointV2HeadingMode::DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
		waypointV2.config.useLocalCruiseVel = 0;
		waypointV2.config.useLocalMaxVel = 0;

		waypointV2.dampingDistance = 40;
		waypointV2.heading = 0;
		waypointV2.turnMode = E_DJIWaypointV2TurnMode::DJI_WAYPOINT_V2_TURN_MODE_CLOCK_WISE;

		waypointV2.pointOfInterest.positionX = 0;
		waypointV2.pointOfInterest.positionY = 0;
		waypointV2.pointOfInterest.positionZ = 0;
		waypointV2.maxFlightSpeed = 5;
		waypointV2.autoFlightSpeed = missonTasks[i].wayPointSpeed;

		waypointV2.latitude = missonTasks[i].wayPointLatitude * 3.141592653589 / 180;
		waypointV2.longitude = missonTasks[i].wayPointLongitude * 3.141592653589 / 180;
		waypointV2.relativeHeight = missonTasks[i].wayPointAltitude;
		waypointList.push_back(waypointV2);
	}
	return waypointList;
}

void MQTT_Mission()
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}
	
	printf("MQTT_Mission\r\n");
	switch (iCode)
	{
	case 10001:
		break;
	case 10002:
		break;
	case 10003:
		if (jsonValue_msg.isMember("param"))
		{
			UpdateStatus("00001");
			
			// TODO{zengxw} can not find psdk api like this
			// vehicle->control->obtainCtrlAuthority(1);

			T_DjiWayPointV2MissionSettings missionInitSettings;


			missionInitSettings.missionID = rand();
			missionInitSettings.repeatTimes = 1;
			missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_GO_HOME;
			missionInitSettings.maxFlightSpeed = 10;
			missionInitSettings.autoFlightSpeed = 2;
			missionInitSettings.actionWhenRcLost = E_DJIWaypointV2MissionActionWhenRcLost::DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
			missionInitSettings.gotoFirstWaypointMode = E_DJIWaypointV2MissionGotoFirstWaypointMode::DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;

			modeFly = "READY_TO_GO";

			if (jsonValue_msg["param"].isMember("finishAction"))
			{
				auto finishAction = jsonValue_msg["param"]["finishAction"].asInt();
				std::cout << "finishAction:" << finishAction << std::endl;

				if (finishAction == 0)
				{
					missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
				}
				else if (finishAction == 1)
				{
					missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_GO_HOME;
				}
				else if (finishAction == 2)
				{
					missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_AUTO_LANDING;
				}
				else if (finishAction == 3)
				{
					missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_GO_TO_FIRST_WAYPOINT;
				}
				else if (finishAction == 4)
				{
					missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_CONTINUE_UNTIL_STOP;
				}
			}

			if (jsonValue_msg["param"].isMember("gotoFirstWaypointMode"))
			{

				auto gotoFirstWaypointMode =
					jsonValue_msg["param"]["gotoFirstWaypointMode"].asInt();
				std::cout << "gotoFirstWaypointMode:" << gotoFirstWaypointMode << std::endl;
				if (gotoFirstWaypointMode == 1)
				{
					missionInitSettings.gotoFirstWaypointMode = E_DJIWaypointV2MissionGotoFirstWaypointMode::DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
				}
				else if (gotoFirstWaypointMode == 2)
				{
					missionInitSettings.gotoFirstWaypointMode =
						E_DJIWaypointV2MissionGotoFirstWaypointMode::DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_POINT_TO_POINT;
				}
			}

			if (jsonValue_msg["param"].isMember("autoFlightSpeed"))
			{
				auto autoFlightSpeed =
					jsonValue_msg["param"]["autoFlightSpeed"].asInt();
				std::cout << "autoFlightSpeed:" << autoFlightSpeed << std::endl;

				missionInitSettings.autoFlightSpeed = autoFlightSpeed;
			}

			if (jsonValue_msg["param"].isMember("missionID"))
			{
				auto missionID = jsonValue_msg["param"]["missionID"].asString();
				std::cout << "missionID:" << missionID << std::endl;
			}

			if (jsonValue_msg["param"].isMember("flightPathMode"))
			{
				auto flightPathMode = jsonValue_msg["param"]["flightPathMode"].asInt();
				std::cout << "flightPathMode:" << flightPathMode << std::endl;
				// TODO{zengxw} flightPathMode is not used. same for missionID isRelativeAltitude...
			}

			if (jsonValue_msg["param"].isMember("isRelativeAltitude"))
			{
				auto isRelativeAltitude = jsonValue_msg["param"]["isRelativeAltitude"].asString();
				std::cout << "isRelativeAltitude:" << isRelativeAltitude << std::endl;
			}

			if (jsonValue_msg["param"].isMember("useHomeSeaLevelInRtkUnable"))
			{
				auto useHomeSeaLevelInRtkUnable = jsonValue_msg["param"]["useHomeSeaLevelInRtkUnable"].asString();
				std::cout << "useHomeSeaLevelInRtkUnable:" << useHomeSeaLevelInRtkUnable << std::endl;
			}

			if (jsonValue_msg["param"].isMember("homePointLatitude"))
			{
				auto homePointLatitude = jsonValue_msg["param"]["homePointLatitude"];
				std::cout << "homePointLatitude:" << homePointLatitude << std::endl;
			}

			if (jsonValue_msg["param"].isMember("name"))
			{
				auto name = jsonValue_msg["param"]["name"].asString();
				std::cout << "name:" << name << std::endl;
			}

			if (jsonValue_msg["param"].isMember("headingMode"))
			{
				auto headingMode = jsonValue_msg["param"]["headingMode"].asString();
				std::cout << "headingMode:" << headingMode << std::endl;
			}

			if (jsonValue_msg["param"].isMember("clearHomeLocation"))
			{
				auto clearHomeLocation = jsonValue_msg["param"]["clearHomeLocation"].asString();
				std::cout << "clearHomeLocation:" << clearHomeLocation << std::endl;
			}

			if (jsonValue_msg["param"].isMember("autoFlightSpeed"))
			{
				auto homePointLongitude = jsonValue_msg["param"]["homePointLongitude"].asString();
				std::cout << "homePointLongitude:" << homePointLongitude << std::endl;
			}

			if (jsonValue_msg["executeTime"].isMember("executeTime"))
			{
				auto executeTime = jsonValue_msg["param"]["executeTime"].asString();
				std::cout << "executeTime:" << executeTime << std::endl;
			}

			if (jsonValue_msg["param"].isMember("mission"))
			{
				auto mission = jsonValue_msg["param"]["mission"];

				int sz = jsonValue_msg["param"]["mission"].size();

				MissionTask missonTasks[sz];

				for (int i = 0; i < sz; i++)
				{
					
					missonTasks[i].wayPointAction = jsonValue_msg["param"]["mission"][i]["wayPointAction"].asString();
					missonTasks[i].wayPointActionParam = jsonValue_msg["param"]["mission"][i]["wayPointActionParam"].asString();
					missonTasks[i].shootPhotoDistanceInterval = jsonValue_msg["param"]["mission"][i]["shootPhotoDistanceInterval"].asDouble();
					missonTasks[i].shootPhotoTimeInterval = jsonValue_msg["param"]["mission"][i]["shootPhotoTimeInterval"].asDouble();
					missonTasks[i].wayPointSpeed = jsonValue_msg["param"]["mission"][i]["wayPointSpeed"].asDouble();
					missonTasks[i].wayPointAltitude = jsonValue_msg["param"]["mission"][i]["wayPointAltitude"].asDouble();
					missonTasks[i].wayPointLongitude = jsonValue_msg["param"]["mission"][i]["wayPointLongitude"].asDouble();
					missonTasks[i].wayPointLatitude = jsonValue_msg["param"]["mission"][i]["wayPointLatitude"].asDouble();
				}

				const auto mission_vec = generatePolygonWaypoints3(missonTasks, sz);
				T_DjiWaypointV2 missions[sz];
				for (size_t i = 0; i < sz; i++) {
					missions[i] = mission_vec[i];
				}
				missionInitSettings.mission = missions;
				missionInitSettings.missTotalLen = sz;

				for_each(mission_vec.begin(), mission_vec.end(), fun);

				const auto action_vec = generateWaypointActions3(missonTasks, sz);
				T_DJIWaypointV2Action actions[sz];
				for (size_t i = 0; i < sz; i++) {
					actions[i] = action_vec[i];
				}
				T_DJIWaypointV2ActionList action_list;
				action_list.actions = actions;
				action_list.actionNum = sz;
				missionInitSettings.actionList = action_list;

				T_DjiReturnCode returnCode = DjiWaypointV2_UploadMission(&missionInitSettings);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("Init waypoint V2 mission setting failed, ErrorCode:0x%lX", returnCode);
					// TODO{zengxw} Error handle.
				}
				osalHandler->TaskSleepMs(1000);

				jv_waypoint["execMissionID"] = to_string(rand());
				jv_waypoint["missionID"] = jsonValue_msg["param"]["missionID"];
				jv_waypoint["missionReachIndex"] = 0;
				jv_waypoint["missionState"] = "START";
				jv_waypoint["missionUploadedIndex"] = sz;
				jv_waypoint["missionExecState"] = "INITIALIZING";

				fanStep = 2;
				fileNum = sz;

				jsonValue_param["missionID"] = jv_waypoint["execMissionID"];

				jsonValue_cb["param"] = jsonValue_param.toStyledString();
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
			else
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without missionID";
				break;
			}

		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error";
		}
		break;
	case 10004:
		break;
	case 10006:
		if (jsonValue_msg.isMember("param"))
		{
			UpdateStatus("00002");
			if (jsonValue_msg["param"].isMember("missionID"))
			{
				auto missionID = jsonValue_msg["param"]["missionID"];
				std::cout << "missionID:" << missionID << std::endl;


				if (jsonValue_msg["param"]["missionDataSync"].isMember("uploadUrl"))
				{
				    uploadUrl = jsonValue_msg["param"]["missionDataSync"]["uploadUrl"].asString();
					std::cout << "uploadUrl:" << uploadUrl << std::endl;
				}

				if (jsonValue_msg["param"]["missionDataSync"].isMember("mode"))
				{
					auto mode = jsonValue_msg["param"]["missionDataSync"]["mode"];
					std::cout << "mode:" << mode << std::endl;
				}

				jv_waypoint["missionState"] = "START";

				T_DjiReturnCode returnCode = DjiWaypointV2_Start();
				if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					modeFly = "EXECUTING";
					jsonValue_cb["code"] = callback_success_code;
					jsonValue_cb["msg"] = "success";
				} else{
					USER_LOG_ERROR("Start waypoint V2 mission failed, error code: 0x%08X", returnCode);
					modeFly = "STANDBY";
					UpdateStatus("00003");
					UpdateStatus("00005");
					jsonValue_cb["code"] = callback_param_error;
					jsonValue_cb["msg"] = "startWaypointMission error";
				}
				osalHandler->TaskSleepMs(1000);
			}
			else
			{
				modeFly = "STANDBY";
				UpdateStatus("00003");
				UpdateStatus("00005");
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without missionID";
				break;
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error";
		}
		break;
	case 10009:
		break;
	case 10010:
		T_DjiReturnCode returnCode = DjiWaypointV2_Pause();
		if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{
			USER_LOG_ERROR("Pause waypoint V2 failed, error code: 0x%08X", returnCode);
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "pauseWaypointMission failed";
		}
		osalHandler->TaskSleepMs(1000);
		break;
	case 10011:
		T_DjiReturnCode returnCode = DjiWaypointV2_Resume();
		if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{
			USER_LOG_ERROR("Resume waypoint V2 failed, error code: 0x%08X", returnCode);
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "pauseWaypointMission failed";
		}
		osalHandler->TaskSleepMs(1000);
		break;
	case 10012:
		break;
	case 10013:
		break;
	case 10014:
		if (1)
		{
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "stop failed";
		}
		break;
	case 10015:
		break;
	case 10017:
		break;
	case 10018:
		break;
	default:
		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
		break;
	}
}

void TaskFan()
{
	T_DjiOsalHandler* osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}

	// TODO{zengxw} Seems lack of file.
	struct ParametersWind pa;

	pa.pianhang = fd.yaw;
	pa.jiangye = 50.53;
	pa.tagaoH = fd.towerHeight;
	pa.jiangyeR = fd.bladeLen;

	pa.startingPoint[0] = fd.cenLat;
	pa.startingPoint[1] = fd.cenLon;
	pa.startingPoint[2] = fd.cenAlt;

	pa.shootS0 = fd.distanceForCheck;
	pa.shootS = fd.distanceForWork;
	pa.shootD = fd.relativeAlt;
	pa.shootMidu = fd.shootThickness;

	printf("%10.8f,%10.8f,%10.8f,%10.8f\r\n", pa.pianhang, pa.jiangye, pa.tagaoH, pa.jiangyeR);
	printf("%10.8f,%10.8f,%10.8f\r\n", pa.startingPoint[0], pa.startingPoint[1], pa.startingPoint[2]);
	printf("%10.8f,%10.8f,%10.8f,%d\r\n", pa.shootS0, pa.shootS, pa.shootD, pa.shootMidu);

	PointWork pw;

	int res = model_emulat(pa, &pw);

	printf("res :%d\r\n", res);
	printf("start %10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.start[0].lat, pw.start[0].lng, pw.start[0].alt, pw.start[0].e, pw.start[0].f);
	printf("start %10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.start[1].lat, pw.start[1].lng, pw.start[1].alt, pw.start[1].e, pw.start[1].f);
	for (int i = 0; i < res; i++)
	{
		printf("%10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.work[i].lat, pw.work[i].lng, pw.work[i].alt, pw.work[i].e, pw.work[i].f);
	}
	printf("fun task start %s\r\n", fd.missionID.c_str());


	// TODO{zengxw} can not find psdk api like this
	// vehicle->control->obtainCtrlAuthority(1);

	T_DjiWayPointV2MissionSettings missionInitSettings;

	missionInitSettings.missionID = rand();
	missionInitSettings.repeatTimes = 1;
	missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
	missionInitSettings.maxFlightSpeed = 10;
	missionInitSettings.autoFlightSpeed = fd.speedForCheck;
	missionInitSettings.actionWhenRcLost = E_DJIWaypointV2MissionActionWhenRcLost::DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
	missionInitSettings.gotoFirstWaypointMode = E_DJIWaypointV2MissionGotoFirstWaypointMode::DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;


	modeFly = "READY_TO_GO";

	MissionTask missonTasks[2];

	for (int i = 0; i < 2; i++)
	{
		char buf[256];
		memset(buf, 0, 256);
		sprintf(buf, "%f,%f,1", pw.start[i].e, pw.start[i].f);

		missonTasks[i].wayPointAction = "4,5,1";
		missonTasks[i].wayPointActionParam = buf;
		missonTasks[i].shootPhotoDistanceInterval = 0.0;
		missonTasks[i].shootPhotoTimeInterval = 0.0;
		missonTasks[i].wayPointSpeed = 4.0;
		missonTasks[i].wayPointAltitude = pw.start[i].alt;
		missonTasks[i].wayPointLongitude = pw.start[i].lng;;
		missonTasks[i].wayPointLatitude = pw.start[i].lat;
		printf("get lat %.14lf %.14lf\r\n", missonTasks[i].wayPointLatitude, missonTasks[i].wayPointLongitude);
	}
	
	const auto mission_vec = generatePolygonWaypoints3(missonTasks, 2);
	T_DjiWaypointV2 missions[2];
	for (size_t i = 0; i < 2; i++) {
		missions[i] = mission_vec[i];
	}
	missionInitSettings.mission = missions;
	missionInitSettings.missTotalLen = 2;

	for_each(mission_vec.begin(), mission_vec.end(), fun);

	const auto action_vec = generateWaypointActions3(missonTasks, 2);
	T_DJIWaypointV2Action actions[2];
	for (size_t i = 0; i < 2; i++) {
		actions[i] = action_vec[i];
	}
	T_DJIWaypointV2ActionList action_list;
	action_list.actions = actions;
	action_list.actionNum = 2;
	missionInitSettings.actionList = action_list;
	
	T_DjiReturnCode returnCode = DjiWaypointV2_UploadMission(&missionInitSettings);
	if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
		USER_LOG_ERROR("Init waypoint V2 mission setting failed, ErrorCode:0x%lX", returnCode);
		// TODO{zengxw} Error handle.
	}
	osalHandler->TaskSleepMs(1000);


	jsonValue_param["missionID"] = jv_waypoint["execMissionID"].asString();
	jsonValue_param["missionTotal"] = 2;
	jsonValue_param["missionCount"] = 1;


	Json::Value item;
	for (int i = 0; i < 2; i++)
	{
		item["wayPointLatitude"] = missonTasks[0].wayPointLatitude;
		item["wayPointLongitude"] = missonTasks[0].wayPointLongitude;
		item["wayPointAltitude"] = missonTasks[0].wayPointAltitude;
		item["wayPointSpeed"] = missonTasks[0].wayPointSpeed;
		item["wayPointAction"] = missonTasks[0].wayPointAction;
		item["wayPointActionParam"] = missonTasks[0].wayPointActionParam;

		jsonValue_param["mission"].append(item);
	}



	jsonValue_cb["param"] = jsonValue_param;
	jsonValue_cb["msg"] = "success";
	jsonValue_cb["code"] = callback_success_code;

	std::cout << jsonValue_cb.toStyledString() << std::endl;


	jv_waypoint["execMissionID"] = to_string(rand());
	jv_waypoint["missionID"] = jsonValue_msg["param"]["missionID"];
	jv_waypoint["missionReachIndex"] = 0;
	jv_waypoint["missionState"] = "START";
	jv_waypoint["missionUploadedIndex"] = 2;
	jv_waypoint["missionExecState"] = "INITIALIZING";

	fanStep = 1;
	fileNum = 1;
}

void TaskFan2(string fileName)
{
	struct ParametersWind pa;


	char bufSend[256];
	char bufRec[256];
	memset(bufSend, 0, 256);
	memset(bufRec, 0, 256);
	sprintf(bufSend, "python3 /home/rer/mybin/bin/windPost.py %s 2>&1", fileName.c_str());
	printf("%s\r\n", bufSend);

	executeCMD(bufSend, bufRec);
	std::cout << bufRec << std::endl;

	Json::Reader reader;
	Json::Value root;
	if (!reader.parse(bufRec, root, false))
	{
		std::cout << "parse json file error!" << std::endl;
	}

	if (root.isMember("path"))
	{
		std::cout << "path " << root["path"].asString() << std::endl;

		SendFile(jv_waypoint["execMissionID"].asString(), 0, 20.607123, 110.221142, 80.25, 50.21, 0, root["path"].asString(), uploadUrl);

		fs.download = 1;
		fs.downloadFail = 0;
		fs.upload = 1;
		fs.uploadFail = 0;
		fs.uploadTotal = 1;
		UpdateStatus("00006");
	}
	if (root.isMember("angle"))
	{
		std::cout << "angle " << root["angle"].asDouble() << std::endl;

		pa.pianhang = fd.yaw;
		pa.jiangye = root["angle"].asDouble();
		//pa.jiangye = 4.2;


		pa.tagaoH = fd.towerHeight;
		pa.jiangyeR = fd.bladeLen;

		pa.startingPoint[0] = fd.cenLat;
		pa.startingPoint[1] = fd.cenLon;
		pa.startingPoint[2] = fd.cenAlt;

		pa.shootS0 = fd.distanceForCheck;
		pa.shootS = fd.distanceForWork;
		pa.shootD = fd.relativeAlt;
		pa.shootMidu = fd.shootThickness;

		printf("%10.8f,%10.8f,%10.8f,%10.8f\r\n", pa.pianhang, pa.jiangye, pa.tagaoH, pa.jiangyeR);
		printf("%10.8f,%10.8f,%10.8f\r\n", pa.startingPoint[0], pa.startingPoint[1], pa.startingPoint[2]);
		printf("%10.8f,%10.8f,%10.8f,%d\r\n", pa.shootS0, pa.shootS, pa.shootD, pa.shootMidu);

		PointWork pw;

		int res = model_emulat(pa, &pw);
		printf("res :%d\r\n", res);
		printf("start %10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.start[0].lat, pw.start[0].lng, pw.start[0].alt, pw.start[0].e, pw.start[0].f);
		printf("start %10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.start[1].lat, pw.start[1].lng, pw.start[1].alt, pw.start[1].e, pw.start[1].f);
		for (int i = 0; i < res; i++)
		{
			printf("%10.8f,%10.8f,%10.8f,%10.8f,%10.8f\r\n", pw.work[i].lat, pw.work[i].lng, pw.work[i].alt, pw.work[i].e, pw.work[i].f);
		}
		printf("fun task start %s\r\n", fd.missionID.c_str());


    	T_DjiWaypointV2GlobalCruiseSpeed getGlobalCruiseSpeed = 0;
		T_DjiReturnCode returnCode = DjiWaypointV2_GetGlobalCruiseSpeed(&getGlobalCruiseSpeed);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Get global cruise speed failed, error code: 0x%08X", returnCode);
		}
		USER_LOG_INFO("Current global cruise speed is %f m/s", getGlobalCruiseSpeed);
		osalHandler->TaskSleepMs(1000);


		// TODO{zengxw} can not find psdk api like this
		// vehicle->control->obtainCtrlAuthority(1);
		T_DjiWayPointV2MissionSettings missionInitSettings;

		missionInitSettings.missionID = rand();
		missionInitSettings.repeatTimes = 1;
		missionInitSettings.finishedAction = E_DJIWaypointV2MissionFinishedAction::DJI_WAYPOINT_V2_FINISHED_GO_HOME;
		missionInitSettings.maxFlightSpeed = 10;
		missionInitSettings.autoFlightSpeed = 2;

		missionInitSettings.actionWhenRcLost = E_DJIWaypointV2MissionActionWhenRcLost::DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
		missionInitSettings.gotoFirstWaypointMode = E_DJIWaypointV2MissionGotoFirstWaypointMode::DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;

		modeFly = "READY_TO_GO";

		MissionTask missonTasks[res];
		for (int i = 0; i < res; i++)
		{
			char buf[256];
			memset(buf, 0, 256);
			sprintf(buf, "%f,%f,%f", pw.work[i].e, pw.work[i].f, 0.0);

			missonTasks[i].wayPointAction = "4,5,1";
			missonTasks[i].wayPointActionParam = buf;
			missonTasks[i].shootPhotoDistanceInterval = 0.0;
			missonTasks[i].shootPhotoTimeInterval = 0.0;
			missonTasks[i].wayPointSpeed = 5.0;
			missonTasks[i].wayPointAltitude = pw.work[i].alt;
			missonTasks[i].wayPointLongitude = pw.work[i].lng;;
			missonTasks[i].wayPointLatitude = pw.work[i].lat;
		}

		printf("get lat %.14lf %.14lf\r\n", missonTasks[0].wayPointAltitude, missonTasks[0].wayPointLongitude);

		const auto mission_vec = generatePolygonWaypoints3(missonTasks, res);
		T_DjiWaypointV2 missions[res];
		for (size_t i = 0; i < res; i++) {
			missions[i] = mission_vec[i];
		}
		missionInitSettings.mission = missions;
		missionInitSettings.missTotalLen = res;

		for_each(mission_vec.begin(), mission_vec.end(), fun);

		const auto action_vec = generateWaypointActions3(missonTasks, res);
		T_DJIWaypointV2Action actions[res];
		for (size_t i = 0; i < res; i++) {
			actions[i] = action_vec[i];
		}
		T_DJIWaypointV2ActionList action_list;
		action_list.actions = actions;
		action_list.actionNum = res;
		missionInitSettings.actionList = action_list;
		
		T_DjiReturnCode returnCode = DjiWaypointV2_UploadMission(&missionInitSettings);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Init waypoint V2 mission setting failed, ErrorCode:0x%lX", returnCode);
			// TODO{zengxw} Error handle.
		}
		osalHandler->TaskSleepMs(1000);

		jsonValue_cb.clear();
		jsonValue_cb["date"] = (int)now_t;

		jsonValue_param["missionID"] = jv_waypoint["execMissionID"].asString();
		jsonValue_param["missionTotal"] = 2;
		jsonValue_param["missionCount"] = 2;
		Json::Value item;
		for (int i = 0; i < res; i++)
		{
			item["wayPointLatitude"] = missonTasks[i].wayPointLatitude;
			item["wayPointLongitude"] = missonTasks[i].wayPointLongitude;
			item["wayPointAltitude"] = missonTasks[i].wayPointAltitude;
			item["wayPointSpeed"] = missonTasks[i].wayPointSpeed;
			item["wayPointAction"] = missonTasks[i].wayPointAction;
			item["wayPointActionParam"] = missonTasks[i].wayPointActionParam;

			jsonValue_param["mission"].append(item);
		}

		jsonValue_cb["param"] = jsonValue_param;
		jsonValue_cb["msg"] = "success";
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["pCode"] = "341001";

		std::cout << jsonValue_cb.toStyledString() << std::endl;

		topic_cb->publish(jsonValue_cb.toStyledString());

		fanStep = 2;
		fileNum = res;
	}
}

void MQTT_FAN()
{
	printf("MQTT_FAN\r\n");
	switch (iCode)
	{
	case 341001:
		if (!jsonValue_msg.isMember("param"))
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param";
		}
		else
		{
			//TaskFan2("test.jpg");

			if (jsonValue_msg["param"].isMember("towerHeight"))
			{
				fd.towerHeight = jsonValue_msg["param"]["towerHeight"].asDouble();
			}

			if (jsonValue_msg["param"].isMember("relativeAlt"))
			{
				fd.relativeAlt = jsonValue_msg["param"]["relativeAlt"].asInt();
			}
			if (jsonValue_msg["param"].isMember("shootThickness"))
			{
				fd.shootThickness = jsonValue_msg["param"]["shootThickness"].asInt();
			}

			if (jsonValue_msg["param"].isMember("airportId"))
			{
				fd.airportId = jsonValue_msg["param"]["airportId"].asString();
			}
			if (jsonValue_msg["param"].isMember("airportName"))
			{
				fd.airportName = jsonValue_msg["param"]["airportName"].asString();
			}
			if (jsonValue_msg["param"].isMember("yaw"))
			{
				fd.yaw = jsonValue_msg["param"]["yaw"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("bladeLen"))
			{
				fd.bladeLen = jsonValue_msg["param"]["bladeLen"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("bladeLength"))
			{
				fd.bladeLength = jsonValue_msg["param"]["bladeLength"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("cenAlt"))
			{
				fd.cenAlt = jsonValue_msg["param"]["cenAlt"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("cenLat"))
			{
				fd.cenLat = jsonValue_msg["param"]["cenLat"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("cenLon"))
			{
				fd.cenLon = jsonValue_msg["param"]["cenLon"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("createTime"))
			{
				fd.createTime = jsonValue_msg["param"]["createTime"].asString();
			}
			if (jsonValue_msg["param"].isMember("creator"))
			{
				fd.creator = jsonValue_msg["param"]["creator"].asString();
			}
			if (jsonValue_msg["param"].isMember("delFlag"))
			{
				fd.delFlag = jsonValue_msg["param"]["delFlag"].asString();
			}
			if (jsonValue_msg["param"].isMember("distanceForCheck"))
			{
				fd.distanceForCheck = jsonValue_msg["param"]["distanceForCheck"].asInt();
			}
			if (jsonValue_msg["param"].isMember("distanceForWork"))
			{
				fd.distanceForWork = jsonValue_msg["param"]["distanceForWork"].asInt();
			}
			if (jsonValue_msg["param"].isMember("fanId"))
			{
				fd.fanId = jsonValue_msg["param"]["fanId"].asString();
			}
			if (jsonValue_msg["param"].isMember("id"))
			{
				fd.id = jsonValue_msg["param"]["id"].asString();
			}
			if (jsonValue_msg["param"].isMember("missionId"))
			{
				fd.missionID = jsonValue_msg["param"]["missionID"].asString();
			}
			if (jsonValue_msg["param"].isMember("Name"))
			{
				fd.Name = jsonValue_msg["param"]["Name"].asString();
			}
			if (jsonValue_msg["param"].isMember("name"))
			{
				fd.name = jsonValue_msg["param"]["name"].asString();
			}
			if (jsonValue_msg["param"].isMember("remark"))
			{
				fd.remark = jsonValue_msg["param"]["remark"].asString();
			}
			if (jsonValue_msg["param"].isMember("returnAlt"))
			{
				fd.returnAlt = jsonValue_msg["param"]["returnAlt"].asInt();
			}
			if (jsonValue_msg["param"].isMember("speedForCheck"))
			{
				fd.speedForCheck = jsonValue_msg["param"]["speedForCheck"].asInt();
			}
			if (jsonValue_msg["param"].isMember("speedForWork"))
			{
				fd.speedForWork = jsonValue_msg["param"]["speedForWork"].asInt();
			}
			if (jsonValue_msg["param"].isMember("substationName"))
			{
				fd.substationName = jsonValue_msg["param"]["substationName"].asString();
			}
			if (jsonValue_msg["param"].isMember("updateBy"))
			{
				fd.updateBy = jsonValue_msg["param"]["updateBy"].asString();
			}
			if (jsonValue_msg["param"].isMember("updateTime"))
			{
				fd.updateTime = jsonValue_msg["param"]["updateTime"].asString();
			}

			//fd.distanceForWork = 15;
			fanStep = 0;

			TaskFan();
		}
		break;
	
	default:

		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
		break;
	}
}

typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiTestFlightControlVector3f; 
void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}
    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    osalHandler->GetTimeMs(&originTime);
    osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystickMode);
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(20);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }
}

void MQTT_RC()
{
	T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
	if (!osalHandler) {
		USER_LOG_ERROR("DjiPlatform_GetOsalHandler failed");
		return NULL;
	}
	switch (iCode)
	{
	case 99000:
		jsonValue_cb["code"] = callback_unknown_error;
		jsonValue_cb["msg"] = "M300 not support";
		break;
	case 99001:
		if (!bool_JoystickCtrlAuthority)
		{
			bool_JoystickCtrlAuthority = true;
			T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
			}
		}
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["msg"] = "success";
		break;
	case 99002:
		if (bool_JoystickCtrlAuthority)
		{
			bool_JoystickCtrlAuthority = false;
			T_DjiReturnCode returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
			}
		}
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["msg"] = "success";
		break;
	case 99003:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			if (!jsonValue_msg.isMember("param"))
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without param";
			}
			else
			{
				float pitch = 0.0, roll = 0.0, yaw = 0.0, throttle = 0.0;
				if (jsonValue_msg["param"].isMember("pitch"))
				{
					pitch = jsonValue_msg["param"]["pitch"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("roll"))
				{
					roll = jsonValue_msg["param"]["roll"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("yaw"))
				{
					yaw = jsonValue_msg["param"]["yaw"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("throttle"))
				{
					throttle = jsonValue_msg["param"]["throttle"].asDouble();
				}
				std::cout << "pryt is " << pitch << "," << roll << ", " << yaw << ", " << throttle << std::endl;
				DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {pitch, roll, throttle}, yaw, 1000);
				T_DjiReturnCode returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
				}
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		break;
	case 99004:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			T_DjiReturnCode returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
			}
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 99005:
#if 1 //---------------------------------------------------------------------------------------------//
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			if (jsonValue_msg.isMember("param") &&
				jsonValue_msg["param"].isMember("mode") &&
				jsonValue_msg["param"]["mode"].isInt())
			{
				iParam = jsonValue_msg["param"]["mode"].asInt();
				if (iParam == 1)
				{
					T_DjiReturnCode returnCode = osalHandler->TaskCreate(
						"MyGoHomeAndConfirmLanding_Task", MyGoHomeAndConfirmLanding_Task,
						GO_HOME_AND_CONFIRM_LANDING_TASK_STACK_SIZE, NULL, &s_myGoHomeAndConfirmLandingThread);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
						std::cout << "======<Debug>====== Create Go Home and Confirm Landing "
							"task failed."
							<< std::endl;
						jsonValue_cb["code"] = callback_aircraft_error;
						jsonValue_cb["msg"] = "go home and confirm landing failed";
					}
					else
					{
						jsonValue_cb["code"] = callback_success_code;
						jsonValue_cb["msg"] = "success";
					}
				}
				else if (iParam == 0)
				{
					T_DjiReturnCode returnCode = osalHandler->TaskCreate(
						"MyGoHome_Task", MyGoHome_Task,
						GO_HOME_TASK_STACK_SIZE, NULL, &s_myGoHomeThread);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
						std::cout << "======<Debug>====== Create Go Home task failed." << std::endl;
						jsonValue_cb["code"] = callback_aircraft_error;
						jsonValue_cb["msg"] = "go home failed";
					}
					else
					{
						jsonValue_cb["code"] = callback_success_code;
						jsonValue_cb["msg"] = "success";
					}
				}
				else
				{
					jsonValue_cb["code"] = callback_param_error;
					jsonValue_cb["msg"] = "param error with param mode outrange";
				}
			}
			else
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without param mode";
			}
		}
		break;
#endif //--------------------------------------------------------------------------------------------//
	case 99006:
		jsonValue_cb["code"] = callback_unknown_error;
		jsonValue_cb["msg"] = "M300 not support";
		break;
	case 99007:
#if 1 //---------------------------------------------------------------------------------------------//
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			if (jsonValue_msg.isMember("param") &&
				jsonValue_msg["param"].isMember("mode") &&
				jsonValue_msg["param"]["mode"].isInt())
			{
				iParam = jsonValue_msg["param"]["mode"].asInt();
				if (iParam == 1)
				{
					T_DjiReturnCode returnCode = osalHandler->TaskCreate(
						"MyLandingAndConfirmLanding_Task", MyLandingAndConfirmLanding_Task,
						LANDING_AND_CONFIRM_LANDING_TASK_STACK_SIZE, NULL, &s_myLandingAndConfirmLandingThread);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
						std::cout << "======<Debug>====== Create Landing and Confirm Landing "
							"task failed."
							<< std::endl;
						jsonValue_cb["code"] = callback_aircraft_error;
						jsonValue_cb["msg"] = "landing and confirm landing failed";
					}
					else
					{
						jsonValue_cb["code"] = callback_success_code;
						jsonValue_cb["msg"] = "success";
					}
				}
				else if (iParam == 0)
				{
					T_DjiReturnCode returnCode = osalHandler->TaskCreate(
						"MyLanding_Task", MyLanding_Task,
						LANDING_TASK_STACK_SIZE, NULL, &s_myLandingThread);
					if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
						std::cout << "======<Debug>====== Create Landing task failed." << std::endl;
						jsonValue_cb["code"] = callback_aircraft_error;
						jsonValue_cb["msg"] = "landing failed";
					}
					else
					{
						jsonValue_cb["code"] = callback_success_code;
						jsonValue_cb["msg"] = "success";
					}
				}
				else
				{
					jsonValue_cb["code"] = callback_param_error;
					jsonValue_cb["msg"] = "param error with param mode outrange";
				}
			}
			else
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without param mode";
			}
		}
		break;
#endif //--------------------------------------------------------------------------------------------//
	case 99008:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			T_DjiReturnCode djiStat = DjiFlightController_CancelGoHome();
			if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Cancel go home failed, error code: 0x%08X", djiStat);
			}
			if (s_myGoHomeThread) {
				T_DjiReturnCode djiStat = osalHandler->TaskDestroy(s_myLandingThread);
				if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("TaskDestroy s_myLandingThread failed, error code: 0x%08X", djiStat);
				}
			}
			if (s_myGoHomeAndConfirmLandingThread) {
				T_DjiReturnCode djiStat = osalHandler->TaskDestroy(s_myGoHomeAndConfirmLandingThread);
				if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("TaskDestroy s_myGoHomeAndConfirmLandingThread failed, error code: 0x%08X", djiStat);
				}
			}
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 99009:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			T_DjiReturnCode djiStat = DjiFlightController_CancelLanding();
			if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
				USER_LOG_ERROR("Cancel landing failed, error code: 0x%08X", djiStat);
			}
			if (s_myLandingThread){
				T_DjiReturnCode djiStat = osalHandler->TaskDestroy(s_myLandingThread);
				if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("TaskDestroy s_myLandingThread failed, error code: 0x%08X", djiStat);
				}
			}
			if (s_myLandingAndConfirmLandingThread){
				T_DjiReturnCode djiStat = osalHandler->TaskDestroy(s_myLandingAndConfirmLandingThread);
				if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("TaskDestroy s_myLandingAndConfirmLandingThread failed, error code: 0x%08X", djiStat);
				}
			}
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 99010:
		jsonValue_cb["code"] = callback_unknown_error;
		jsonValue_cb["msg"] = "M300 not support";
		break;
	case 99011:
		jsonValue_cb["code"] = callback_unknown_error;
		jsonValue_cb["msg"] = "M300 not support";
		break;
	case 99012:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			flightSample->monitoredTakeoff();
			DSTATUS("Take off over!\n");
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 99023:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			if (!jsonValue_msg.isMember("param"))
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without param";
			}
			else
			{
				float pitch = 0.0, roll = 0.0, yaw = 0.0, throttle = 0.0;
				int time = 0;
				if (jsonValue_msg["param"].isMember("pitch"))
				{
					pitch = jsonValue_msg["param"]["pitch"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("roll"))
				{
					roll = jsonValue_msg["param"]["roll"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("yaw"))
				{
					yaw = jsonValue_msg["param"]["yaw"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("throttle"))
				{
					throttle = jsonValue_msg["param"]["throttle"].asDouble();
				}
				if (jsonValue_msg["param"].isMember("time"))
				{
					time = jsonValue_msg["param"]["time"].asInt();
				}
				std::cout << "pryt is " << pitch << "," << roll << ", " << yaw << ", " << throttle << std::endl;
				joyTime = time;
				joyX = pitch;
				joyY = roll;
				joyZ = throttle;
				joyW = yaw;

			/*	uint32_t originTime = 0;
				uint32_t currentTime = 0;
				uint32_t elapsedTimeInMs = 0;
				OsdkOsal_GetTimeMs(&originTime);
				OsdkOsal_GetTimeMs(&currentTime);
				elapsedTimeInMs = currentTime - originTime;

				FlightController::JoystickMode joystickMode = {
				  FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
				  FlightController::VerticalLogic::VERTICAL_VELOCITY,
				  FlightController::YawLogic::YAW_ANGLE,
				  FlightController::HorizontalCoordinate::HORIZONTAL_BODY,
				  FlightController::StableMode::STABLE_ENABLE,
				};

				vehicle->flightController->setJoystickMode(joystickMode);
				FlightController::JoystickCommand joystickCommand = { pitch, roll, throttle,yaw };
				vehicle->flightController->setJoystickCommand(joystickCommand);

				while (elapsedTimeInMs <= time)
				{
					vehicle->flightController->joystickAction();
					usleep(20000);
					OsdkOsal_GetTimeMs(&currentTime);
					elapsedTimeInMs = currentTime - originTime;
				}*/

				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		break;
	case 99024:
		if (!bool_JoystickCtrlAuthority)
		{
			jsonValue_cb["code"] = callback_aircraft_error;
			jsonValue_cb["msg"] = "Please enable the virtual joystick first";
		}
		else
		{
			if (!jsonValue_msg.isMember("param"))
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error without param";
			}
			else
			{
				int angle, speed;

				if (jsonValue_msg["param"].isMember("angle"))
				{
					angle = jsonValue_msg["param"]["angle"].asInt();
				}
				if (jsonValue_msg["param"].isMember("speed"))
				{
					speed = jsonValue_msg["param"]["speed"].asInt();
				}

				std::cout << "angle is " << angle << "," << speed << std::endl;

				DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {0, 0, 0}, angle, 2000);

				T_DjiReturnCode returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
					USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
				}

				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		break;
	default:

		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
		break;
	}
}

T_DjiReturnCode DjiTest_CameraManagerStartShootSinglePhoto(E_DjiMountPosition position)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiCameraManagerWorkMode workMode;

    /*!< set camera work mode as shoot photo */
    USER_LOG_INFO("Set mounted position %d camera's work mode as shoot-photo mode", position);
    returnCode = DjiCameraManager_SetMode(position, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("set mounted position %d camera's work mode as shoot-photo mode failed,"
                       " error code :0x%08X", position, returnCode);
        return returnCode;
    }

    osalHandler->TaskSleepMs(1000);

    returnCode = DjiCameraManager_GetMode(position, &workMode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("get mounted position %d camera's work mode failed,"
                       " error code :0x%08X", position, returnCode);
        return returnCode;
    }
    USER_LOG_INFO("Camera current workmode is %d", workMode);

    /*!< set shoot-photo mode */
    USER_LOG_INFO("Set mounted position %d camera's shoot photo mode as single-photo mode", position);
    returnCode = DjiCameraManager_SetShootPhotoMode(position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("set mounted position %d camera's shoot photo mode as single-photo mode failed,"
                       " error code :0x%08X", position, returnCode);
        return returnCode;
    }

    /*! wait the APP change the shoot-photo mode display */
    USER_LOG_INFO("Sleep 0.5s...");
    osalHandler->TaskSleepMs(500);

    /*!< start to shoot single photo */
    USER_LOG_INFO("Mounted position %d camera start to shoot photo", position);
    returnCode = DjiCameraManager_StartShootPhoto(position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Mounted position %d camera shoot photo failed, "
                       "error code :0x%08X", position, returnCode);
    }

    return returnCode;
}

T_DjiReturnCode DjiTest_CameraManagerStartRecordVideo(E_DjiMountPosition position)
{
    T_DjiReturnCode returnCode;
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    E_DjiCameraManagerWorkMode workMode;

    /*!< set camera work mode as record video */
    USER_LOG_INFO("set mounted position %d camera's work mode as record-video mode", position);
    returnCode = DjiCameraManager_SetMode(position, DJI_CAMERA_MANAGER_WORK_MODE_RECORD_VIDEO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("set mounted position %d camera's work mode as record-video mode failed,"
                       " error code :0x%08X", position, returnCode);
        return returnCode;
    }

    osalHandler->TaskSleepMs(1000);

    returnCode = DjiCameraManager_GetMode(position, &workMode);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("get mounted position %d camera's work mode failed,"
                       " error code :0x%08X", position, returnCode);
        return returnCode;
    }
    USER_LOG_INFO("Camera current workmode is %d", workMode);

    /*!< start to take video */
    USER_LOG_INFO("Mounted position %d camera start to record video.", position);
    returnCode = DjiCameraManager_StartRecordVideo(position);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("Mounted position %d camera start to record video failed,"
                       " error code:0x%08X.", position, returnCode);
    }

    return returnCode;
}

T_DjiReturnCode DjiTest_CameraManagerStopRecordVideo(E_DjiMountPosition position)
{
    T_DjiReturnCode returnCode;
    USER_LOG_INFO("Mounted position %d camera stop to record video.", position);
    returnCode = DjiCameraManager_StopRecordVideo(position);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("Mounted position %d camera stop to record video failed,"
                       " error code:0x%08X.", position, returnCode);
    }

    return returnCode;
}

T_DjiReturnCode DjiTest_CameraManagerSetFocusPoint(E_DjiMountPosition position,
                                                   T_DjiCameraManagerFocusPosData focusPoint)
{
    T_DjiReturnCode returnCode;

    /*!< set camera focus mode to be CameraModule::FocusMode::AUTO */
    USER_LOG_INFO("Set mounted position %d camera's focus mode to auto mode.",
                  position);
    returnCode = DjiCameraManager_SetFocusMode(position, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("Set mounted position %d camera's focus mode(%d) failed,"
                       " error code :0x%08X.", position, DJI_CAMERA_MANAGER_FOCUS_MODE_AUTO,
                       returnCode);
        return returnCode;
    }

    USER_LOG_INFO("Set mounted position %d camera's focus point to (%0.1f, %0.1f).",
                  position, focusPoint.focusX, focusPoint.focusY);
    returnCode = DjiCameraManager_SetFocusTarget(position, focusPoint);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("Set mounted position %d camera's focus point(%0.1f, %0.1f) failed,"
                       " error code :0x%08X.", position, focusPoint.focusX, focusPoint.focusY,
                       returnCode);
    }

    return returnCode;
}

T_DjiReturnCode DjiTest_CameraManagerOpticalZoom(E_DjiMountPosition position,
                                                 E_DjiCameraZoomDirection zoomDirection,
                                                 dji_f32_t factor)
{
    T_DjiReturnCode returnCode;
    T_DjiCameraManagerOpticalZoomParam opticalZoomParam;

    returnCode = DjiCameraManager_GetOpticalZoomParam(position, &opticalZoomParam);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_ERROR("Get mounted position %d camera's zoom param failed, error code :0x%08X",
                       position, returnCode);
        return returnCode;
    }

    USER_LOG_INFO("The mounted position %d camera's current optical zoom factor is:%0.1f x, "
                  "max optical zoom factor is :%0.1f x", position, opticalZoomParam.currentOpticalZoomFactor,
                  opticalZoomParam.maxOpticalZoomFactor);

    USER_LOG_INFO("Set mounted position %d camera's zoom factor: %0.1f x.", position, factor);
    returnCode = DjiCameraManager_SetOpticalZoomParam(position, zoomDirection, factor);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
        USER_LOG_INFO("Set mounted position %d camera's zoom factor(%0.1f) failed, error code :0x%08X",
                      position, factor, returnCode);
    }

    return returnCode;
}

void MQTT_Camera()
{
	switch (iCode)
	{
	case 13006: // set gimbal angle
		if (jsonValue_msg.isMember("param"))
		{
			float pitch = 0.0, roll = 0.0, yaw = 0.0;
			if (jsonValue_msg["param"].isMember("pitch"))
			{
				pitch = jsonValue_msg["param"]["pitch"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("roll"))
			{
				roll = jsonValue_msg["param"]["roll"].asDouble();
			}
			if (jsonValue_msg["param"].isMember("yaw"))
			{
				yaw = jsonValue_msg["param"]["yaw"].asDouble();
			}
			T_DjiGimbalManagerRotation rotation;
			rotation.pitch = pitch;
			rotation.roll = roll;
			rotation.yaw = yaw;
			rotation.rotationMode = E_DjiGimbalRotationMode::DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
			rotation.time = 0.5;
			// TODO{zengxw} use base_info.mountPosition instead of hard code here.
			T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, rotation);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "rotate gimbal failed";
            }
			else
			{
				USER_LOG_ERROR("Gimbal target rotate failed");
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param pitch, roll or yaw";
		}
		break;
	case 13007: // reset gimbal angle
	    T_DjiReturnCode returnCode = DjiGimbalManager_Reset(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, E_DjiGimbalResetMode::DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			USER_LOG_ERROR("Reset gimbal failed, error code: 0x%08X", returnCode);
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "reset gimbal failed";
		}
		else
		{
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16000:
		T_DjiAircraftInfoBaseInfo base_info =
          DjiUser_FlightControlGetValueOfAircraftBaseInfo();
		E_DjiCameraType camera_type =
			DjiUser_FlightControlGetValueOfCameraType(base_info.mountPosition);
		jsonValue_param["cameraName"] = cameraTypeEnumToString(camera_type);
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["msg"] = "success";
		break;
	case 16001:
	    E_DjiCameraManagerWorkMode workMode;
	    T_DjiReturnCode returnCode = DjiCameraManager_GetMode(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &workMode);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) { // TODO{zengxw} check this if condition
			USER_LOG_ERROR("get mounted position %d camera's work mode failed,"
						" error code :0x%08X", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["cameraMode"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get work mode failed";
		}
		else
		{
			jsonValue_param["cameraMode"] = cameraWorkModeEnumToString(workMode);
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16002:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("exposureMode"))
		{
			sParam = jsonValue_msg["param"]["exposureMode"].asString();
			E_DjiCameraManagerExposureMode exposureMode;
			if (0 == sParam.compare("PROGRAM") ) {
				exposureMode = E_DjiCameraManagerExposureMode::DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO;
			} else if (0 == sParam.compare("SHUTTER_PRIORITY")){
				exposureMode = E_DjiCameraManagerExposureMode::DJI_CAMERA_MANAGER_EXPOSURE_MODE_SHUTTER_PRIORITY;
			} else if(0 == sParam.compare("APERTURE_PRIORITY")){
				exposureMode = E_DjiCameraManagerExposureMode::DJI_CAMERA_MANAGER_EXPOSURE_MODE_APERTURE_PRIORITY;
			} else if(0 == sParam.compare("MANUAL")) {
				exposureMode = E_DjiCameraManagerExposureMode::DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_MANUAL;
			} else {
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set exposure mode failed";
				break;
			}
			T_DjiReturnCode returnCode = DjiCameraManager_SetExposureMode(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, exposureMode);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("Set mounted position %d camera's exposure mode %d failed, error code: 0x%08X", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, exposureMode, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set exposure mode failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param exposureMode";
		}
		break;
	case 16003:
		E_DjiCameraManagerExposureMode exposureModeTemp;
		T_DjiReturnCode returnCode = DjiCameraManager_GetExposureMode(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &exposureModeTemp);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d exposure mode failed, error code: 0x%08X",
						E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["exposureMode"] = "UNKNOWN"; // failed
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get exposure mode failed";
		}
		else
		{
			const char *exposureMode_names[] = {"UNKNOWN", "PROGRAM", "SHUTTER_PRIORITY", "APERTURE_PRIORITY", "MANUAL"};
			jsonValue_param["exposureMode"] = (0<exposureModeTemp && exposureModeTemp<5) ? exposureMode_names[exposureModeTemp] : "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16004:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("iso"))
		{
			sParam = jsonValue_msg["param"]["iso"].asString();
			E_DjiCameraManagerISO isoData;
			if (0 == sParam.compare("AUTO") ) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_AUTO;
			} else if (0 == sParam.compare("ISO_100")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_100;
			} else if (0 == sParam.compare("ISO_200")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_200;
			} else if (0 == sParam.compare("ISO_400")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_400;
			} else if (0 == sParam.compare("ISO_800")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_800;
			} else if (0 == sParam.compare("ISO_1600")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_1600;
			} else if (0 == sParam.compare("ISO_3200")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_3200;
			} else if (0 == sParam.compare("ISO_6400")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_6400;
			} else if (0 == sParam.compare("ISO_12800")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_12800;
			} else if (0 == sParam.compare("ISO_25600")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_25600;
			} else if (0 == sParam.compare("FIXED")) {
				isoData = E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_FIXED;
			} else { //UNKNOWN
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set ISO failed";
				break;
			}
			returnCode = DjiCameraManager_SetISO(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, isoData);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("Set mounted position %d camera's iso %d failed, "
							"error code: 0x%08X.", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, isoData, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set ISO failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param iso";
		}
		break;
	case 16005:
		E_DjiCameraManagerISO isoData;
		T_DjiReturnCode returnCode = DjiCameraManager_GetISO(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &isoData);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d camera's iso failed, error code: 0x%08X.",
						E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["iso"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get ISO failed";
		}
		else
		{
			if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_AUTO){
				sParam = "AUTO";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_100){
				sParam = "ISO_100";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_200){
				sParam = "ISO_200";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_400){
				sParam = "ISO_400";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_800){
				sParam = "ISO_800";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_1600){
				sParam = "ISO_1600";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_3200){
				sParam = "ISO_3200";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_6400){
				sParam = "ISO_6400";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_12800){
				sParam = "ISO_12800";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_25600){
				sParam = "ISO_25600";
			} else if(isoData == E_DjiCameraManagerISO::DJI_CAMERA_MANAGER_ISO_FIXED){
				sParam = "FIXED";
			} else{
				jsonValue_param["iso"] = "UNKNOWN";
				jsonValue_cb["param"] = jsonValue_param.toStyledString();
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "get ISO failed";
				break;
			}
			jsonValue_param["iso"] = sParam;
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16006:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("shutterSpeed"))
		{
			sParam = jsonValue_msg["param"]["shutterSpeed"].asString();
			E_DjiCameraManagerShutterSpeed shutterSpeed;
			if (0 == sParam.compare("SHUTTER_SPEED_1_8000") ) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_6400")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6400;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_6000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_5000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_4000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_3200")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3200;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_3000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_2500")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2500;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_2000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1600")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1600;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1500")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1500;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1250")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1250;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1000")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1000;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_800")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_800;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_725")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_725;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_640")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_640;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_500")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_500;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_400")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_400;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_350")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_350;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_320")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_320;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_250")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_250;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_240")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_240;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_200")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_200;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_180")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_180;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_160")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_160;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_125")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_125;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_120")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_120;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_100")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_100;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_90")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_90;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_80")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_80;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_60")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_60;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_50")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_50;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_40")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_40;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_30")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_30;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_25")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_25;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_20")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_20;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_15")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_15;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_12DOT5")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_12DOT5;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_10")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_10;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_8")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_8;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_6DOT25")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_6DOT25;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_5")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_5;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_4")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_4;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_3")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_3;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_2DOT5")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2DOT5;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_2")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_2;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1DOT67")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT67;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1_1DOT25")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_1DOT25;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1DOT3")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT3;
			} else if (0 == sParam.compare("SHUTTER_SPEED_1DOT6")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_1DOT6;
			} else if (0 == sParam.compare("SHUTTER_SPEED_2")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_2;
			} else if (0 == sParam.compare("SHUTTER_SPEED_2DOT5")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_2DOT5;
			} else if (0 == sParam.compare("SHUTTER_SPEED_3")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_3;
			} else if (0 == sParam.compare("SHUTTER_SPEED_3DOT2")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_3DOT2;
			} else if (0 == sParam.compare("SHUTTER_SPEED_4")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_4;
			} else if (0 == sParam.compare("SHUTTER_SPEED_5")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_5;
			} else if (0 == sParam.compare("SHUTTER_SPEED_6")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_6;
			} else if (0 == sParam.compare("SHUTTER_SPEED_7")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_7;
			} else if (0 == sParam.compare("SHUTTER_SPEED_8")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_8;
			} else if (0 == sParam.compare("SHUTTER_SPEED_9")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_9;
			} else if (0 == sParam.compare("SHUTTER_SPEED_10")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_10;
			} else if (0 == sParam.compare("SHUTTER_SPEED_13")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_13;
			} else if (0 == sParam.compare("SHUTTER_SPEED_15")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_15;
			} else if (0 == sParam.compare("SHUTTER_SPEED_20")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_20;
			} else if (0 == sParam.compare("SHUTTER_SPEED_25")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_25;
			} else if (0 == sParam.compare("SHUTTER_SPEED_30")) {
				shutterSpeed = E_DjiCameraManagerShutterSpeed::DJI_CAMERA_MANAGER_SHUTTER_SPEED_30;
			} else { //UNKNOWN
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
				break;
			}
			returnCode = DjiCameraManager_SetShutterSpeed(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, shutterSpeed);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("Set mounted position %d camera's shutter speed %d failed, "
							"error code: 0x%08X.", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, shutterSpeed, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set shutter speed failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param shutterSpeed";
		}
		break;
	case 16007:
		E_DjiCameraManagerShutterSpeed shutterSpeedTemp;
		T_DjiReturnCode returnCode = DjiCameraManager_GetShutterSpeed(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &shutterSpeedTemp);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d camera's shutter speed failed, "
						"error code: 0x%08X.", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["shutterSpeed"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get shutter speed failed";
		}
		else
		{
			const char *shutterSpeed_names[] = 
			{"SHUTTER_SPEED_1_8000", //= 0,    /*!< 1/8000 s */
			"SHUTTER_SPEED_1_6400", //= 1,     /*!< 1/6400 s */
			"SHUTTER_SPEED_1_6000", //= 2,     /*!< 1/6000 s */
			"SHUTTER_SPEED_1_5000", //= 3,     /*!< 1/5000 s */
			"SHUTTER_SPEED_1_4000", //= 4,     /*!< 1/4000 s */
			"SHUTTER_SPEED_1_3200", //= 5,     /*!< 1/3200 s */
			"SHUTTER_SPEED_1_3000", //= 6,     /*!< 1/3000 s */
			"SHUTTER_SPEED_1_2500", //= 7,     /*!< 1/2500 s */
			"SHUTTER_SPEED_1_2000", //= 8,     /*!< 1/2000 s */
			"SHUTTER_SPEED_1_1600", //= 9,     /*!< 1/1600 s */
			"SHUTTER_SPEED_1_1500", //= 10,    /*!< 1/1500 s */
			"SHUTTER_SPEED_1_1250", //= 11,    /*!< 1/1250 s */
			"SHUTTER_SPEED_1_1000", //= 12,    /*!< 1/1000 s */
			"SHUTTER_SPEED_1_800", //= 13,     /*!< 1/800 s */
			"SHUTTER_SPEED_1_725", //= 14,     /*!< 1/725 s */
			"SHUTTER_SPEED_1_640", //= 15,     /*!< 1/640 s */
			"SHUTTER_SPEED_1_500", //= 16,     /*!< 1/500 s */
			"SHUTTER_SPEED_1_400", //= 17,     /*!< 1/400 s */
			"SHUTTER_SPEED_1_350", //= 18,     /*!< 1/350 s */
			"SHUTTER_SPEED_1_320", //= 19,     /*!< 1/320 s */
			"SHUTTER_SPEED_1_250", //= 20,     /*!< 1/250 s */
			"SHUTTER_SPEED_1_240", //= 21,     /*!< 1/240 s */
			"SHUTTER_SPEED_1_200", //= 22,     /*!< 1/200 s */
			"SHUTTER_SPEED_1_180", //= 23,     /*!< 1/180 s */
			"SHUTTER_SPEED_1_160", //= 24,     /*!< 1/160 s */
			"SHUTTER_SPEED_1_125", //= 25,     /*!< 1/125 s */
			"SHUTTER_SPEED_1_120", //= 26,     /*!< 1/120 s */
			"SHUTTER_SPEED_1_100", //= 27,     /*!< 1/100 s */
			"SHUTTER_SPEED_1_90", //= 28,      /*!< 1/90 s */
			"SHUTTER_SPEED_1_80", //= 29,      /*!< 1/80 s */
			"SHUTTER_SPEED_1_60", //= 30,      /*!< 1/60 s */
			"SHUTTER_SPEED_1_50", //= 31,      /*!< 1/50 s */
			"SHUTTER_SPEED_1_40", //= 32,      /*!< 1/40 s */
			"SHUTTER_SPEED_1_30", //= 33,      /*!< 1/30 s */
			"SHUTTER_SPEED_1_25", //= 34,      /*!< 1/25 s */
			"SHUTTER_SPEED_1_20", //= 35,      /*!< 1/20 s */
			"SHUTTER_SPEED_1_15", //= 36,      /*!< 1/15 s */
			"SHUTTER_SPEED_1_12DOT5", //= 37,  /*!< 1/12.5 s */
			"SHUTTER_SPEED_1_10", //= 38,      /*!< 1/10 s */
			"SHUTTER_SPEED_1_8", //= 39,       /*!< 1/8 s */
			"SHUTTER_SPEED_1_6DOT25", //= 40,  /*!< 1/6.25 s */
			"SHUTTER_SPEED_1_5", //= 41,       /*!< 1/5 s */
			"SHUTTER_SPEED_1_4", //= 42,       /*!< 1/4 s */
			"SHUTTER_SPEED_1_3", //= 43,       /*!< 1/3 s */
			"SHUTTER_SPEED_1_2DOT5", //= 44,   /*!< 1/2.5 s */
			"SHUTTER_SPEED_1_2", //= 45,       /*!< 1/2 s */
			"SHUTTER_SPEED_1_1DOT67", //= 46,  /*!< 1/1.67 s */
			"SHUTTER_SPEED_1_1DOT25", //= 47,  /*!< 1/1.25 s */
			"SHUTTER_SPEED_1", //= 48,         /*!< 1.0 s */
			"SHUTTER_SPEED_1DOT3", //= 49,     /*!< 1.3 s */
			"SHUTTER_SPEED_1DOT6", //= 50,     /*!< 1.6 s */
			"SHUTTER_SPEED_2", //= 51,         /*!< 2.0 s */
			"SHUTTER_SPEED_2DOT5", //= 52,     /*!< 2.5 s */
			"SHUTTER_SPEED_3", //= 53,         /*!< 3.0 s */
			"SHUTTER_SPEED_3DOT2", //= 54,     /*!< 3.2 s */
			"SHUTTER_SPEED_4", //= 55,         /*!< 4.0 s */
			"SHUTTER_SPEED_5", //= 56,         /*!< 5.0 s */
			"SHUTTER_SPEED_6", //= 57,         /*!< 6.0 s */
			"SHUTTER_SPEED_7", //= 58,         /*!< 7.0 s */
			"SHUTTER_SPEED_8", //= 59,         /*!< 8.0 s */
			"SHUTTER_SPEED_9", //= 60,         /*!< 9.0 s */
			"SHUTTER_SPEED_10", //= 61,        /*!< 10.0 s */
			"SHUTTER_SPEED_13", //= 62,        /*!< 13.0 s */
			"SHUTTER_SPEED_15", //= 63,        /*!< 15.0 s */
			"SHUTTER_SPEED_20", //= 64,        /*!< 20.0 s */
			"SHUTTER_SPEED_25", //= 65,        /*!< 25.0 s */
			"SHUTTER_SPEED_30"  //= 66,        /*!< 30.0 s */
			};
			jsonValue_param["shutterSpeed"] = (0<=shutterSpeedTemp && shutterSpeedTemp<=66) ? shutterSpeed_names[shutterSpeedTemp] : "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16008: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("meteringMode"))
		{
			sParam = jsonValue_msg["param"]["meteringMode"].asString();
			jsonValue_cb["code"] =
				callback_unknown_error; // OSDK未实现
			// osdk-core/modules/src/payload/dji_camera_module.cpp
			// 238
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param meteringMode";
		}
		break;
	case 16009: // TODO
		jsonValue_param["meteringMode"] =
			"UNKNOWN"; // OSDK未实现
		// osdk-core/modules/src/payload/dji_camera_module.cpp 238
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	case 16010:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("exposureCompensation"))
		{
			sParam = jsonValue_msg["param"]["exposureCompensation"].asString();
			E_DjiCameraManagerExposureCompensation exposureCompensation;
			if (0 == sParam.compare("N_5_0") ) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_5_0;
			} else if (0 == sParam.compare("N_4_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_7;
			} else if (0 == sParam.compare("N_4_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_3;
			} else if (0 == sParam.compare("N_4_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_4_0;
			} else if (0 == sParam.compare("N_3_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_7;
			} else if (0 == sParam.compare("N_3_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_3;
			} else if (0 == sParam.compare("N_3_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_3_0;
			} else if (0 == sParam.compare("N_2_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_7;
			} else if (0 == sParam.compare("N_2_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_3;
			} else if (0 == sParam.compare("N_2_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_2_0;
			} else if (0 == sParam.compare("N_1_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_7;
			} else if (0 == sParam.compare("N_1_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_3;
			} else if (0 == sParam.compare("N_1_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_1_0;
			} else if (0 == sParam.compare("N_0_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_7;
			} else if (0 == sParam.compare("N_0_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_3;
			} else if (0 == sParam.compare("N_0_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_N_0_0;
			} else if (0 == sParam.compare("P_0_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_0_3;
			} else if (0 == sParam.compare("P_0_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_0_7;
			} else if (0 == sParam.compare("P_1_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_0;
			} else if (0 == sParam.compare("P_1_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_3;
			} else if (0 == sParam.compare("P_1_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_1_7;
			} else if (0 == sParam.compare("P_2_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_0;
			} else if (0 == sParam.compare("P_2_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_3;
			} else if (0 == sParam.compare("P_2_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_2_7;
			} else if (0 == sParam.compare("P_3_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_0;
			} else if (0 == sParam.compare("P_3_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_3;
			} else if (0 == sParam.compare("P_3_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_3_7;
			} else if (0 == sParam.compare("P_4_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_0;
			} else if (0 == sParam.compare("P_4_3")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_3;
			} else if (0 == sParam.compare("P_4_7")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_4_7;
			} else if (0 == sParam.compare("P_5_0")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_P_5_0;
			} else if (0 == sParam.compare("FIXED")) {
				exposureCompensation = E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_FIXED;
			} else { //UNKNOWN
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set exposure compensation failed";
				break;
			}
			T_DjiReturnCode returnCode = DjiCameraManager_SetExposureCompensation(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, exposureCompensation);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("Set mounted position %d camera's exposure compensation %d failed,"
							"error code: 0x%08X.", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, exposureCompensation, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set exposure compensation failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param exposureCompensation";
		}
		break;
	case 16011:
		const char *ev_names[] = {
			"0",// set 0
			/*! The camera's exposure compensation is -5.0ev.*/
			"N_5_0",// = 1,
			/*! The camera's exposure compensation is -4.7ev.*/
			"N_4_7",// = 2,
			/*! The camera's exposure compensation is -4.3ev.*/
			"N_4_3",// = 3,
			/*! The camera's exposure compensation is -4.0ev.*/
			"N_4_0",// = 4,
			/*! The camera's exposure compensation is -3.7ev.*/
			"N_3_7",// = 5,
			/*! The camera's exposure compensation is -3.3ev.*/
			"N_3_3",// = 6,
			/*! The camera's exposure compensation is -3.0ev.*/
			"N_3_0",// = 7,
			/*! The camera's exposure compensation is -2.7ev.*/
			"N_2_7",// = 8,
			/*! The camera's exposure compensation is -2.3ev.*/
			"N_2_3",// = 9,
			/*! The camera's exposure compensation is -2.0ev.*/
			"N_2_0",// = 10,
			/*! The camera's exposure compensation is -1.7ev.*/
			"N_1_7",// = 11,
			/*! The camera's exposure compensation is -1.3ev.*/
			"N_1_3",// = 12,
			/*! The camera's exposure compensation is -1.0ev.*/
			"N_1_0",// = 13,
			/*! The camera's exposure compensation is -0.7ev.*/
			"N_0_7",// = 14,
			/*! The camera's exposure compensation is -0.3ev.*/
			"N_0_3",// = 15,
			/*! The camera's exposure compensation is 0.0ev.*/
			"N_0_0",// = 16,
			/*! The camera's exposure compensation is +0.3ev.*/
			"P_0_3",// = 17,
			/*! The camera's exposure compensation is +0.7ev.*/
			"P_0_7",// = 18,
			/*! The camera's exposure compensation is +1.0ev.*/
			"P_1_0",// = 19,
			/*! The camera's exposure compensation is +1.3ev.*/
			"P_1_3",// = 20,
			/*! The camera's exposure compensation is +1.7ev.*/
			"P_1_7",// = 21,
			/*! The camera's exposure compensation is +2.0ev.*/
			"P_2_0",// = 22,
			/*! The camera's exposure compensation is +2.3ev.*/
			"P_2_3",// = 23,
			/*! The camera's exposure compensation is +2.7ev.*/
			"P_2_7",// = 24,
			/*! The camera's exposure compensation is +3.0ev.*/
			"P_3_0",// = 25,
			/*! The camera's exposure compensation is +3.3ev.*/
			"P_3_3",// = 26,
			/*! The camera's exposure compensation is +3.7ev.*/
			"P_3_7",// = 27,
			/*! The camera's exposure compensation is +4.0ev.*/
			"P_4_0",// = 28,
			/*! The camera's exposure compensation is +4.3ev.*/
			"P_4_3",// = 29,
			/*! The camera's exposure compensation is +4.7ev.*/
			"P_4_7",// = 30,
			/*! The camera's exposure compensation is +5.0ev.*/
			"P_5_0" // = 31,
			/*! The camera's exposure compensation is fixed by the camera.*/
			//FIXED = 0xFF,
			/*! The camera's exposure compensation is unknown. */
			//UNKNOWN = 0xFFFF,
		};
		E_DjiCameraManagerExposureCompensation exposureCompensation;
		T_DjiReturnCode returnCode = DjiCameraManager_GetExposureCompensation(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &exposureCompensation);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d exposure compensation failed, error code: 0x%08X.",
						E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["exposureCompensation"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get exposure compensation failed";
			break;
		}
		if(exposureCompensation == E_DjiCameraManagerExposureCompensation::DJI_CAMERA_MANAGER_EXPOSURE_COMPENSATION_FIXED){
			sParam = "FIXED";
		} else {
			sParam = (0 < exposureCompensation && exposureCompensation < 32) ? ev_names[exposureCompensation] : "UNKNOWN";
			jsonValue_param["exposureCompensation"] = sParam;
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16012: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("AELock"))
		{
			sParam = jsonValue_msg["param"]["AELock"].asString();
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
			jsonValue_cb["code"] = callback_unknown_error;
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param AELock";
		}
		break;
	case 16013: // TODO
		jsonValue_param["AELock"] = "UNKNOWN";
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	case 16014:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("photoAEBCount"))
		{
			sParam = jsonValue_msg["param"]["photoAEBCount"].asString();
			E_DjiCameraManagerPhotoAEBCount aebCount;
			if (0 == sParam.compare("AEB_COUNT_3") ) {
				aebCount = E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_3;
			} else if (0 == sParam.compare("AEB_COUNT_5")) {
				aebCount = E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_5;
			} else if (0 == sParam.compare("AEB_COUNT_7")) {
				aebCount = E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_7;
			} else { //UNKNOWN
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set photo AEB count failed.H20 not support";
				break;
			}
			T_DjiReturnCode returnCode = DjiCameraManager_SetPhotoAEBCount(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, aebCount);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("set mounted position %d camera's AEB count(%d) failed,"
							" error code :0x%08X.", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, aebCount, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set photo AEB count failed.H20 not support";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param photoAEBCount";
		}
		break;
	case 16015:
		E_DjiCameraManagerPhotoAEBCount aebCount;
		T_DjiReturnCode returnCode = DjiCameraManager_GetPhotoAEBCount(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, aebCount);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d exposure aebCount failed, error code: 0x%08X.",
						E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["photoAEBCount"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get aebCount failed";
			break;
		}
		switch (aebCount)
		{
		case E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_3:
			sParam = "AEB_COUNT_3";
			break;
		case E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_5:
			sParam = "AEB_COUNT_5";
			break;
		case E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_7:
			sParam = "AEB_COUNT_7";
			break;
		case E_DjiCameraManagerPhotoAEBCount::DJI_CAMERA_MANAGER_PHOTO_AEB_COUNT_KNOWN:
			sParam = "AEB_COUNT_KNOWN";
			break;
		default:
			break;
		}
		jsonValue_param["photoAEBCount"] = sParam;
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["msg"] = "success";
		break;
	case 16016: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("whiteBalancePreset"))
		{
			sParam = jsonValue_msg["param"]["whiteBalancePreset"].asString();
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
			jsonValue_cb["code"] = callback_unknown_error;
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param whiteBalancePreset";
		}
		break;
	case 16017: // TODO
		jsonValue_param["whiteBalancePreset"] = "UNKNOWN";
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	case 16018: // TODO
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	case 16019: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("mode"))
		{
			sParam = jsonValue_msg["param"]["mode"].asString();
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
			jsonValue_cb["code"] = callback_unknown_error;
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param mode";
		}
		break;
	case 16020:
		T_DjiReturnCode returnCode = DjiTest_CameraManagerStartShootSinglePhoto(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "shoot photo failed";
		}
		else
		{
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16021:
		T_DjiReturnCode returnCode = DjiTest_CameraManagerStartRecordVideo(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
		{
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "start record video failed";
		}
		else
		{
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16022:
		T_DjiReturnCode returnCode = DjiTest_CameraManagerStopRecordVideo(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "stop record video failed";
		}
		else
		{
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16023:
		if (jsonValue_msg.isMember("param") &&
			(jsonValue_msg["param"].isMember("focusPointX") ||
				jsonValue_msg["param"].isMember("focusPointY")))
		{
			fx = ((jsonValue_msg["param"].isMember("focusPointX") &&
				jsonValue_msg["param"]["focusPointX"].isDouble())
				? (jsonValue_msg["param"]["focusPointX"].asDouble())
				: (0.5f));
			fy = ((jsonValue_msg["param"].isMember("focusPointY") &&
				jsonValue_msg["param"]["focusPointY"].isDouble())
				? (jsonValue_msg["param"]["focusPointY"].asDouble())
				: (0.5f));
			T_DjiCameraManagerFocusPosData focusPoint{fx, fy};
			T_DjiReturnCode returnCode = DjiTest_CameraManagerSetFocusPoint(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, focusPoint);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
			{
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set focus point failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] =
				"param error without param focusPointX or focusPointY";
		}
		break;
		// TODO get focus point
	case 16024:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("shootPhotoMode"))
		{
			sParam = jsonValue_msg["param"]["shootPhotoMode"].asString();
			E_DjiCameraManagerShootPhotoMode shootPhotoMode;
			if (0 == sParam.compare("SINGLE") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE;
			} else if (0 == sParam.compare("HDR") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_HDR;
			} else if (0 == sParam.compare("BURST") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST;
			} else if (0 == sParam.compare("AEB") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB;
			} else if (0 == sParam.compare("INTERVAL") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL;
			} else if (0 == sParam.compare("RAW_BURST") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_RAW_BURST;
			} else if (0 == sParam.compare("REGIONAL_SR") ) {
				shootPhotoMode = E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_REGIONAL_SR;
			} else { //UNKNOWN
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set shoot photo failed";
				break;
			}
			returnCode = DjiCameraManager_SetShootPhotoMode(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				USER_LOG_ERROR("set mounted position %d camera's shoot photo mode as single-photo mode failed,"
							" error code :0x%08X", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set shoot photo failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param shootPhotoMode";
		}
		break;
	case 16025:
		const char *shootPhotoMode_names[] =
		{   "0",     //   DJI_CAMERA_TAKE_PHOTO_TYPE_STOP    0
			"SINGLE",// = DJI_CAMERA_TAKE_PHOTO_TYPE_NORMAL, 1
			"HDR",   // = DJI_CAMERA_TAKE_PHOTO_TYPE_HDR,    2
			"3",     //   DJI_CAMERA_TAKE_PHOTO_TYPE_BOKEH   3
			"BURST", // = DJI_CAMERA_TAKE_PHOTO_TYPE_BURST,  4
			"AEB",   // = DJI_CAMERA_TAKE_PHOTO_TYPE_AEB,    5
			"INTERVAL"//= DJI_CAMERA_TAKE_PHOTO_TYPE_TIME_LAPSE, 6
			"7",     //   DJI_CAMERA_TAKE_PHOTO_TYPE_PANO_APP    7
			"8",     //   DJI_CAMERA_TAKE_PHOTO_TYPE_TRACKING    8
			"RAW_BURST"//=DJI_CAMERA_TAKE_PHOTO_TYPE_RAW_BURST,  9
			"EHDR"   // = DJI_CAMERA_TAKE_PHOTO_TYPE_EHDR,      10
			"REGIONAL_SR",// = 0x16,
		};
		E_DjiCameraManagerShootPhotoMode shootPhotoMode;
		T_DjiReturnCode returnCode = DjiCameraManager_GetShootPhotoMode(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &shootPhotoMode);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("set mounted position %d camera's shoot photo mode as single-photo mode failed,"
						" error code :0x%08X", E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["shootPhotoMode"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "getting shoot photo mode failed";
			break;
		}
		if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE){
			sParam = "SINGLE";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_HDR){
			sParam = "HDR";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_BURST){
			sParam = "BURST";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_AEB){
			sParam = "AEB";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_INTERVAL){
			sParam = "INTERVAL";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_RAW_BURST){
			sParam = "RAW_BURST";
		} else if(shootPhotoMode == E_DjiCameraManagerShootPhotoMode::DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_REGIONAL_SR){
			sParam = "REGIONAL_SR";
		} else { // unknown
			sParam = "UNKNOWN";
		}
		
		jsonValue_param["shootPhotoMode"] = sParam;
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["code"] = callback_success_code;
		jsonValue_cb["msg"] = "success";
		
		break;
	case 16026: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("panoramaMode"))
		{
			sParam = jsonValue_msg["param"]["panoramaMode"].asString();
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
			jsonValue_cb["code"] = callback_unknown_error;
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param panoramaMode";
		}
		break;
	case 16027: // TODO
		jsonValue_param["panoramaMode"] = "UNKNOWN";
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	case 16028:
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("hybridZoomKey") &&
			(jsonValue_msg["param"]["hybridZoomKey"].isInt() ||
				jsonValue_msg["param"]["hybridZoomKey"].isDouble()))
		{
			// TODO{zengxw} Check: change camera source to zoom source
			USER_LOG_INFO("Set camera stream source to zoom camera.");
			T_DjiReturnCode returnCode = DjiCameraManager_SetStreamSource(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, E_DjiCameraManagerStreamSource::DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM);
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				std::cout << "Failed to change zoom camera source!" << std::endl;
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set hybridZoomKey failed";
				break;
			}
			i_CAMERA_MODE = 2; // ZOOM
			fParam = jsonValue_msg["param"]["hybridZoomKey"].asDouble();
			// TODO{zengxw} zoom direction is not support in osdk.
			T_DjiReturnCode returnCode = DjiTest_CameraManagerOpticalZoom(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, E_DjiCameraZoomDirection::DJI_CAMERA_ZOOM_DIRECTION_IN, fParam);
			if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
				returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
				jsonValue_cb["code"] = callback_unknown_error;
				jsonValue_cb["msg"] = "set hybridZoomKey failed";
			}
			else
			{
				jsonValue_cb["code"] = callback_success_code;
				jsonValue_cb["msg"] = "success";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param hybridZoomKey";
		}
		break;
	case 16029:
		T_DjiReturnCode returnCode;
		T_DjiCameraManagerOpticalZoomParam opticalZoomParam;
		returnCode = DjiCameraManager_GetOpticalZoomParam(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, &opticalZoomParam);
		if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
			returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
			USER_LOG_ERROR("Get mounted position %d camera's zoom param failed, error code :0x%08X",
						E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, returnCode);
			jsonValue_param["hybridZoomKey"] = "UNKNOWN";
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "get Zoom failed";
		}
		else
		{
			jsonValue_param["hybridZoomKey"] = opticalZoomParam.currentOpticalZoomFactor;
			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		break;
	case 16030: // TODO
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("msxLevel"))
		{
			sParam = jsonValue_msg["param"]["msxLevel"].asString();
			jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
			jsonValue_cb["code"] = callback_unknown_error;
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param msxLevel";
		}
		break;
	case 16031: // TODO
		jsonValue_param["msxLevel"] = "UNKNOWN";
		jsonValue_cb["param"] = jsonValue_param.toStyledString();
		jsonValue_cb["msg"] = "DJI OSDK Unrealized.";
		jsonValue_cb["code"] = callback_unknown_error;
		break;
	default:
		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
		break;
	}
}


void MQTT_Gen()
{
	switch (iCode)
	{
	case 13008:
		if (!bool_RTMPFlag)
		{
			bool_RTMPFlag = true;
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "Push streaming has started";
		}
		break;
	case 13009:
		if (bool_RTMPFlag)
		{
			bool_RTMPFlag = false;
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{

			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "Push streaming has stopped";
		}
		break;

	case 314002:
		if (bool_RTMPFlag)
		{
			if (jsonValue_msg.isMember("firmwareType"))
			{
				jsonValue_param["version"] = "PSDK V3.8"; // TODO{zengxw} what?
				jsonValue_param["firmwareType"] = jsonValue_msg["firmwareType"].asInt();
			}

			jsonValue_cb["param"] = jsonValue_param.toStyledString();
			jsonValue_cb["code"] = callback_success_code;
			jsonValue_cb["msg"] = "success";
		}
		else
		{
			jsonValue_cb["code"] = callback_unknown_error;
			jsonValue_cb["msg"] = "Push streaming has stopped";
		}
		break;
	default:
		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
		break;
	}
}


void MQTT_V3()
{
	switch (iCode)
	{
	case 313022:
	{
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("cameraMode") &&
			jsonValue_msg["param"]["cameraMode"].isInt())
		{
			iParam = jsonValue_msg["param"]["cameraMode"].asInt();
			if (iParam == 2 || iParam == 1 || iParam == 0)
			{
				E_DjiCameraManagerStreamSource source; // TODO{zengxw} or DjiLiveview_StartH264Stream(E_DjiLiveViewCameraSource::DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE)？
				if (iParam == 0)
					source = E_DjiCameraManagerStreamSource::DJI_CAMERA_MANAGER_SOURCE_WIDE_CAM;
				else if (iParam == 1)
					source = E_DjiCameraManagerStreamSource::DJI_CAMERA_MANAGER_SOURCE_IR_CAM;
				else if (iParam == 2)
					source = E_DjiCameraManagerStreamSource::DJI_CAMERA_MANAGER_SOURCE_ZOOM_CAM;

				T_DjiReturnCode returnCode = DjiCameraManager_SetStreamSource(E_DjiMountPosition::DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, source);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND) {
					jsonValue_cb["code"] = callback_aircraft_error;
					jsonValue_cb["msg"] = "change camera mode failed";
				}
				else
				{
					if (iParam == 0)
						i_CAMERA_MODE = 0; // WIDE
					else if (iParam == 1)
						i_CAMERA_MODE = 1; // IR or IR+ZOOM;decided by Remote Control
					else if (iParam == 2)
						i_CAMERA_MODE = 2; // ZOOM
					jsonValue_cb["code"] = callback_success_code;
					jsonValue_cb["msg"] = "success";
				}
			}
			else
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error with param cameraMode outrange";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param cameraMode";
		}
	}
	break;

	case 313023:
	{
		if (jsonValue_msg.isMember("param") &&
			jsonValue_msg["param"].isMember("avoidSwitch") &&
			jsonValue_msg["param"]["avoidSwitch"].isInt())
		{
			iParam = jsonValue_msg["param"]["avoidSwitch"].asInt();
			if (iParam == 1 || iParam == 0)
			{
				E_DjiFlightControllerObstacleAvoidanceEnableStatus avoid_status = iParam ? E_DjiFlightControllerObstacleAvoidanceEnableStatus::DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE : E_DjiFlightControllerObstacleAvoidanceEnableStatus::DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE;

				// TODO{zengxw} osdk collision avoidance setting doesn't define the direction.
				bool all_direction_set = true;
				T_DjiReturnCode returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(avoid_status);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set HorizontalVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    all_direction_set = false;
                }
				returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(avoid_status);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set UpwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    all_direction_set = false;
                }
				returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(avoid_status);
				if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                    USER_LOG_ERROR("Set DownwardsVisualObstacleAvoidanceEnable failed, errno = 0x%08llX", returnCode);
                    all_direction_set = false;
                }

				if (!all_direction_set)
				{
					jsonValue_cb["code"] = callback_aircraft_error;
					jsonValue_cb["msg"] = "set avoid enable failed";
				}
				else
				{
					jsonValue_cb["code"] = callback_success_code;
					jsonValue_cb["msg"] = "success";
				}
			}
			else
			{
				jsonValue_cb["code"] = callback_param_error;
				jsonValue_cb["msg"] = "param error with param avoidSwitch outrange";
			}
		}
		else
		{
			jsonValue_cb["code"] = callback_param_error;
			jsonValue_cb["msg"] = "param error without param avoidSwitch";
		}
	}
	break;
	default:
	{
		jsonValue_cb["code"] = (int)9002;
		jsonValue_cb["msg"] = "code undefined";
	}
	break;
	}
}
