#include "camera_stream_inference_rtmp.h"

#include <sys/time.h>

#include <iostream>

#include "liveview/test_liveview.hpp"
#include "dji_error.h"
#include "taskmanage.h"

#ifdef OPEN_CV_INSTALLED
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
#endif

extern std::string s_RTMP_URI;
extern std::string s_CLIENT_ID;
extern bool bool_RTMPFlag;

char mainName[] = "MAIN_CAM";
void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void* userData) {
  if (!bool_RTMPFlag) {
    sleep(1);
  } else {
    std::string name = std::string(reinterpret_cast<char*>(userData));

#ifdef OPEN_CV_INSTALLED
    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(),
                img.width * 3);
    cvtColor(mat, mat, COLOR_RGB2BGR);
    imshow(name, mat);
    cv::waitKey(30);
#endif
  }
}

void* MyRtmp_Task(void* arg) {
  std::string strModelPath;
  strModelPath = "/home/rer/SdkDemo/modelV3.2/";

  BambHandle* handle;
  bamb_load_manager(strModelPath, &handle);

  LiveviewSample* liveViewSample;
  try {
    liveViewSample = new LiveviewSample();
  } catch (...) {
    return;
  }
  T_DjiReturnCode CamResult = liveViewSample->StartMainCameraStream(
      &DjiUser_ShowRgbImageCallback, &mainName);
  if (CamResult != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    std::cout << "Failed to open Camera" << std::endl;
    return NULL;
  }

  while (1) {
    sleep(10);
  }

  T_DjiReturnCode result = liveviewSample->StopMainCameraStream();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    std::cout << "Failed to stop Camera" << std::endl;
    delete liveviewSample;
    return NULL;
  }
  delete liveviewSample;

  return NULL;
}
