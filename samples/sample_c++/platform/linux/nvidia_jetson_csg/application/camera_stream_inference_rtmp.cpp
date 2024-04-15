#include "camera_stream_inference_rtmp.h"

#include <iostream>
#include <unistd.h>

#include "liveview/test_liveview.hpp"
#include "dji_error.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"

extern std::string s_RTMP_URI;
extern std::string s_CLIENT_ID;
extern bool bool_RTMPFlag;

char mainName[] = "MAIN_CAM";

cv::VideoWriter writer;

void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void* userData) {
  if (!bool_RTMPFlag) {
    sleep(1);
  } else {
    std::string name = std::string(reinterpret_cast<char*>(userData));

    cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(),
                img.width * 3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

    cv::imshow(name, mat);
    writer << mat;
    cv::waitKey(30);
  }
}

void* MyRtmp_Task(void* arg) {
  // std::string strModelPath;
  // strModelPath = "/home/rer/SdkDemo/modelV3.2/";
  // BambHandle* handle;
  // bamb_load_manager(strModelPath, &handle);

  LiveviewSample* liveViewSample;
  try {
    liveViewSample = new LiveviewSample();
  } catch (...) {
    return NULL;
  }

  std::string sPipeline = "appsrc ! videoconvert ! omxh264enc bitrate=8000000 ! flvmux ! rtmpsink location=";
  sPipeline = sPipeline + s_RTMP_URI + "/live" + s_CLIENT_ID + "_flight sync=false";
  writer.open(sPipeline, 0, 30/*videoFps*/, cv::Size(1920, 1440), true);
  if (!writer.isOpened()) {
      printf("Can't Create video writer.\n");
      return NULL;
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

  T_DjiReturnCode result = liveViewSample->StopMainCameraStream();
  if (result != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    std::cout << "Failed to stop Camera" << std::endl;
    delete liveViewSample;
    return NULL;
  }
  delete liveViewSample;

  return NULL;
}
