#ifndef TASKMANAGE_H
#define TASKMANAGE_H

//#include "bambmana.h"
#include "iostream"
#include "fstream"
#include "ostream"
#include <opencv2/opencv.hpp>

//#include "rapidjson/document.h"
//#include "rapidjson/writer.h"
//#include "rapidjson/stringbuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Bamb_MALLOC(T, size) (T*)malloc(sizeof(T) * (size))
#define Bamb_FREE(ptr) { if(ptr != NULL) free(ptr); ptr = NULL; }
#define person_thr 0.5
#define person_ret 0.3
#define sign_thr 0.3
#define other_thr 0.3

    typedef void BambHandle;

    typedef struct CdResultFly
    {
        cv::Rect st_CrfRtObj;		//����Ŀ���
        cv::Rect st_CrfRtHead;		//ͷ��Ŀ���
        cv::Rect st_CrfRtPerson;	//����Ŀ���

        cv::Rect st_CrfRtSign;		//��ʶ��Ŀ���
        cv::Rect st_CrfRtOther;		//���ֻ�Ŀ���
        cv::Rect st_CrfRtOther2;	//����Ŀ���

        int st_CrfIntHelmet;		//��ȫñʶ����
        int st_CrfIntCloth;			//������ʶ����
        int st_CrfIntSign;			//��ʶ��ʶ����
        int st_CrfIntOther;			//���ֻ�������ʶ����

        CdResultFly()
        {
            st_CrfIntHelmet = 0;
            st_CrfIntCloth = 0;
            st_CrfIntSign = 0;
            st_CrfIntOther = 0;
        }
    }CdResultFly;
    //typedef std::vector<CdResultFly> CdResultVecFly;

    typedef struct CdTaskRet
    {
        bool st_bCtrSafety;
        bool st_bCtrViolation;
        bool st_bCtrSignboard;

        CdTaskRet()
        {
            st_bCtrSafety = false;
            st_bCtrViolation = false;
            st_bCtrSignboard = false;
        }
    }CdTaskRet;

    int bamb_load_manager(std::string c_strModelPath, BambHandle** rhandle);
    //void* bamb_load_manager(std::string c_strModelPath, BambHandle **rhandle);

    std::vector<CdResultFly> algFlyRet(BambHandle* ptr, cv::Mat c_matImg, CdTaskRet c_stTask);

    void bamb_release_manager(BambHandle** ptr);

#ifdef __cplusplus
};
#endif

#endif // TASKMANAGE_H