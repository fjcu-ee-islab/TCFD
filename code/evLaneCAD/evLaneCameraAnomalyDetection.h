#ifndef _EVCAMERAANOMALYDETECTION_H_
#define _EVCAMERAANOMALYDETECTION_H_

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifndef EV_CAD_SINGLETEST
#define EV_CAD_SINGLETEST 1
#endif

#ifndef EV_CAD_FROMYML
#define EV_CAD_FROMYML 4
#endif

#ifndef EV_CAD_FROMARGV
#define EV_CAD_FROMARGV 16
#endif

#ifndef EV_CAD_SHOWIMAGE
#define EV_CAD_SHOWIMAGE 1
#endif

#include "evkcore.h"
#include "evLaneOnlineCAD.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cv.h>
#include <highgui.h>

//ConfigData
typedef struct EvConfigData
{
	char strFolderRootPath[1024];	  //log根目錄路徑
	FILE* fConfig;					  //config.yml檔
	EvOnlineCADParam sOnlineCADParam; //evCameraAnomalyDetectionMisc判斷相關
}EvConfigData;

//Param
typedef struct _EvCameraAnomalyDetectionParam
{
	int iArgc;
	int iCADParamNum;
	int iOnlineCADParamNum;

	EvConfigData sConfigData; //設定檔相關

	/*evLaneCameraAnomalyDetection判斷相關*/
	EvOnlineCADParam sOnlineCADParam; 

}EvCameraAnomalyDetectionParam;

//model
typedef struct _EvCameraAnomalyDetection
{
	EvCameraAnomalyDetectionParam sCADParam;

	char strConfigName[256];	//config檔名
	EvConfigData* sConfigData;  //設定檔相關
	EvOnlineCAD* sOnlineCAD;	//特徵擷取相關
	int iCond;
	int iCondKeep;
	
}EvCameraAnomalyDetection;


int evSetCameraAnomalyDetection(EvCameraAnomalyDetection* sCAD, EvCameraAnomalyDetectionParam* sCADParam,int ievCADParamNum);
EvCameraAnomalyDetection* evCreateCameraAnomalyDetection(IplImage* imgSrc, EvCameraAnomalyDetectionParam* sCADParam);
int evDoCameraAnomalyDetection(IplImage* imgSrc, EvCameraAnomalyDetection* sCAD, IplImage* imgSR = NULL);
void evReleaseCameraAnomalyDetection(EvCameraAnomalyDetection* sCAD);
//void evConfigRead( EvConfigData* sConfigData, char* config_name = EV_DEFAULT("config.yml"));

#endif //_EVCAMERAANOMALYDETECTION_H_