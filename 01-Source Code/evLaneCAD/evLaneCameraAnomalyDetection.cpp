#include "evLaneCameraAnomalyDetection.h"

EvConfigData* ievCreateConfigData(void);
void ievWriteConfigDefault(char* config_name ,char FolderRootPath[]);
void ievWriteConfig(EvConfigData* sConfigData, char* config_name, char FolderRootPath[]);

EvCameraAnomalyDetectionParam ievSetCAD1();
EvCameraAnomalyDetectionParam ievSetCAD2();
EvCameraAnomalyDetectionParam ievSetCAD3();
EvCameraAnomalyDetectionParam ievSetCAD4();
EvCameraAnomalyDetectionParam ievSetCAD5();
EvCameraAnomalyDetectionParam ievSetCAD6();
EvCameraAnomalyDetectionParam ievSetCAD7();
EvCameraAnomalyDetectionParam ievSetCAD8();
EvCameraAnomalyDetectionParam ievSetCAD9();
EvCameraAnomalyDetectionParam ievSetCAD10();
EvCameraAnomalyDetectionParam ievSetCAD999();

int evSetCameraAnomalyDetection(EvCameraAnomalyDetection* sCAD, EvCameraAnomalyDetectionParam* sCADParam,int iNumber)
{
	FILE* fConfig;

	//若無config.yml檔 (使用者自定檔案)，則會產生預設檔案
	//if( (fConfig = fopen("config.yml","r")) == NULL )
	//	ievWriteConfigDefault("config.yml", "Log");
	//else
	//	fclose(fConfig); //fopen在此僅檢查有無檔案，故馬上fclose

	switch(iNumber)
	{
	case 1:
		*sCADParam=ievSetCAD1();break;
	case 2:
		*sCADParam=ievSetCAD2();break;
	case 3:
		*sCADParam=ievSetCAD3();break;
	case 4:
		*sCADParam=ievSetCAD4();break;
	case 5:
		*sCADParam=ievSetCAD5();break;
	case 6:
		*sCADParam=ievSetCAD6();break;
	case 7:
		*sCADParam=ievSetCAD7();break;
	case 8:
		*sCADParam=ievSetCAD8();break;
	case 9:
		*sCADParam=ievSetCAD9();break;
	case 10:
		*sCADParam=ievSetCAD10();break;
	case 999:
		*sCADParam=ievSetCAD999();break;
	case 0:default:
		*sCADParam=ievSetCAD7();
	}

	//模組未配置就對param設定，否則直接設定模組內的param
	if(sCAD != NULL)
		sCAD->sCADParam = *sCADParam;

	return 0;
}

EvConfigData* ievCreateConfigData(void)
{	
	EvConfigData *sConfigData;
	sConfigData = (EvConfigData*)cvAlloc(sizeof(*sConfigData));
	return (EvConfigData*)sConfigData;
}

EvCameraAnomalyDetection* evCreateCameraAnomalyDetection(IplImage* imgSrc, EvCameraAnomalyDetectionParam* sCADParam)
{	
	EvCameraAnomalyDetection *sCAD;
	sCAD = (EvCameraAnomalyDetection*)cvAlloc(sizeof(*sCAD));

	//配置各模組記憶體
	sCAD->sOnlineCAD = evCreateOnlineCAD(imgSrc, &sCADParam->sOnlineCADParam);

	sCAD->iCond = 0;
	sCAD->iCondKeep = 0;

	return (EvCameraAnomalyDetection*)sCAD;
}

int evDoCameraAnomalyDetection(IplImage* imgSrc,EvCameraAnomalyDetection* sCAD,IplImage* imgSR)
{
	/*iCond = 0表示無事件 1表示注意 2或以上表示有事件發生*/
	sCAD->iCond = evDoOnlineCAD( imgSrc, sCAD->sOnlineCAD ); 

	/*iCondKeep 發生警報後保持警報用*/
	if ( sCAD->iCondKeep < sCAD->iCond )
		sCAD->iCondKeep = sCAD->iCond;

	return sCAD->iCond;
}

//釋放記憶體
void evReleaseCameraAnomalyDetection(EvCameraAnomalyDetection* sCAD)
{
	if(sCAD)
	{
		//釋放各模組記憶體
		evReleaseOnlineCAD(sCAD->sOnlineCAD);
		cvFree(&sCAD);
	}
}

//設定參數組1
EvCameraAnomalyDetectionParam ievSetCAD1()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 1;
	sParamOut.iOnlineCADParamNum = 1;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 1);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組2
EvCameraAnomalyDetectionParam ievSetCAD2()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 2;
	sParamOut.iOnlineCADParamNum = 2;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 2);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組3
EvCameraAnomalyDetectionParam ievSetCAD3()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 3;
	sParamOut.iOnlineCADParamNum = 3;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 3);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組4
EvCameraAnomalyDetectionParam ievSetCAD4()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 4;
	sParamOut.iOnlineCADParamNum = 4;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 4);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組5
EvCameraAnomalyDetectionParam ievSetCAD5()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 5;
	sParamOut.iOnlineCADParamNum = 5;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 5);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組6
EvCameraAnomalyDetectionParam ievSetCAD6()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 6;
	sParamOut.iOnlineCADParamNum = 6;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 6);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組7
EvCameraAnomalyDetectionParam ievSetCAD7()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 7;
	sParamOut.iOnlineCADParamNum = 7;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 7);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組8
EvCameraAnomalyDetectionParam ievSetCAD8()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 8;
	sParamOut.iOnlineCADParamNum = 8;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 8);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組9
EvCameraAnomalyDetectionParam ievSetCAD9()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 9;
	sParamOut.iOnlineCADParamNum = 9;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 9);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組10
EvCameraAnomalyDetectionParam ievSetCAD10()
{
	EvCameraAnomalyDetectionParam sParamOut;
	sParamOut.iCADParamNum = 10;
	sParamOut.iOnlineCADParamNum = 10;

	evSetOnlineCAD( NULL , &sParamOut.sConfigData.sOnlineCADParam, 10);
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//設定參數組999
EvCameraAnomalyDetectionParam ievSetCAD999()
{
	EvCameraAnomalyDetectionParam sParamOut;

	sParamOut.iCADParamNum = EV_PARAM_BY_USER;
	sParamOut.iOnlineCADParamNum = EV_PARAM_BY_USER;

	//evConfigRead( &sParamOut.sConfigData );

	sParamOut.sConfigData.sOnlineCADParam.ievOnlineCADParamNum = EV_PARAM_BY_USER;
	sParamOut.sOnlineCADParam = sParamOut.sConfigData.sOnlineCADParam;

	return (EvCameraAnomalyDetectionParam)sParamOut;
}

//讀取config.yml設定值
void evConfigRead( EvConfigData* sConfigData, char* config_name)
{
	CvMemStorage* memstorage = cvCreateMemStorage(0);
	CvFileStorage* fs_read_yml = cvOpenFileStorage( config_name, memstorage, CV_STORAGE_READ);

	//Log資料夾目錄名
	strcpy( sConfigData->strFolderRootPath, cvReadStringByName( fs_read_yml, NULL, "FolderRootPath"));

	sConfigData->sOnlineCADParam.iCountLimit			= cvReadIntByName ( fs_read_yml, NULL, "CountLimit");
	sConfigData->sOnlineCADParam.dSdvThres				= cvReadRealByName( fs_read_yml, NULL, "SdvThreshold");
	sConfigData->sOnlineCADParam.dEdgeTres				= cvReadRealByName( fs_read_yml, NULL, "EdgeThreshold");
	sConfigData->sOnlineCADParam.dLocalSdvThres			= cvReadRealByName( fs_read_yml, NULL, "LoaclSdvThreshold");
	sConfigData->sOnlineCADParam.dProcessNoiseSigma		= cvReadRealByName( fs_read_yml, NULL, "KFProcessNoiseSigma");
	sConfigData->sOnlineCADParam.dMeasurementNoiseSigma	= cvReadRealByName( fs_read_yml, NULL, "KFMeasurementNoiseSigma");
	sConfigData->sOnlineCADParam.iCornerMaxNumber	= cvReadRealByName( fs_read_yml, NULL, "iCornerMaxNumber");
	sConfigData->sOnlineCADParam.dMinDistance	= cvReadRealByName( fs_read_yml, NULL, "dMinDistance");
	sConfigData->sOnlineCADParam.dQualityLevel	= cvReadRealByName( fs_read_yml, NULL, "dQualityLevel");
	sConfigData->sOnlineCADParam.iLevel							= cvReadRealByName( fs_read_yml, NULL, "iLevel");
	sConfigData->sOnlineCADParam.dOpticalFlowLengthThreshold	= cvReadRealByName( fs_read_yml, NULL, "dOpticalFlowLengthThreshold");

	int iTemp;
	iTemp = cvReadRealByName( fs_read_yml, NULL, "sizeWinLK");
	sConfigData->sOnlineCADParam.sizeWinLK = cvSize(iTemp, iTemp);

	cvReleaseFileStorage(&fs_read_yml);	
	cvReleaseMemStorage(&memstorage);
}
//產生預設之config.yml
void ievWriteConfigDefault(char* config_name, char FolderRootPath[])
{
	CvMemStorage* memstorage = cvCreateMemStorage(0);
	CvFileStorage* fs_write_yml = cvOpenFileStorage( config_name, memstorage, CV_STORAGE_WRITE);

	cvWriteComment( fs_write_yml, "Config Table", 1);
	cvWriteString( fs_write_yml, "FolderRootPath", FolderRootPath);

	cvWriteComment( fs_write_yml, "SetJudgement", 0);
	cvWriteInt ( fs_write_yml, "CountLimit",				45);
	cvWriteReal( fs_write_yml, "SdvThreshold",				0.1);	
	cvWriteReal( fs_write_yml, "EdgeThreshold",				0.01);	
	cvWriteReal( fs_write_yml, "LoaclSdvThreshold",			0.03);	
	cvWriteReal( fs_write_yml, "KFProcessNoiseSigma",		0.0001);	//KF = Kalman filter
	cvWriteReal( fs_write_yml, "KFMeasurementNoiseSigma",	0.001);	
	cvWriteReal( fs_write_yml, "iCornerMaxNumber",		1000);	
	cvWriteReal( fs_write_yml, "dMinDistance",		1);	
	cvWriteReal( fs_write_yml, "dQualityLevel",	0.1);	
	cvWriteReal( fs_write_yml, "sizeWinLK",						11);	
	cvWriteReal( fs_write_yml, "iLevel",						6);	
	cvWriteReal( fs_write_yml, "dOpticalFlowLengthThreshold",	2.0);	

	cvReleaseFileStorage(&fs_write_yml);
	cvReleaseMemStorage(&memstorage);
}
//儲存設定值至config.yml
void ievWriteConfig(EvConfigData* sConfigData, char* config_name, char FolderRootPath[])
{
	CvMemStorage* memstorage = cvCreateMemStorage(0);
	CvFileStorage* fs_write_yml = cvOpenFileStorage( config_name, memstorage, CV_STORAGE_WRITE);

	cvWriteComment( fs_write_yml, "Config Table", 1);
	cvWriteString( fs_write_yml, "FolderRootPath", FolderRootPath);

	cvWriteComment( fs_write_yml, "SetJudge", 0);
	cvWriteInt ( fs_write_yml, "CountLimit",				sConfigData->sOnlineCADParam.iCountLimit);
	cvWriteReal( fs_write_yml, "SdvThreshold",				sConfigData->sOnlineCADParam.dSdvThres);
	cvWriteReal( fs_write_yml, "EdgeThreshold",				sConfigData->sOnlineCADParam.dEdgeTres);
	cvWriteReal( fs_write_yml, "LoaclSdvThreshold",			sConfigData->sOnlineCADParam.dLocalSdvThres);
	cvWriteReal( fs_write_yml, "KFProcessNoiseSigma",		sConfigData->sOnlineCADParam.dProcessNoiseSigma);
	cvWriteReal( fs_write_yml, "KFMeasurementNoiseSigma",	sConfigData->sOnlineCADParam.dMeasurementNoiseSigma);

	cvReleaseFileStorage(&fs_write_yml);
	cvReleaseMemStorage(&memstorage);
}
