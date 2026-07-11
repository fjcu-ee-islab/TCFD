#include "evLaneCameraAnomalyDetection.h"

void DrawCondition(IplImage* imgInput,int iCondition);
void FeatureLogOpen(FILE** fFeatureLog, char** argv, int iType);
void FeatureLogClose(FILE** fFeatureLog, int iType);
void FeatureLog(FILE** fFeatureLog, EvCameraAnomalyDetection* sCameraAnomalyDetection, int iType);

int main( int argc, char** argv )
{   
	FILE* fOutputAlarm;	/*輸出偵測結果*/
	fOutputAlarm = fopen( "OutputAlarm.txt", "a" );

	int iCond = 0;			/*0:無事件, 1:可能異常, 2:發生異常事件*/
	int iShowImage = 0;		/*1:Show結果影像, 0反之*/
	int iSensitivity = 0;	/*預設為7, 共1~10組參數可選擇*/
	int iLogType = 0;		/*1:bin和dat, 0:bin */

	char strTemp[128];

	CvCapture* capInput = NULL;
	IplImage*  imgInput = NULL;

	/*將影像的上方和下方各減30， i.e.影像的長少了60。*/
	/*因為有些影像的上方或下方"有顯示日期時間"，會影響光流的判斷，所以設定並且使用ROI影像作為演算法的輸入影像*/
	/*ROI Image Size = 640*420。 若不用ROI，則Image Size = 640*480。*/
#if EV_CAD_SETIMAGEROI
	IplImage*  imgSetImageROITemp	= cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 3 );
	IplImage*  imgSrc				= cvCreateImage( cvSize( 640, 420 ), IPL_DEPTH_8U, 3 );
#else 
	IplImage*  imgSrc				= cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 3 );
#endif

	FILE* fFeatureLog[5]; 
	EvCameraAnomalyDetection*         sCameraAnomalyDetection = NULL;
	EvCameraAnomalyDetectionParam     sCameraAnomalyDetectionParam;

	//if( argc == EV_CAD_FROMARGV )
	//{
	//	iShowImage	= atoi(argv[2]);
	//	iLogType	= atoi(argv[3]);
	//	sprintf( strTemp, "%s.avi", argv[1] );
	//	capInput = cvCaptureFromAVI( strTemp );
	//	FeatureLogOpen( fFeatureLog, argv, iLogType ); /*開啟文字檔*/

	//	sCameraAnomalyDetectionParam.iArgc = argc;
	//	sCameraAnomalyDetectionParam.iCADParamNum = EV_PARAM_BY_USER;
	//	sCameraAnomalyDetectionParam.iOnlineCADParamNum = EV_PARAM_BY_USER;
	//	sCameraAnomalyDetectionParam.sConfigData.sOnlineCADParam.ievOnlineCADParamNum = EV_PARAM_BY_USER;

	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dProcessNoiseSigma		= atof(argv[4]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dMeasurementNoiseSigma	= atof(argv[5]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.iCountLimit			= atoi(argv[6]);
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dSdvThres				= atof(argv[7]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dEdgeTres				= atof(argv[8]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dLocalSdvThres			= atof(argv[9]);	

	//	/*cvGoodFeaturesToTrack函數的參數*/
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.iCornerMaxNumber		= atoi(argv[10]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dMinDistance			= atof(argv[11]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dQualityLevel			= atof(argv[12]);	

	//	/*cvCalcOpticalFlowPyrLK函數的參數*/
	//	int iTemp;
	//	iTemp = atoi(argv[13]);	
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.sizeWinLK				= cvSize(iTemp, iTemp);
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.iLevel					= atoi(argv[14]);	

	//	/*Histgram of Optical Flow函數的參數*/
	//	sCameraAnomalyDetectionParam.sOnlineCADParam.dOpticalFlowLengthThreshold = atof(argv[15]);
	//}
	if( argc == EV_CAD_FROMYML )
	{
		sCameraAnomalyDetectionParam.iArgc = argc;
		sprintf(strTemp, "%s.avi", argv[1]);
		iShowImage	= atoi(argv[2]);
		iSensitivity= atoi(argv[3]);
		capInput = cvCaptureFromAVI( strTemp );
		//iLogType	= atoi(argv[4]);
		//FeatureLogOpen(fFeatureLog, argv, iLogType); /*開啟文字檔*/
	}
	else if(argc == EV_CAD_SINGLETEST)
	{
		sCameraAnomalyDetectionParam.iArgc = argc;
		capInput = cvCaptureFromAVI("LaneBlockage01.avi");
	}
	else
	{
		printf("Error Parameter Number! argc Need 1 or 4 or 16 number parameter.");
		printf("\nargc = %d\n",argc);
		system("pause");
		return 0;
	}

	imgInput = cvQueryFrame( capInput );

#if EV_CAD_SETIMAGEROI
	cvResize( imgInput, imgSetImageROITemp );
	CvRect rectROI;
	int iROIStartingPointX = 0;
	int iROIStartingPointY = 30;
	rectROI = cvRect( iROIStartingPointX, iROIStartingPointY, 640-iROIStartingPointX*2, 480-iROIStartingPointY*2 );
	cvSetImageROI( imgSetImageROITemp, rectROI );
	cvCopyImage( imgSetImageROITemp, imgSrc );
#else 
	cvResize( imgInput, imgSrc );
#endif

	/*選擇1~10參數組。(可省略。EV_PARAM_BY_USER為讀取config.yml檔自行輸入參數)*/
	if( argc != EV_CAD_FROMARGV )
		evSetCameraAnomalyDetection( NULL, &sCameraAnomalyDetectionParam, iSensitivity );

	/*配置記憶體*/
	sCameraAnomalyDetection = evCreateCameraAnomalyDetection( imgSrc, &sCameraAnomalyDetectionParam );

	while( imgInput )
	{
		/*frame計數*/
		sCameraAnomalyDetection->sOnlineCAD->iFrameCount++;

#if EV_CAD_SETIMAGEROI
		cvResetImageROI( imgSetImageROITemp );
		cvResize( imgInput, imgSetImageROITemp );
		cvSetImageROI( imgSetImageROITemp, rectROI );
		cvCopyImage( imgSetImageROITemp, imgSrc );
#else 
		cvResize( imgInput, imgSrc );
#endif

		/*攝影機異常偵測*/
		iCond = evDoCameraAnomalyDetection ( imgSrc, sCameraAnomalyDetection );

		if ( iCond > 1 )
			fprintf( fOutputAlarm, "alarm\t%s\tFrame# %d\n", argv[1], 
			sCameraAnomalyDetection->sOnlineCAD->iFrameCount );	

		if( iShowImage == EV_CAD_SHOWIMAGE || argc == EV_CAD_SINGLETEST )
		{
			/*繪製輸出影像 正常畫綠框 注意畫黃框 異常畫紅框*/
			DrawCondition( imgSrc, iCond ); 

			cvShowImage( "CameraAnomalyDetection Image", imgSrc );

			if( cvWaitKey( 1 ) == 27 )
				break;
		}

		//if( argc != EV_CAD_SINGLETEST )
		//FeatureLog( fFeatureLog, sCameraAnomalyDetection, iLogType ); /*將特徵值存在文字檔內*/

		imgInput = cvQueryFrame( capInput );
	}

	//if( argc != EV_CAD_SINGLETEST )
	//FeatureLogClose( fFeatureLog, iLogType ); /*結束文字檔*/

	/*釋放記憶體*/
	fclose( fOutputAlarm ); 
	evReleaseCameraAnomalyDetection( sCameraAnomalyDetection );
	cvReleaseCapture( &capInput );

	return 0;
}

/*加工繪製輸出影像 正常畫綠框 注意畫黃框 異常畫紅框*/
void DrawCondition(IplImage* imgInput,int iCondition)
{
	if(iCondition == 0) /*無事件 畫綠框*/
	{
		cvLine(imgInput,cvPoint(0,0),cvPoint(0,imgInput->height),CV_RGB(0,255,0),5,8);
		cvLine(imgInput,cvPoint(0,0),cvPoint(imgInput->width,0),CV_RGB(0,255,0),5,8);
		cvLine(imgInput,cvPoint(0,imgInput->height-2),cvPoint(imgInput->width,imgInput->height-2),CV_RGB(0,255,0),5,8);
		cvLine(imgInput,cvPoint(imgInput->width-2,0),cvPoint(imgInput->width-2,imgInput->height),CV_RGB(0,255,0),5,8);

	}
	else if(iCondition == 1) /*事件計數中 畫黃框*/
	{
		cvLine(imgInput,cvPoint(0,0),cvPoint(0,imgInput->height),CV_RGB(255,255,0),5,8);
		cvLine(imgInput,cvPoint(0,0),cvPoint(imgInput->width,0),CV_RGB(255,255,0),5,8);
		cvLine(imgInput,cvPoint(0,imgInput->height-2),cvPoint(imgInput->width,imgInput->height-2),CV_RGB(255,255,0),5,8);
		cvLine(imgInput,cvPoint(imgInput->width-2,0),cvPoint(imgInput->width-2,imgInput->height),CV_RGB(255,255,0),5,8);

	}
	else /*警報 畫紅框和打叉*/
	{
		cvLine(imgInput,cvPoint(0,0),cvPoint(0,imgInput->height),CV_RGB(255,0,0),5,8);
		cvLine(imgInput,cvPoint(0,0),cvPoint(imgInput->width,0),CV_RGB(255,0,0),5,8);
		cvLine(imgInput,cvPoint(0,0),cvPoint(imgInput->width,imgInput->height),CV_RGB(255,0,0),5,8);
		cvLine(imgInput,cvPoint(0,imgInput->height),cvPoint(imgInput->width,0),CV_RGB(255,0,0),5,8);
		cvLine(imgInput,cvPoint(0,imgInput->height-2),cvPoint(imgInput->width,imgInput->height-2),CV_RGB(255,0,0),5,8);
		cvLine(imgInput,cvPoint(imgInput->width-2,0),cvPoint(imgInput->width-2,imgInput->height),CV_RGB(255,0,0),5,8);
	}
}

void FeatureLogOpen(FILE** fFeatureLog, char** argv, int iType)
{
	char strTemp[128];

	/*建立儲存特徵值的檔案*/
	if(iType == 0 || iType == 1)
	{
		sprintf(strTemp, "%sbinSdv.bin", argv[1]);
		fFeatureLog[0] = fopen(strTemp,"wb");

		sprintf(strTemp, "%sbinEdge.bin", argv[1]);
		fFeatureLog[1] = fopen(strTemp,"wb");

		sprintf(strTemp, "%sbinLsdv.bin", argv[1]);
		fFeatureLog[2] = fopen(strTemp,"wb");
	}
	if(iType == 1)
	{
		sprintf(strTemp, "%sNowPredict.dat", argv[1]);
		fFeatureLog[3] = fopen(strTemp,"w");

		sprintf(strTemp, "%sResult.dat", argv[1]);
		fFeatureLog[4] = fopen(strTemp,"w");
	}
}

void FeatureLogClose(FILE** fFeatureLog, int iType)
{
	if(iType == 0 || iType == 1)
	{
		fclose(fFeatureLog[0]);
		fclose(fFeatureLog[1]);
		fclose(fFeatureLog[2]);
	}
	if(iType == 1)
	{
		fclose(fFeatureLog[3]);
		fclose(fFeatureLog[4]);
	}
}

/*將特徵值和結果紀錄在文字檔內*/
void FeatureLog(FILE** fFeatureLog, EvCameraAnomalyDetection* sCameraAnomalyDetection, int iType)
{
	int i;

	/*目前特徵值(二進位檔案)*/
	if(iType == 0 || iType == 1)
	{
		fwrite(&sCameraAnomalyDetection->sOnlineCAD-> sSdv->dPredictNow , 1 , sizeof(double) , fFeatureLog[0] );
		fwrite(&sCameraAnomalyDetection->sOnlineCAD->sEdge->dPredictNow , 1 , sizeof(double) , fFeatureLog[1] );
		fwrite(&sCameraAnomalyDetection->sOnlineCAD->sLsdv->dPredictNow , 1 , sizeof(double) , fFeatureLog[2] );
	}
	if(iType == 1)
	{
		/*目前特徵值*/
		fprintf(fFeatureLog[3],"%lf %lf %lf\n" 
			, sCameraAnomalyDetection->sOnlineCAD-> sSdv->dPredictNow 
			, sCameraAnomalyDetection->sOnlineCAD->sEdge->dPredictNow 
			, sCameraAnomalyDetection->sOnlineCAD->sLsdv->dPredictNow );

		/*判斷結果*/
		fprintf(fFeatureLog[4],"%d %d %d %d\n" 
			, sCameraAnomalyDetection->sOnlineCAD->Judgement
			, sCameraAnomalyDetection->sOnlineCAD->iCondCount
			, sCameraAnomalyDetection->iCondKeep
			, 0);
	}
}