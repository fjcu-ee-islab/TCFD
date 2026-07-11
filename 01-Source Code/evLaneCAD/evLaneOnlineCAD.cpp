#include "evLaneOnlineCAD.h"

#define OCAD_LC_MASK 32		//區域mask大小
#define OCAD_LC_SHIFT 16	//區域mask移動距離

void ievFeatureExtraction(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);
void ievMeanSdv(EvOnlineCAD *sOnlineCAD);
void ievEdgeIntensity(EvOnlineCAD *sOnlineCAD);
void ievLocalContrastLBP(EvOnlineCAD *sOnlineCAD);
void ievFeatureSmoothingNow(EvOnlineCAD *sOnlineCAD);
void ievJudgement(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);
double ievAvg_2D8UC1(IplImage* imgSrc);
double ievSdv_2D8UC1(IplImage* imgSrc, double dAvg);
double ievdLC(IplImage* imgSrc, int iHeight, int iWidth, int iMask);
iEvKalman* ievCreateOneToOneKalmanFilter(double dQ,double dR);
float ievDoOneToOneKalmanFilter(iEvKalman* sKalman, double dNow);
void ievReleaseOneToOneKalmanFilter(iEvKalman* sKalman);
void ievMiscDebug(IplImage *Debug, double dUpLimit, double dDownLimit, double dNow);

void ievOpticalFlowHistType(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);
int ievHOOFMeasurement(EvOnlineCAD *sOnlineCAD);
void ievCornerDebug(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);
void ievOpticalFlowDebug(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);
void ievHOOFDegub(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc);

EvOnlineCADParam ievSetOnlineCAD1();
EvOnlineCADParam ievSetOnlineCAD2();
EvOnlineCADParam ievSetOnlineCAD3();
EvOnlineCADParam ievSetOnlineCAD4();
EvOnlineCADParam ievSetOnlineCAD5();
EvOnlineCADParam ievSetOnlineCAD6();
EvOnlineCADParam ievSetOnlineCAD7();
EvOnlineCADParam ievSetOnlineCAD8();
EvOnlineCADParam ievSetOnlineCAD9();
EvOnlineCADParam ievSetOnlineCAD10();
EvOnlineCADParam ievSetOnlineCAD999(EvOnlineCADParam* sOnlineCADParam);

int evSetOnlineCAD(EvOnlineCAD* sOnlineCAD, EvOnlineCADParam* sOnlineCADParam,int ievOnlineCADParamNum)
{
	//設定參數
	switch(ievOnlineCADParamNum)
	{
	case 1:
		*sOnlineCADParam=ievSetOnlineCAD1();break;
	case 2:
		*sOnlineCADParam=ievSetOnlineCAD2();break;
	case 3:
		*sOnlineCADParam=ievSetOnlineCAD3();break;
	case 4:
		*sOnlineCADParam=ievSetOnlineCAD4();break;
	case 5:
		*sOnlineCADParam=ievSetOnlineCAD5();break;
	case 6:
		*sOnlineCADParam=ievSetOnlineCAD6();break;
	case 7:
		*sOnlineCADParam=ievSetOnlineCAD7();break;
	case 8:
		*sOnlineCADParam=ievSetOnlineCAD8();break;
	case 9:
		*sOnlineCADParam=ievSetOnlineCAD9();break;
	case 10:
		*sOnlineCADParam=ievSetOnlineCAD10();break;
	case 999:
		*sOnlineCADParam=ievSetOnlineCAD999(sOnlineCADParam);break;
	case 0:
	default:
		*sOnlineCADParam=ievSetOnlineCAD7();break;
	}

	//若模組不為NULL就將上面設定好得參數複製至模組內
	if(sOnlineCAD != NULL)
		sOnlineCAD->sOnlineCADParam = *sOnlineCADParam;

	return 0;
}

EvOnlineCAD* evCreateOnlineCAD(IplImage* imgSrc, EvOnlineCADParam* sParam)
{	
	EvOnlineCAD *sOnlineCAD;

	double dQ = sParam->dProcessNoiseSigma;
	double dR = sParam->dMeasurementNoiseSigma;

	//配置記憶體
	sOnlineCAD = (EvOnlineCAD*)cvAlloc(sizeof(*sOnlineCAD));

	sOnlineCAD->sOnlineCADParam = *sParam;

	//影像部份
	sOnlineCAD->imgGray			= cvCreateImage(cvGetSize( imgSrc ),IPL_DEPTH_8U,1);
	sOnlineCAD->imgCanny		= cvCreateImage(cvGetSize( imgSrc ),8,1);

	/*cvGoodFeaturesToTrack記憶體空間*/
	int iCornerMaxNumber = sParam->iCornerMaxNumber;
	sOnlineCAD->imgEig	 = cvCreateImage ( cvGetSize( imgSrc ), IPL_DEPTH_32F, 1 );
	sOnlineCAD->imgTemp  = cvCreateImage ( cvGetSize( imgSrc ), IPL_DEPTH_32F, 1 );
	sOnlineCAD->point2D32fCurrentCornerSet = ( CvPoint2D32f * ) cvAlloc ( iCornerMaxNumber * sizeof ( CvPoint2D32f ) );  
	sOnlineCAD->iCornerMaxNumber = 0;

	/*cvCalcOpticalFlowPyrLK記憶體空間*/
	sOnlineCAD->imgPreviou = cvCreateImage ( cvGetSize( imgSrc ), IPL_DEPTH_8U, 1 );
	sOnlineCAD->imgPrevPyr = cvCreateImage ( cvSize ( imgSrc->width + 8, imgSrc->height / 3 ), IPL_DEPTH_8U, 1 );
	sOnlineCAD->imgCurrPyr = cvCreateImage ( cvSize ( imgSrc->width + 8, imgSrc->height / 3 ), IPL_DEPTH_8U, 1 );
	sOnlineCAD->point2D32fPreviouCornerSet	     = ( CvPoint2D32f * ) cvAlloc ( iCornerMaxNumber * sizeof ( CvPoint2D32f ) );
	sOnlineCAD->point2D32fCurrentOpticalFlowSet = ( CvPoint2D32f * ) cvAlloc ( iCornerMaxNumber * sizeof ( CvPoint2D32f ) );
	sOnlineCAD->iPreviouCornerMaxNumber = 0;
	sOnlineCAD->status = ( char * ) cvAlloc ( iCornerMaxNumber );  

	/*ievHOOFMeasurement記憶體空間*/
	sOnlineCAD->iarrHistMod[0] = 0;
	sOnlineCAD->iarrHistMod[1] = 0;
	sOnlineCAD->iHOOFDistributionType = 3;

	//卡爾曼部份
	sOnlineCAD->sKalman[0] = ievCreateOneToOneKalmanFilter(dQ,dR);
	sOnlineCAD->sKalman[1] = ievCreateOneToOneKalmanFilter(dQ,dR);
	sOnlineCAD->sKalman[2] = ievCreateOneToOneKalmanFilter(dQ,dR);
	sOnlineCAD->sKalman[3] = ievCreateOneToOneKalmanFilter(dQ,dR);

	//特徵結構部份
	sOnlineCAD->sMean	= (iEvFeature*)malloc(sizeof(iEvFeature));
	sOnlineCAD->sSdv	= (iEvFeature*)malloc(sizeof(iEvFeature));
	sOnlineCAD->sEdge	= (iEvFeature*)malloc(sizeof(iEvFeature));
	sOnlineCAD->sLsdv	= (iEvFeature*)malloc(sizeof(iEvFeature));

	//計數器初始化
	sOnlineCAD->iFrameCount			= 0;
	sOnlineCAD->iCondCount			= 0;
	sOnlineCAD->now					= Normal;

#if EV_CAD_DEBUG
	//是否顯示debug視窗以及配置debug用記憶體
	for(int i=0;i<3;i++)
	{
		sOnlineCAD->imgT[i] = cvCreateImage(cvSize(300, 256), IPL_DEPTH_8U,3);
		cvZero(sOnlineCAD->imgT[i]);
	}

	sOnlineCAD->imgHOOF = cvCreateImage( cvSize( 250, 480 ), IPL_DEPTH_8U, 3 );

#if EV_CAD_SETIMAGEROI
	sOnlineCAD->imgCorner		= cvCreateImage(cvSize(640, 420), IPL_DEPTH_8U,3);
	sOnlineCAD->imgOpticalFlow	= cvCreateImage(cvSize(640, 420), IPL_DEPTH_8U,3);
#else
	sOnlineCAD->imgCorner = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U,3);
	sOnlineCAD->imgOpticalFlow = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U,3);
#endif
#endif

	return (EvOnlineCAD*)sOnlineCAD;
}

int evDoOnlineCAD(IplImage *imgSrc, EvOnlineCAD *sOnlineCAD)
{
	if(!imgSrc) /*沒影像*/
	{
		printf("Error No imgSrc!\n");
		return -999;
	}
	else
	{
		/*轉灰階*/
		cvCvtColor( imgSrc, sOnlineCAD->imgGray, CV_BGR2GRAY );

		/*擷取特徵*/
		ievFeatureExtraction( sOnlineCAD, imgSrc );

		/*Kalman Filter*/
		if( sOnlineCAD->iFrameCount != 1 )
		{
			ievFeatureSmoothingNow(sOnlineCAD); /*Kalman Filter影像品質特徵值*/
		}
		else /*在第一張影像時， 初始化Kalman Filter的值*/
		{
			sOnlineCAD->sKalman[0]->kalman->state_post->data.fl[0] = sOnlineCAD->sMean->dNow;
			sOnlineCAD->sKalman[1]->kalman->state_post->data.fl[0] = sOnlineCAD-> sSdv->dNow;
			sOnlineCAD->sKalman[2]->kalman->state_post->data.fl[0] = sOnlineCAD->sEdge->dNow;
			sOnlineCAD->sKalman[3]->kalman->state_post->data.fl[0] = sOnlineCAD->sLsdv->dNow;

			sOnlineCAD->sMean->dPredictNow = sOnlineCAD->sMean->dNow;
			sOnlineCAD-> sSdv->dPredictNow = sOnlineCAD-> sSdv->dNow;
			sOnlineCAD->sEdge->dPredictNow = sOnlineCAD->sEdge->dNow;
			sOnlineCAD->sLsdv->dPredictNow = sOnlineCAD->sLsdv->dNow;
		}

		/*判斷狀況*/
		ievJudgement(sOnlineCAD, imgSrc);
	}

	/*回傳判斷結果*/
	if( sOnlineCAD->iFrameCount > 0 )
		return sOnlineCAD->Judgement;
}

/*釋放CameraAnomalyDetection記憶體*/
void evReleaseOnlineCAD(EvOnlineCAD *sOnlineCAD)
{	
	if(sOnlineCAD)
	{

#if EV_CAD_DEBUG
		//釋放額外配置的debug記憶體
		for(int i=0;i<3;i++)
			cvReleaseImage(&sOnlineCAD->imgT[i]);

		cvReleaseImage(&sOnlineCAD->imgCorner);
		cvReleaseImage(&sOnlineCAD->imgOpticalFlow);
		cvReleaseImage(&sOnlineCAD->imgHOOF);
#endif

		cvReleaseImage(&sOnlineCAD->imgGray);
		cvReleaseImage(&sOnlineCAD->imgCanny);

		/*cvGoodFeaturesToTrack的Image*/
		cvReleaseImage(&sOnlineCAD->imgEig);
		cvReleaseImage(&sOnlineCAD->imgTemp);

		/*cvCalcOpticalFlowPyrLK的Image*/
		cvReleaseImage(&sOnlineCAD->imgPreviou);
		cvReleaseImage(&sOnlineCAD->imgPrevPyr);
		cvReleaseImage(&sOnlineCAD->imgCurrPyr);

		free(sOnlineCAD->sMean);
		free(sOnlineCAD->sSdv);
		free(sOnlineCAD->sEdge);
		free(sOnlineCAD->sLsdv);

		ievReleaseOneToOneKalmanFilter(sOnlineCAD->sKalman[0]);
		ievReleaseOneToOneKalmanFilter(sOnlineCAD->sKalman[1]);
		ievReleaseOneToOneKalmanFilter(sOnlineCAD->sKalman[2]);
		ievReleaseOneToOneKalmanFilter(sOnlineCAD->sKalman[3]);

		cvFree(&sOnlineCAD);
	}
}

//特徵擷取
void ievFeatureExtraction(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	/*平均 和 標準差*/
	ievMeanSdv(sOnlineCAD);

	/*邊緣*/
	ievEdgeIntensity(sOnlineCAD);

	/*區域標準差*/
	ievLocalContrastLBP(sOnlineCAD);

	/*光流方向直方圖的類別*/
	ievOpticalFlowHistType(sOnlineCAD, imgSrc);
}

/*kalman Filter當前值*/
void ievFeatureSmoothingNow(EvOnlineCAD *sOnlineCAD)
{
	/*Kalman 濾波*/
	sOnlineCAD->sMean->dPredictNow = (double)ievDoOneToOneKalmanFilter(sOnlineCAD->sKalman[0],sOnlineCAD->sMean->dNow);
	sOnlineCAD-> sSdv->dPredictNow = (double)ievDoOneToOneKalmanFilter(sOnlineCAD->sKalman[1],sOnlineCAD-> sSdv->dNow);
	sOnlineCAD->sEdge->dPredictNow = (double)ievDoOneToOneKalmanFilter(sOnlineCAD->sKalman[2],sOnlineCAD->sEdge->dNow);
	sOnlineCAD->sLsdv->dPredictNow = (double)ievDoOneToOneKalmanFilter(sOnlineCAD->sKalman[3],sOnlineCAD->sLsdv->dNow);

	/*避免各值因卡爾曼特性超出範圍(0~1)*/
	if(sOnlineCAD->sMean->dPredictNow < 0.0)
		sOnlineCAD->sMean->dPredictNow = 0.0;
	if(sOnlineCAD->sSdv->dPredictNow < 0.0)
		sOnlineCAD->sSdv->dPredictNow = 0.0;
	if(sOnlineCAD->sEdge->dPredictNow < 0.0)
		sOnlineCAD->sEdge->dPredictNow = 0.0;
	if(sOnlineCAD->sLsdv->dPredictNow < 0.0)
		sOnlineCAD->sLsdv->dPredictNow = 0.0;

	if(sOnlineCAD->sMean->dPredictNow > 1.0)
		sOnlineCAD->sMean->dPredictNow = 1.0;
	if(sOnlineCAD->sSdv->dPredictNow > 1.0)
		sOnlineCAD->sSdv->dPredictNow = 1.0;
	if(sOnlineCAD->sEdge->dPredictNow > 1.0)
		sOnlineCAD->sEdge->dPredictNow = 1.0;
	if(sOnlineCAD->sLsdv->dPredictNow > 1.0)
		sOnlineCAD->sLsdv->dPredictNow = 1.0;
}

//計算邊緣強度
void ievEdgeIntensity(EvOnlineCAD *sOnlineCAD)
{
	int iEdgeCount=0, i;
	int iTotalpixel = sOnlineCAD->imgGray->width*sOnlineCAD->imgGray->height;
	uchar* CannyPtr = (uchar*)sOnlineCAD->imgCanny->imageData;
	uchar* SRPtr = NULL;

	//Canny邊緣偵測
	cvCanny(sOnlineCAD->imgGray, sOnlineCAD->imgCanny, 50, 150, 3);	

	//計數判斷為邊緣的像素數量
	for(i = 0 ; i < iTotalpixel ; i++, CannyPtr++)
	{
		if(CannyPtr[0] == 255)
			iEdgeCount++;
	}

	//正規化 0~1 之間
	sOnlineCAD->sEdge->dNow = (double)iEdgeCount/iTotalpixel;
}

//計算區域標準差
void ievLocalContrastLBP(EvOnlineCAD *sOnlineCAD)
{
	int i, j;
	int RangeH = sOnlineCAD->imgGray->height - OCAD_LC_SHIFT;
	int RangeW = sOnlineCAD->imgGray->width  - OCAD_LC_SHIFT ;
	int LCNormalize = 0 ;
	double dTemp;
	double dLCSum = 0 ;

	//加總所有Block內的標準差
	for(i = 0 ; i < RangeH ; i += OCAD_LC_SHIFT)				//上下都間隔 LC_SHIFT 像素
		for(j = 0 ; j < RangeW ; j += OCAD_LC_SHIFT)			//左右都間隔 LC_SHIFT 像素
		{	
			//計算當前像素往右32往下32範圍 (32*32的小圖) 之標準差，若範圍內顯著區域少於一半則回傳 -1
			dTemp = ievdLC(sOnlineCAD->imgGray, i, j, OCAD_LC_MASK);				

			dLCSum += dTemp;

			//計數Block數量
			LCNormalize++;																	
		}


		if(LCNormalize == 0)
			dLCSum = 0;				//防止除以零
		else
			dLCSum /= LCNormalize;	//取平均

		sOnlineCAD->sLsdv->dNow = dLCSum/127.5;	//8bits圖標準差值介於0~127.5(理想值) 正規化0~1之間除以127.5即可											
}

//計算平均亮度與標準差
void ievMeanSdv(EvOnlineCAD *sOnlineCAD)
{
	//計算平均亮度
	sOnlineCAD->sMean->dNow = ievAvg_2D8UC1(sOnlineCAD->imgGray);
	//計算標準差
	sOnlineCAD->sSdv ->dNow = ievSdv_2D8UC1(sOnlineCAD->imgGray, sOnlineCAD->sMean->dNow);

	//正規化 0~1 區間
	sOnlineCAD->sMean->dNow /= 255;  //8bits圖平均亮度值介於0~255 正規化0~1之間除以255即可
	sOnlineCAD->sSdv->dNow /= 127.5; //8bits圖標準差值介於0~127.5(理想值) 正規化0~1之間除以127.5即可
}

//狀況判斷
void ievJudgement(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	int iTemp = 0;
	sOnlineCAD->Judgement = 0;

	/*判斷影像品質*/
	if( sOnlineCAD->sSdv->dPredictNow < sOnlineCAD->sOnlineCADParam.dSdvThres ) 
		iTemp +=1;
	else if( sOnlineCAD->sEdge->dPredictNow < sOnlineCAD->sOnlineCADParam.dEdgeTres ) 
		iTemp +=1;
	else if( sOnlineCAD->sLsdv->dPredictNow < sOnlineCAD->sOnlineCADParam.dLocalSdvThres )
		iTemp +=1;

	/*結合光流分布特性*/
	/*iHOOFDistributionType == 1為異常均勻分布*/
	/*iHOOFDistributionType == 2為異常高斯分布*/
	/*iHOOFDistributionType == 3為正常無分布*/
	/*iHOOFDistributionType == 4為正常指數分布*/
	/*iHOOFDistributionType == 5為正常閃爍分布*/
	if ( iTemp == 1 )
	{
		if( sOnlineCAD->iHOOFDistributionType == 1 )
			iTemp +=1;
		else if( sOnlineCAD->iHOOFDistributionType == 2 )
			iTemp +=1;
		else if( sOnlineCAD->iHOOFDistributionType == 3 )
			iTemp -=1;
		else if( sOnlineCAD->iHOOFDistributionType == 4 )
			iTemp -=1;
		else if( sOnlineCAD->iHOOFDistributionType == 5 )
			iTemp -=1;
	}

	//有限狀態機，0為正常、1為需注意、2為異常
	if(iTemp > 0)
	{
		if(sOnlineCAD->iCondCount < sOnlineCAD->sOnlineCADParam.iCountLimit)
		{
			//正常狀態且特徵值超過閥值，在sOnlineCAD->sOnlineCADParam.iCountLimit幀內為需注意
			if ( sOnlineCAD->iHOOFDistributionType == 1 )
			{
				sOnlineCAD->iCondCount = sOnlineCAD->iCondCount + 6; /*如果是均勻分布, 給予的FSM權重值較高*/
			}
			else
			{
				sOnlineCAD->iCondCount++;
			}

			sOnlineCAD->now = NtoAWait;		//狀態：Normal → NtoAWait
			sOnlineCAD->Judgement = 1;		//輸出：1=需注意
		}
		else 
		{
			//異常狀態且特徵值超過閥值，超過sOnlineCAD->sOnlineCADParam.iCountLimit幀為異常
			sOnlineCAD->now = Alarm;		//狀態：Wait → Alarm
			sOnlineCAD->Judgement = 2;		//輸出：iTemp=異常
		}
	}
	else
	{
		if(sOnlineCAD->now == Alarm)
		{
			//異常狀態且特徵值少於閥值，且在sOnlineCAD->sOnlineCADParam.iCountLimit幀內輸出保持為異常
			sOnlineCAD->iCondCount--;
			sOnlineCAD->now = AtoNWait;		//狀態：Alarm → AtoNWait
			sOnlineCAD->Judgement = 2;		//輸出：2=異常(AtoNWait版)
		}
		else if(sOnlineCAD->now == NtoAWait)
		{
			//異常狀態、特徵值少於閥值，且在sOnlineCAD->sOnlineCADParam.iCountLimit幀內保持為異常，超過30幀轉為正常
			if(sOnlineCAD->iCondCount > 0)
			{
				sOnlineCAD->iCondCount--;
				//sOnlineCAD->now = NtoAWait;	//狀態：NtoAWait → NtoAWait
				sOnlineCAD->Judgement = 1;	//輸出：1=需注意
			}
			else 
			{
				sOnlineCAD->now = Normal;	//狀態：Wait → Normal
				sOnlineCAD->Judgement = 0;	//輸出：0=正常
			}
		}
		else if(sOnlineCAD->now == AtoNWait)
		{
			//異常狀態、特徵值少於閥值，且在sOnlineCAD->sOnlineCADParam.iCountLimit幀內保持為異常，超過30幀轉為正常
			if(sOnlineCAD->iCondCount > 0)
			{
				sOnlineCAD->iCondCount--;
				sOnlineCAD->now = AtoNWait;		//狀態：Wait → Wait
				//sOnlineCAD->Judgement = sOnlineCAD->Judgement;	//輸出：上次輸出
			}
			else 
			{
				sOnlineCAD->now = Normal;	//狀態：Wait → Normal
				sOnlineCAD->Judgement = 0;	//輸出：0=正常
			}
		}
		else
		{
			//正常狀態且特徵值少於閥值 輸出為正常
			sOnlineCAD->now = Normal;			//狀態：Normal → Normal
			sOnlineCAD->Judgement =0;			//輸出：0=正常
		}
	}
}

//配置單一輸入且只做一預測用之kalmanFilter
iEvKalman* ievCreateOneToOneKalmanFilter(double dQ,double dR)
{
	iEvKalman* sKalman;
	sKalman = (iEvKalman*)cvAlloc(sizeof(*sKalman));

	/*************************************卡爾曼部份開始*************************************/
	//						┌		┐
	sKalman->A[0] = 1;										//						│ 1  1 │
	sKalman->A[1] = 1;										// 宣告 A 讓卡爾曼視為	│		│
	sKalman->A[2] = 0;										//						│ 0  1 │		
	sKalman->A[3] = 1;										//						└		┘

	sKalman->rng = cvRNG(-1);								// 宣告 rng 隨機數
	sKalman->process_noise = cvCreateMat( 2, 1, CV_32FC1 );	// 宣告 process_noise 假想處理雜訊的矩陣維度
	sKalman->state = cvCreateMat( 2, 1, CV_32FC1 );			// 宣告 state 暫存狀態的矩陣維度

	sKalman->measurement = cvCreateMat( 1, 1, CV_32FC1 );	// 宣告 measurement 暫存測量值的矩陣維度

	// 宣告 Kalman 濾波器
	sKalman->kalman = cvCreateKalman( 2, 1, 0 );			

	// 更新 rng 值、並用雜訊干擾 state 的值
	cvRandArr( &sKalman->rng, sKalman->state, CV_RAND_NORMAL, cvRealScalar(0), cvRealScalar(0.1) );		

	// 將矩陣 A 複製到 kalman->transition_matrix->data.db 內
	memcpy( sKalman->kalman->transition_matrix->data.db, sKalman->A, sizeof(sKalman->A));
	//										┌		┐
	//										│ 1  0 │
	cvSetIdentity( sKalman->kalman->measurement_matrix, cvRealScalar(1));				// 設定 kalman_A->measurement_matrix 為	│		│
	//										│ 0  1 │
	//										└		┘
	// process_noise sigma Q 設定接收雜訊的高斯sigma為 Q (實際雜訊高的情況要調低)
	cvSetIdentity( sKalman->kalman->process_noise_cov, cvRealScalar(dQ));
	// measurement sigma R 設定觀察雜訊的高斯sigma為 R (測量雜訊(如儀器本身原因)高的情況要調低)
	cvSetIdentity( sKalman->kalman->measurement_noise_cov, cvRealScalar(dR));	
	// priori error estimate covariance matrix P'(k)=A*P(k-1)*At + Q)
	cvSetIdentity( sKalman->kalman->error_cov_post, cvRealScalar(1));					

	/*************************************卡爾曼部份結束*************************************/
	return (iEvKalman*)sKalman;
}

//Kalman濾波：僅一個測量值輸入並預測一個預測值 KalmanInfo->prediction->data.db[0]
float ievDoOneToOneKalmanFilter(iEvKalman* sKalman, double dNow)
{
	// 更新 prediction 值
	sKalman->prediction = cvKalmanPredict( sKalman->kalman, 0 );	

	// 更新 rng 值、並用高斯雜訊 measurement_noise_cov 干擾 measurement 的值 
	cvRandArr( &sKalman->rng, sKalman->measurement, CV_RAND_NORMAL, cvRealScalar(0), cvRealScalar(sqrt(sKalman->kalman->measurement_noise_cov->data.fl[0])) );

	// 將三個矩陣的積 ( kalman->measurement_matrix 乘 state 乘 measurement ) 存入 measurement (x=Fx+w)
	cvMatMulAdd( sKalman->kalman->measurement_matrix, sKalman->state, sKalman->measurement, sKalman->measurement );	

	// 輸入平均亮度測量值
	sKalman->measurement->data.fl[0] = (float)dNow ;

	// 用測量值更新kalman
	cvKalmanCorrect( sKalman->kalman, sKalman->measurement );	

	// 更新 rng 值、並用高斯雜訊 measurement_noise_cov 干擾 process_noise 的值 
	cvRandArr( &sKalman->rng, sKalman->process_noise, CV_RAND_NORMAL, cvRealScalar(0),cvRealScalar(sqrt(sKalman->kalman->process_noise_cov->data.fl[0])));

	// 將三個矩陣的積 ( kalman->measurement_matrix 乘 state 乘 process_noise ) 存入 state (z=Hx+v)
	cvMatMulAdd( sKalman->kalman->transition_matrix, sKalman->state, sKalman->process_noise, sKalman->state );		

	//輸出此次預測結果
	return sKalman->prediction->data.fl[0];
}

//釋放Kalman記憶體
void ievReleaseOneToOneKalmanFilter(iEvKalman* sKalman)
{
	if(sKalman)
	{
		cvReleaseMat(&sKalman->process_noise);	
		cvReleaseMat(&sKalman->state);			
		cvReleaseMat(&sKalman->measurement);	
		cvReleaseKalman(&sKalman->kalman);			
		cvFree(&sKalman);
	}
}

//debug用各特徵值走勢圖
void ievMiscDebug(IplImage *Debug, double dUpLimit, double dDownLimit, double dNow)
{
	//跑時間走勢圖 以紅色表下方閥值、黃色表上方閥值、綠色表示特徵值
	int i,j;
	uchar ucThresUpRed,ucThresUpBlue,ucThresUpGreen;
	uchar ucThresDownRed,ucThresDownBlue,ucThresDownGreen;
	uchar ucFRed,ucFBlue,ucFGreen;

	if(dNow < 0)
		dNow = 0;
	if(dNow > 255)
		dNow = 255;

	if(Debug && Debug->height == 256 )
	{
		ucThresUpRed = 255;
		ucThresUpGreen = 255;
		ucThresUpBlue = 0;

		ucThresDownRed = 255;
		ucThresDownGreen = 0;
		ucThresDownBlue = 0;

		ucFRed = 0;
		ucFGreen = 255;
		ucFBlue = 0;

		//由上往下、由左往右找，往左複製
		for ( j=0; j< Debug->width; j++)
			for ( i=0; i< Debug->height; i++)
			{	
				if(   ((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+0] == 0 
					&&((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+1] == 255 
					&&((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+2] == 0  )
				{	
					//清為黑色 
					((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+0] = 0 ;
					((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+1] = 0 ;
					((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+2] = 0 ;

					//(最左一列外)左移一個像素
					if(j)
					{
						((uchar*)( Debug->imageData+ Debug->widthStep* i ))[(j-1)*Debug->nChannels+0] = 0 ;
						((uchar*)( Debug->imageData+ Debug->widthStep* i ))[(j-1)*Debug->nChannels+1] = 255 ;
						((uchar*)( Debug->imageData+ Debug->widthStep* i ))[(j-1)*Debug->nChannels+2] = 0 ;
					}

					//結束他
					i = Debug->height;
				}
			}

			//在最右一列點上特徵值
			i = 255 - (int)(dNow*255);
			((uchar*)( Debug->imageData+ Debug->widthStep* i))[(j-1)*Debug->nChannels+0] = ucFBlue ;
			((uchar*)( Debug->imageData+ Debug->widthStep* i))[(j-1)*Debug->nChannels+1] = ucFGreen ;
			((uchar*)( Debug->imageData+ Debug->widthStep* i))[(j-1)*Debug->nChannels+2] = ucFRed ;

			//繪製上面的閥值
			i = 255 - (int)(dUpLimit*255);
			for ( j=0; j< Debug->width; j++)
			{	
				/*BGR*/
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+0] = ucThresUpBlue ;
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+1] = ucThresUpGreen ;
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+2] = ucThresUpRed ;
			}

			//繪製下面的閥值
			i = 255 - (int)(dDownLimit*255);
			for ( j=0; j< Debug->width; j++)
			{	
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+0] = ucThresDownBlue ;
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+1] = ucThresDownGreen ;
				((uchar*)( Debug->imageData+ Debug->widthStep* i))[j*Debug->nChannels+2] = ucThresDownRed ;
			}
	}
}

//計算平均亮度
double ievAvg_2D8UC1(IplImage* imgSrc)
{
	int i;
	double dSum = 0.0;
	int totalpixel = imgSrc->width*imgSrc->height;
	uchar* ucSrcData = (uchar*)imgSrc->imageData;

	//灰階值加總
	for(i = 0 ; i < totalpixel ; i++, ucSrcData++)						
		dSum += ucSrcData[0];
	dSum = dSum/(double)totalpixel;	//總和後除像素總數 = 平均值

	return dSum;
}
//計算標準差
double ievSdv_2D8UC1(IplImage* imgSrc, double dAvg)
{
	int i;
	double dSum = 0.0;
	int iTotalpixel = imgSrc->width*imgSrc->height;
	uchar* ucSrcData = (uchar*)imgSrc->imageData;

	//灰階值加總
	for(i = 0 ; i < iTotalpixel ; i++, ucSrcData++)						
		dSum += (ucSrcData[0]-dAvg)*(ucSrcData[0]-dAvg);

	//正規化
	dSum = (double)dSum/(iTotalpixel-1);

	dSum = sqrt(dSum);

	return dSum;
}

//計算區域標準差
double ievdLC(IplImage* imgSrc, int iHeight, int iWidth, int iMask)
{
	uchar* ucSrcData = NULL;
	uchar* ucSRData  = NULL;
	int i, j, k = 0;
	float fSRSum = OCAD_LC_MASK * OCAD_LC_MASK;
	double dSum  = 0.0, dAvg;
	int iSrcNextRow = imgSrc->widthStep - iMask;

	/*計算區域內的平均值*/
	ucSrcData = (uchar*)(imgSrc->imageData + iHeight * imgSrc->widthStep + iWidth);

	for(i = iHeight ; i < iHeight + iMask ; i++, ucSrcData += iSrcNextRow)
	{
		for(j = iWidth ; j < iWidth + iMask ; j++, ucSrcData++)
		{
			dSum += ucSrcData[0]; /*加總像素灰階值*/
		}
	}

	dAvg = dSum / fSRSum;

	dSum = 0.0;
	ucSrcData = (uchar*)(imgSrc->imageData + iHeight * imgSrc->widthStep + iWidth);

	/*計算區域內的標準差*/
	for(i = iHeight ; i < iHeight + iMask ; i++, ucSrcData += iSrcNextRow)
	{
		for(j = iWidth ; j < iWidth + iMask ; j++, ucSrcData++)
		{
			dSum += (ucSrcData[0]-dAvg)*(ucSrcData[0]-dAvg);
		}
	}

	dSum = dSum/(fSRSum - 1);
	dSum = sqrt(dSum);

	return dSum;
}

//1 = 最敏感
EvOnlineCADParam ievSetOnlineCAD1()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 1;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.5;		
	sOnlineCADParam.dEdgeTres = 0.2;		
	sOnlineCADParam.dLocalSdvThres = 0.3;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.0005;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//2
EvOnlineCADParam ievSetOnlineCAD2()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 2;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.5;		
	sOnlineCADParam.dEdgeTres = 0.2;		
	sOnlineCADParam.dLocalSdvThres = 0.3;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.001;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//3
EvOnlineCADParam ievSetOnlineCAD3()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 3;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.2;		
	sOnlineCADParam.dEdgeTres = 0.1;		
	sOnlineCADParam.dLocalSdvThres = 0.15;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.001;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//4
EvOnlineCADParam ievSetOnlineCAD4()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 4;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.2;		
	sOnlineCADParam.dEdgeTres = 0.1;		
	sOnlineCADParam.dLocalSdvThres = 0.15;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.005;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//5
EvOnlineCADParam ievSetOnlineCAD5()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 5;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.2;		
	sOnlineCADParam.dEdgeTres = 0.1;		
	sOnlineCADParam.dLocalSdvThres = 0.15;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.01;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//6
EvOnlineCADParam ievSetOnlineCAD6()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 6;

	sOnlineCADParam.iCountLimit = 30;

	sOnlineCADParam.dSdvThres = 0.1;		
	sOnlineCADParam.dEdgeTres = 0.01;		
	sOnlineCADParam.dLocalSdvThres = 0.03;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.1;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//7
EvOnlineCADParam ievSetOnlineCAD7()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 7;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.1;		
	sOnlineCADParam.dEdgeTres = 0.01;		
	sOnlineCADParam.dLocalSdvThres = 0.03;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.1;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//8
EvOnlineCADParam ievSetOnlineCAD8()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 8;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.1;		
	sOnlineCADParam.dEdgeTres = 0.01;		
	sOnlineCADParam.dLocalSdvThres = 0.03;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.2;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//9
EvOnlineCADParam ievSetOnlineCAD9()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 9;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.1;		
	sOnlineCADParam.dEdgeTres = 0.01;		
	sOnlineCADParam.dLocalSdvThres = 0.03;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.25;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}

//10
EvOnlineCADParam ievSetOnlineCAD10()
{
	EvOnlineCADParam sOnlineCADParam;
	sOnlineCADParam.ievOnlineCADParamNum = 10;

	sOnlineCADParam.iCountLimit = 45;

	sOnlineCADParam.dSdvThres = 0.1;		
	sOnlineCADParam.dEdgeTres = 0.01;		
	sOnlineCADParam.dLocalSdvThres = 0.03;	

	sOnlineCADParam.dProcessNoiseSigma = 1e-4;
	sOnlineCADParam.dMeasurementNoiseSigma = 1e-3;

	/*cvGoodFeaturesToTrack函數的參數*/
	sOnlineCADParam.iCornerMaxNumber   = 1000;
	sOnlineCADParam.dMinDistance  = 1;
	sOnlineCADParam.dQualityLevel = 0.3;

	/*cvCalcOpticalFlowPyrLK函數的參數*/
	sOnlineCADParam.sizeWinLK = cvSize(11, 11);
	sOnlineCADParam.iLevel = 6;

	/*ievHOOFMeasurement函數的參數*/
	sOnlineCADParam.dOpticalFlowLengthThreshold = 2.0;

	return (EvOnlineCADParam)sOnlineCADParam;
}


//使用自訂參數
EvOnlineCADParam ievSetOnlineCAD999(EvOnlineCADParam* sOnlineCADParam)
{
	EvOnlineCADParam sParamOut;
	sParamOut.ievOnlineCADParamNum = EV_PARAM_BY_USER;

	sParamOut.iCountLimit				= sOnlineCADParam->iCountLimit;
	sParamOut.dSdvThres					= sOnlineCADParam->dSdvThres;
	sParamOut.dEdgeTres					= sOnlineCADParam->dEdgeTres;
	sParamOut.dLocalSdvThres			= sOnlineCADParam->dLocalSdvThres;
	sParamOut.dProcessNoiseSigma		= sOnlineCADParam->dProcessNoiseSigma;
	sParamOut.dMeasurementNoiseSigma	= sOnlineCADParam->dMeasurementNoiseSigma;
	sParamOut.iCornerMaxNumber					= sOnlineCADParam->iCornerMaxNumber;
	sParamOut.dMinDistance					= sOnlineCADParam->dMinDistance;
	sParamOut.dQualityLevel				= sOnlineCADParam->dQualityLevel;
	sParamOut.sizeWinLK							= sOnlineCADParam->sizeWinLK;
	sParamOut.iLevel							= sOnlineCADParam->iLevel;
	sParamOut.dOpticalFlowLengthThreshold		= sOnlineCADParam->dOpticalFlowLengthThreshold;

	return (EvOnlineCADParam)sParamOut;
}

/*找中階特徵: HOOF的分布類別*/
void ievOpticalFlowHistType(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	int i;
	sOnlineCAD->iCornerMaxNumber = sOnlineCAD->sOnlineCADParam.iCornerMaxNumber;

	cvGoodFeaturesToTrack( sOnlineCAD->imgGray, sOnlineCAD->imgEig, 
		sOnlineCAD->imgTemp, sOnlineCAD->point2D32fCurrentCornerSet,
		&sOnlineCAD->iCornerMaxNumber, sOnlineCAD->sOnlineCADParam.dQualityLevel, 
		sOnlineCAD->sOnlineCADParam.dMinDistance );

	/*第2張影像開始做光流點的追蹤*/
	if ( sOnlineCAD->iFrameCount > 0 )
	{
		cvCalcOpticalFlowPyrLK ( sOnlineCAD->imgPreviou, sOnlineCAD->imgGray,
			sOnlineCAD->imgPrevPyr, sOnlineCAD->imgCurrPyr, 
			sOnlineCAD->point2D32fPreviouCornerSet, sOnlineCAD->point2D32fCurrentOpticalFlowSet, 
			sOnlineCAD->iPreviouCornerMaxNumber, sOnlineCAD->sOnlineCADParam.sizeWinLK,
			sOnlineCAD->sOnlineCADParam.iLevel, sOnlineCAD->status, 
			NULL, cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ), 0 );

		/*統計光流方向直方圖和判斷分布類別*/	
		sOnlineCAD->iHOOFDistributionType = ievHOOFMeasurement(sOnlineCAD);


#if EV_CAD_DEBUG
		ievOpticalFlowDebug(sOnlineCAD, imgSrc);
		cvShowImage( "OpticalFlow",sOnlineCAD->imgOpticalFlow );

		ievHOOFDegub(sOnlineCAD, imgSrc);
		cvShowImage( "HOOF",sOnlineCAD->imgHOOF );
#endif
	}

	/*Copy*/
	/*複製當前影像 和 角點集合 還有 角點數量， 作為下一張影像要做光流法所需要用到的 "上一張影像之data"。*/
	cvCopyImage( sOnlineCAD->imgGray, sOnlineCAD->imgPreviou );
	for ( i = 0; i < sOnlineCAD->iCornerMaxNumber; i++ )
	{
		sOnlineCAD->point2D32fPreviouCornerSet[i].x = sOnlineCAD->point2D32fCurrentCornerSet[i].x;
		sOnlineCAD->point2D32fPreviouCornerSet[i].y = sOnlineCAD->point2D32fCurrentCornerSet[i].y;
	}
	sOnlineCAD->iPreviouCornerMaxNumber = sOnlineCAD->iCornerMaxNumber;
	/*End Copy*/
}

int ievHOOFMeasurement(EvOnlineCAD *sOnlineCAD)
{
	int i=0, j=0;
	int aa = 0;
	int bb = 0;
	int iMax = 0;
	int iIndex = 0;
	int iAngle_Hist = 0;
	int iOFHistNumber = 0;
	int iDistributionResult = 0;
	int iArr_hist[19] = {0}; /*第19Bin儲存未移動點的數量*/

	double maxSlpoe = 0;
	double dDistance = 0;
	double dRadian_atan2_Hist = 0.0;
	double dAngle_atan2_Hist  = 0.0;
	double dSlpoe[19] = {0};
	double dArr_HistPDF[19] = {0};
	double dArr_histCDF[19] = {0}; 

	char cStr[64]  = {0};
	char cStr2[64] = {0};
	char cStr3[64] = {0};

	/*20130112 Timon 角度直方圖陣列歸零*/
	for( i = 0; i < 19; i++ )
	{
		iArr_hist[i] = 0;
	}

	/*計算並且統計特徵點角度*/
	for( i = 0; i < sOnlineCAD->iPreviouCornerMaxNumber; i++ )
	{
		if( !sOnlineCAD->status[i] )
			continue; /*continue 表示回到上一層迴圈執行，否則向下執行。*/

		/*20130112 Timon 計算角度。*/
		dRadian_atan2_Hist = atan2( (double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y - (double)sOnlineCAD->point2D32fPreviouCornerSet[i].y, 
			(double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x - (double) sOnlineCAD->point2D32fPreviouCornerSet[i].x );
		dAngle_atan2_Hist = -dRadian_atan2_Hist * 180 / 3.14159265358979323846;
		iAngle_Hist = (int) ( 360 + dAngle_atan2_Hist ) % 360;

		dDistance = sqrt( double( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) +
			( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) );

		/*0130114 Timon 特徵點位置不變， 會被計算成0度， 跟特徵點向右一樣， 因此必須要刪除未移動的點。*/
		/*在這裡直接過濾移動距離過小的特徵點(包含未移動)。*/
		if ( dDistance < sOnlineCAD->sOnlineCADParam.dOpticalFlowLengthThreshold )
		{
			iAngle_Hist = 360; // 後面360/20=18 會存到第19bin。
		}

		/*20130112 Timon 統計角度直方圖, 將360度(0~359)分為18bin。*/
		iArr_hist[ iAngle_Hist / 20]++;
	}

	iOFHistNumber = 0;
	for( i = 0; i < 18; i++ )
	{
		iOFHistNumber += iArr_hist[i];
	}

	/*PDF*/
	for ( i=0; i<18; i++ )
	{
		if ( iArr_hist[i] == 0 )
		{
			dArr_HistPDF[i] = 0;
		} 
		else
		{
			dArr_HistPDF[i] = (double)iArr_hist[i] / iOFHistNumber;
		}
	}

	/*CDF*/
	dArr_histCDF[0] = dArr_HistPDF[0];
	for( i = 1; i < 18; i++ )
	{
		dArr_histCDF[i] = dArr_HistPDF[i] + dArr_histCDF[i-1];
	}

	maxSlpoe = 0;
	for (i=0; i<19; i++)
	{
		dSlpoe[i] = 0;
	}
	dSlpoe[0] = dArr_histCDF[0];
	for ( i=1; i<18; i++ )
	{
		dSlpoe[i] = dArr_histCDF[i] - dArr_histCDF[i-1];
	}

	maxSlpoe = dSlpoe[0];
	for ( i=1; i<18; i++)
	{
		if ( dSlpoe[i] > dSlpoe[i-1] )
		{
			maxSlpoe = dSlpoe[i];
		}
	}

	iMax = 0;
	iIndex = 55555;

	for( i = 0; i < 18; i++ )
	{
		/*20130112 Timon 沒有考慮到兩個Bin一樣高的情況, 現在先不考慮了, 這個影響不大, 先往後做吧。*/
		if ( iMax < iArr_hist[i] && iArr_hist[i] > 0 )
		{
			iMax = iArr_hist[i];
			iIndex = i;
		}
	}

	aa = 0;
	bb = 0;
	iDistributionResult = 0;

	for( i = 0; i < 18; i++ )
	{
		if ( iArr_hist[i] >= 30 ) /*第一次是>=101*/ 
			aa++;

		if ( iArr_hist[i] >= 5 )
			bb++;
	}

	if ( aa > 0 )
	{
		if ( maxSlpoe > 0.5 ) /*指數分布*/
		{
			iDistributionResult = 4;
		} 
		else  /*高斯分布*/
		{
			iDistributionResult = 2;
		}
	}
	else
	{
		if ( bb<=5 ) /*無分布*/
		{
			iDistributionResult = 3;
		} 
		else /*均勻分布*/
		{
			iDistributionResult = 1;
		}
	}

	sOnlineCAD->iarrHistMod[0] = sOnlineCAD->iarrHistMod[1];
	sOnlineCAD->iarrHistMod[1] = iIndex+1;

	if ( abs(sOnlineCAD->iarrHistMod[0]-sOnlineCAD->iarrHistMod[1]) == 9 )
	{
		iDistributionResult = 5; /*閃爍光源， 相差180度*/
	}

	return iDistributionResult;
}

void ievCornerDebug(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	int i = 0;

	cvCopyImage( imgSrc, sOnlineCAD->imgCorner );

	for( i = 0; i < sOnlineCAD->iCornerMaxNumber; i++ )
		cvCircle( sOnlineCAD->imgCorner, cvPointFrom32f( sOnlineCAD->point2D32fCurrentCornerSet[i] ), 
		3, CV_RGB( 255, 0, 255 ), -1, 8, 0 );
}

void ievOpticalFlowDebug(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	cvCopyImage( imgSrc, sOnlineCAD->imgOpticalFlow );

	double dRadian_draw = 0.0;
	double dHypotenuse  = 0.0;

	int i=0, j=0;
	int x=0, y=0;
	double dDistance = 0;

	for( i = j = 0; i < sOnlineCAD->iPreviouCornerMaxNumber; i++ )
	{
		/*20130113 Timon  這裡的畫特徵點要再確認j和i的變化有何影響？*/
		if( !sOnlineCAD->status[i] )
			continue; /*continue表示回到上一層迴圈執行，否則向下執行。*/

		j++;

		dDistance = sqrt( double( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) +
			( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) );

		/*在這裡直接過濾移動距離過小的特徵點(包含未移動)。*/
		if ( dDistance > sOnlineCAD->sOnlineCADParam.dOpticalFlowLengthThreshold )
		{
			/*20130111 Timon 這邊的精準度要再確認？*/
			/*還有要確認箭頭有沒有畫反？*/
			/*20130111 Timon 起點到終點。*/
			cvLine( sOnlineCAD->imgOpticalFlow, cvPoint( sOnlineCAD->point2D32fPreviouCornerSet[i].x, sOnlineCAD->point2D32fPreviouCornerSet[i].y ),
				cvPoint( sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x, sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ), CV_RGB( 255, 0,0 ), 1, CV_AA, 0 );

			/*20130112 Timon 計算弧度，為了畫箭頭或延長線段用。*/
			dRadian_draw = atan2( (double) sOnlineCAD->point2D32fPreviouCornerSet[i].y - (double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y, (double) sOnlineCAD->point2D32fPreviouCornerSet[i].x - (double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x );

			/*013011 Timon 把起點到終點的線拉長。並且繪製被拉長的起點到終點的線。*/
			/*20130111 Timon 繪製終點的箭頭(也就是繪製終點的左右兩旁的線)。*/
			/*20130111 Timon 終點的左邊線段。*/
			x = (int) ( sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x + 9 * cos( dRadian_draw + 3.14159265358979323846 / 4 ) );
			y = (int) ( sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y + 9 * sin( dRadian_draw + 3.14159265358979323846 / 4 ) );
			cvLine( sOnlineCAD->imgOpticalFlow, cvPoint(x, y), cvPoint(sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x, sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y), CV_RGB(255,0,0), 1, CV_AA, 0);

			/*20130111 Timon 終點的右邊線段。*/
			x = (int) (sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x + 9 * cos( dRadian_draw - 3.14159265358979323846 / 4 ) );
			y = (int) (sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y + 9 * sin( dRadian_draw - 3.14159265358979323846 / 4 ) );
			cvLine( sOnlineCAD->imgOpticalFlow, cvPoint( x, y ),cvPoint( sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x, sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ), CV_RGB( 255, 0,0 ), 1, CV_AA, 0 );
		}
	}
}

void ievHOOFDegub(EvOnlineCAD *sOnlineCAD, IplImage *imgSrc)
{
	int i=0, j=0, k=0;
	int iMax = 0;
	int iIndex = 0;
	int iAngle_Hist = 0;;
	int iOFHistNumber = 0;
	int iArr_hist[19] = {0}; /*第19Bin儲存未移動點的數量*/

	double dDistance = 0;
	double dRadian_atan2_Hist = 0.0;
	double dAngle_atan2_Hist  = 0.0;
	double dArr_HistPDF[19] = {0};
	double dArr_histCDF[19] = {0}; 

	char cStr[64]  = {0};
	char cStr2[64] = {0};
	char cStr3[64] = {0};

	CvFont Font1;

	sOnlineCAD->imgHOOF->origin = 1;

	/*20130112 Timon 畫角度直方圖*/
	for( i = 0; i < 19; i++ )
	{
		iArr_hist[i] = 0;
	}

	/*計算並且統計特徵點角度*/
	for( i = 0; i < sOnlineCAD->iPreviouCornerMaxNumber; i++ )
	{
		if( !sOnlineCAD->status[i] )
			continue; /*continue表示回到上一層迴圈執行，否則向下執行。*/

		/*20130112 Timon 計算角度*/
		dRadian_atan2_Hist = atan2( (double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y - (double)sOnlineCAD->point2D32fPreviouCornerSet[i].y, 
			(double) sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x - (double) sOnlineCAD->point2D32fPreviouCornerSet[i].x );
		dAngle_atan2_Hist = -dRadian_atan2_Hist * 180 / 3.14159265358979323846;
		iAngle_Hist = (int) ( 360 + dAngle_atan2_Hist ) % 360;

		dDistance = sqrt( double( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].x - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].x ) +
			( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) * ( sOnlineCAD->point2D32fPreviouCornerSet[i].y - sOnlineCAD->point2D32fCurrentOpticalFlowSet[i].y ) );

		/*0130114 Timon 特徵點位置不變， 會被計算成0度， 跟特徵點向右一樣， 因此必須要刪除未移動的點。*/
		/*在這裡直接過濾移動距離過小的特徵點(包含未移動)。*/
		if ( dDistance < sOnlineCAD->sOnlineCADParam.dOpticalFlowLengthThreshold )
		{
			iAngle_Hist = 360; // 後面360/20=18 會存到第19bin。
		}

		/*20130112 Timon 統計角度直方圖, 將360度(0~359)分為18bin。*/
		iArr_hist[ iAngle_Hist / 20]++;
	}

	iOFHistNumber = 0;

	for( i = 0; i < 18; i++ )
	{
		iOFHistNumber += iArr_hist[i];
	}

	/*20130112 Timon 畫角度直方圖*/
	cvZero( sOnlineCAD->imgHOOF );
	j = 10;
	iMax = 0;
	iIndex = 55555;

	for( i = 0; i < 18; i++ )
	{
		cvRectangle( sOnlineCAD->imgHOOF, cvPoint( j, 0 ), cvPoint( j+5, iArr_hist[i] ),
			CV_RGB( 255, 0, 0 ), -1, 8, 0 );
		j += 10 ;

		/*20130112 Timon 沒有考慮到兩個Bin一樣高的情況, 現在先不考慮了, 這個影響不大, 先往後做吧。*/
		if ( iMax < iArr_hist[i] && iArr_hist[i] > 0 )
		{
			iMax = iArr_hist[i];
			iIndex = i;
		}
	}

	if ( iIndex != 55555 )
	{
		sprintf( cStr,"Max Num Bin:%d", iIndex+1 );
		sprintf( cStr2,"Angle is:[%d~%d]", iIndex * 20, ( ( iIndex + 1 ) * 20 ) - 1 );
		sprintf( cStr3,"Max Num:%d", iArr_hist[iIndex] );
		cvInitFont( &Font1, CV_FONT_HERSHEY_SIMPLEX , .6, .6, 0, 1, 8 );
		cvPutText( sOnlineCAD->imgHOOF, cStr , cvPoint( 5, 250 ),&Font1,CV_RGB( 255, 255, 255 ) );
		cvPutText( sOnlineCAD->imgHOOF, cStr2, cvPoint( 5, 230 ),&Font1,CV_RGB( 255, 255, 255 ) );
		cvPutText( sOnlineCAD->imgHOOF, cStr3, cvPoint( 5, 210 ),&Font1,CV_RGB( 255, 255, 255 ) );

		k = 250;
		for ( i = 17; i > -1; i-- )
		{
			if ( iArr_hist[i] == iMax && i != iIndex )
			{
				sprintf( cStr,"%d=%d Bin Num", iIndex+1, i+1 );
				cvInitFont( &Font1, CV_FONT_HERSHEY_SIMPLEX , .6, .6, 0, 1, 8 );
				cvPutText( sOnlineCAD->imgHOOF, cStr , cvPoint( 5, k+=20 ),&Font1,CV_RGB( 255, 255, 255 ) );
			}
		}
	}
	else
	{
		cvInitFont( &Font1, CV_FONT_HERSHEY_SIMPLEX , .6, .6, 0, 1, 8 );
		cvPutText( sOnlineCAD->imgHOOF, "All Bin Num = 0" , cvPoint( 5, 180 ),&Font1,CV_RGB( 255, 255, 255 ) );
	}
}