#include "CRTKPoint.h"
#include "GNSSCmnFunc.h"
#include "BaseCmnFunc.h"
#include "BaseMath.h"
#include <stdlib.h>

///< Constructor function, to be called when defining
void InitCRTKPointBase(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	pStruct->m_pGNSSOpt = NULL;
	pStruct->m_pGNSSSol = NULL;
	pStruct->m_pOBSData[0] = NULL;
	pStruct->m_pOBSData[1] = NULL;

	pStruct->m_bInitRTK = true;
	for (int f = 0; f < NFREQ; f++) pStruct->m_bFreqUsed[f] = true;
	pStruct->m_bUseP = true;
	pStruct->m_bUseL = true;
	pStruct->m_bGoodEpoch = true;

	InitCEphemerisComputer(&pStruct->m_EphComputer);
	InitCSPPoint(&pStruct->m_SPPoint);
	InitCTroposphere(&pStruct->m_TropModel);
	InitCGNSSQualityControl(&pStruct->m_GNSSQC);
	InitCGLSEstimator(&pStruct->m_GLS);
	InitCGNSSIAR(&pStruct->m_RTKIAR);
	InitGNSS_GLSINFO(&pStruct->m_GLSInfo);
	for (int i = 0; i < 2; i++)
	{
		InitCCycleSlipDetection(pStruct->m_CSD + i);
		for (int j = 0; j < NSATMAX + 1; j++)
			InitSATSTATUS(&pStruct->m_SatStatus[i][j]);
		InitGPSTIME(pStruct->m_OBSTime + i);
		InitGPSTIME(pStruct->m_OBSTimePre + i);
	}

	pStruct->m_OBSGapTime[0] = 0.0;
	pStruct->m_OBSGapTime[1] = 0.0;
	pStruct->m_RTKPredictEpoch = 0;
	M31Zero(pStruct->m_BaseXYZ);
	M31Zero(pStruct->m_BaseLLH);
	SetValsI(pStruct->m_BaseSat[0], -1, NFREQ * NSYS);
	SetValsI(pStruct->m_PreBasePrn[0], -1, NFREQ * NSYS);//
	pStruct->m_nMinSatThres = 4;
	M31Zero(pStruct->m_RefXYZ);
}


static int GetAmbLoc(CRTKPointBase* pStruct, int f, int prn)
{
	if (!pStruct)return -1;
	return pStruct->m_GLSInfo.m_AmbIndex[f * NSATMAX + (prn - 1)];
}


static void InitKF_Pos(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	double XYZPos[3] = { 0.0 };
	if (pStruct->m_SPPoint.m_SPPEKFSolType >= GNSSMODE_SPP)
	{
		M31EQU(pStruct->m_SPPoint.m_SPPEKFXYZ, XYZPos);
	}
	else if (pStruct->m_SPPoint.m_SPPSolType == GNSSMODE_SPP)
	{
		M31EQU(pStruct->m_SPPoint.m_SPPXYZ, XYZPos);
	}

	for (int i = 0; i < 3; i++)
	{
		InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IPos + i, XYZPos[i], pStruct->m_pGNSSOpt->BASE_SPPPosSigma2);

		if (pStruct->m_GLSInfo.LVel > 0)
		{
			if (pStruct->m_pGNSSOpt->m_PosEstMode == 3) // 加速度/速度模型(VA-EKF)
			{
				InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IVel + i, pStruct->m_SPPoint.m_VAVel[i], pStruct->m_pGNSSOpt->BASE_ISDV_Vel2);
			}
			else
			{
				InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IVel + i, pStruct->m_SPPoint.m_SPVXYZ[i], pStruct->m_pGNSSOpt->BASE_ISDV_Vel2);
			}

		}

		if (pStruct->m_GLSInfo.LAcc > 0)
		{
			InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IAcc + i, 0.0, pStruct->m_pGNSSOpt->BASE_ISDV_Acc2);
		}
	}
	pStruct->m_pGNSSSol->m_FilterTime = pStruct->m_SPPoint.m_SolTime;
}


static void InitKF(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	InitGNSS_StateIndex(&pStruct->m_GLSInfo, 0/*m_EstTrpType*/, 0/*m_EstIonType*/, false/*m_bReverse*/);
	InitKF_Pos(pStruct);
}


static bool InitRTKBase(CRTKPointBase* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, OBS_DATA* pROBSData, OBS_DATA* pBOBSData)
{
	if (!pStruct)return false;

	///< 1.初始化RTK
	pStruct->m_pGNSSOpt = pGNSSOpt;
	pStruct->m_pGNSSSol = pGNSSSol;
	pStruct->m_pOBSData[IROVE] = pROBSData;
	pStruct->m_pOBSData[IBASE] = pBOBSData;
	for (int i = 0; i < 2; i++)
	{
		initSatStatus(pStruct->m_SatStatus[i], NULL);
		pStruct->m_OBSTime[i] = pStruct->m_pOBSData[i]->gt;
		pStruct->m_OBSGapTime[i] = pStruct->m_pGNSSOpt->m_SampleTime[i];
	}

	///< 2.初始化SPP，第一次SPP一定要成功
	if (InitSPP(&pStruct->m_SPPoint, pStruct->m_pGNSSOpt, pStruct->m_pGNSSSol, &pStruct->m_EphComputer, pStruct->m_SatStatus[IROVE], &pStruct->m_GLS) == false) return false;

	if (runOneSPP(&pStruct->m_SPPoint, pStruct->m_pOBSData[0]) == false) return false;

	if (pStruct->m_SPPoint.m_SPPSolType == GNSSMODE_NONE || pStruct->m_SPPoint.m_SPPDOPs[1] > 5.0 || MatrixTrace(3, 3, pStruct->m_SPPoint.m_SPPXYZP) > 900.0) return false;

	///< 3. 得到基站坐标
	int BaseSiteID = 0;
	if (MatrixNorm2(3, 1, pStruct->m_BaseXYZ) < 10.0) M31EQU(pStruct->m_pGNSSOpt->m_BaseXYZ[BaseSiteID], pStruct->m_BaseXYZ);
	if (MatrixNorm2(3, 1, pStruct->m_BaseXYZ) < 10.0)return false;
	XYZ2LLH(pStruct->m_BaseXYZ, pStruct->m_BaseLLH, 0);

	///< 4. 初始化CSD周跳探测模块
	for (int i = 0; i < 2; i++)
	{
		if (InitCS(&pStruct->m_CSD[i], pStruct->m_pGNSSOpt, pStruct->m_SatStatus[i], i) == false) return false;
	}

	///< 5. 初始化QC数据质量控制模块，PMode=2表示RTK
	InitQC(&pStruct->m_GNSSQC,2, pStruct->m_pGNSSOpt, pStruct->m_SatStatus[IROVE], pStruct->m_pOBSData[IROVE], pStruct->m_GLSInfo.MEQ);

	///< 6. 初始化ILS模糊度固定模块
	InitIAR(&pStruct->m_RTKIAR, pStruct->m_pGNSSOpt, pStruct->m_pGNSSSol, pStruct->m_bFreqUsed);

	///< 7. 广义最小二乘估计器初始化
	InitKF(pStruct);

	return true;
}


///< 初始化RTK
bool InitRTK(CRTKPointBase* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, OBS_DATA* pROBSData, OBS_DATA* pBOBSData)
{
	if (!pStruct)return false;

	if (!pGNSSOpt || !pGNSSSol || !pROBSData || !pBOBSData) return false;

	if (InitRTKBase(pStruct, pGNSSOpt, pGNSSSol, pROBSData, pBOBSData) == false) return false;

	pStruct->m_bInitRTK = false;

	return true;
}


static void DetectCycleSlip(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	if (pStruct->m_pGNSSOpt->m_ARMode == ARMODE_INST) return;

	// 注意清空状态
	for (int i = 0; i < 2; i++)
	{
		for (int prn = 1; prn <= NSATMAX; prn++)
		{
			int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pStruct->m_SatStatus[i][prn].sys, &prn);
			if (skip == 1) continue;
			else if (skip == 2) break;
			for (int f = 0; f < NFREQ; f++)
			{
				pStruct->m_SatStatus[i][prn].CS_Type[f] = 0;
				pStruct->m_SatStatus[i][prn].CS_GFNum[f] = 0;
			}
			pStruct->m_SatStatus[i][prn].CS_bGFMulti = false;
		}
	}

	for (int i = 0; i < 2; i++)
	{
		pStruct->m_CSD[i].m_pOBSData = pStruct->m_pOBSData[i];
		pStruct->m_CSD[i].m_OBSTime = pStruct->m_OBSTime[i];
		pStruct->m_CSD[i].m_OBSGapTime = pStruct->m_OBSGapTime[i];
		pStruct->m_CSD[i].m_nOutEpoch = pStruct->m_GLSInfo.nOutEpoch;
		pStruct->m_CSD[i].m_SampleTime = pStruct->m_pGNSSOpt->m_SampleTime[i];
		for (int f = 0; f < NFREQ; f++) pStruct->m_CSD[i].m_bFreqUsed[f] = pStruct->m_bFreqUsed[f];
		runOneCSD(pStruct->m_CSD + i);

		//// for test
		//if (i == 0)
		//{
		//	static FILE* ftmp = NULL;
		//	if (!ftmp)fopen_s(&ftmp, "E:\\01Prog\\04CDTH\\01Code\\CIPSPRN100CST.txt", "w");
		//	if (ftmp)fprintf(ftmp, "%8.1f,%8.3f,%8.3f,%8.3f,%8.3f\n", 
		//		GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt), 
		//		pStruct->m_SatStatus[0][76].CS_GF[0], 
		//		pStruct->m_SatStatus[0][76].CS_GF[1],
		//		pStruct->m_SatStatus[0][76].CS_GF[2],
		//		pStruct->m_SatStatus[0][76].azel[1] * R2D);
		//}
	}
}


static bool SelectCommonSats(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	// 注意清空状态
	for (int i = 0; i < 2; i++)
	{
		if (i == IBASE)continue;
		for (int prn = 1; prn <= NSATMAX; prn++)
		{
			int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pStruct->m_SatStatus[i][prn].sys, &prn);
			if (skip == 1) continue;
			else if (skip == 2) break;
			for (int f = 0; f < NFREQ; f++)
			{
				pStruct->m_SatStatus[i][prn].SD_L[f] = 0;
			}
		}
	}

	int jSat = 0, prn = 0, nCom = 0;

	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		pStruct->m_GLSInfo.MEQ[i].OBSIndex = -1;
		prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
		if ((pStruct->m_SatStatus[IROVE][prn].sys & pStruct->m_pGNSSOpt->m_SatSYS) == 0)continue;

		for (int j = jSat; j < pStruct->m_pOBSData[IBASE]->nsat; j++)
		{
			if (pStruct->m_pOBSData[IROVE]->obs[i].prn == pStruct->m_pOBSData[IBASE]->obs[j].prn)
			{
				pStruct->m_GLSInfo.MEQ[i].OBSIndex = j;
				for (int f = 0; f < NFREQ; f++)
				{
					pStruct->m_SatStatus[IROVE][prn].SD_L[f] = pStruct->m_pOBSData[IROVE]->obs[i].L[f] - pStruct->m_pOBSData[IBASE]->obs[j].L[f];
				}
				//jSat = j + 1;
				nCom++;
				break;
			}
		}
	}

	return (nCom >= pStruct->m_nMinSatThres);
}


static void ZeroSatStatus(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	SATSTATUS* pSatStatus = NULL;

	// 注意，SPP中定义SatStatus指针，指向RTK中流动站，两者互操作需理清
	// SPP需要传向RTK的变量应该保留：az/el/OBSConsist/OBSEVENT中OBSEVT_Resi_v/OBSEVT_XDOP标识
	for (int j = 0; j < 2; j++)
	{
		pSatStatus = pStruct->m_SatStatus[j];

		for (int prn = 1; prn <= NSATMAX; prn++)
		{
			pSatStatus[prn].bExclude = 0;   //0:正常
			if ((pSatStatus[prn].sys & pStruct->m_pGNSSOpt->m_SatSYS) == 0) pSatStatus[prn].bExclude |= 1;

			int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pSatStatus[prn].sys, &prn);
			if (skip == 1) continue;
			else if (skip == 2) break;

			pSatStatus[prn].Satsvh = -1;//0:健康
			pSatStatus[prn].AR_AmbCtrl.bFixWL = false;

			pSatStatus[prn].OBSIndex = -1;// 在观测值列表中的位置

			for (int f = 0; f < NFREQ; f++)
			{
				pSatStatus[prn].S[f] = 0.0;

				for (int i = 0; i < 3; i++)
				{
					pSatStatus[prn].OBSValid[i][f] = false;
				}

				pSatStatus[prn].Amb0[f] = 0.0;//模糊度初值,伪距减去相位

				pSatStatus[prn].AR_Status[f] = ARSTATE_NONE;
				pSatStatus[prn].AR_Amb[f] = 0;
				pSatStatus[prn].AR_AmbCtrl.vL[f] = 0.0;
				pSatStatus[prn].AR_AmbCtrl.NL[f] = 0.0;

				if (j == 0)
				{
					if (pSatStatus[prn].OBSEvent[OBS_LI][f] & OBSEVT_Resi_v)
						pSatStatus[prn].CS_Type[f] |= CST_TDCP;
					if (pSatStatus[prn].OBSEvent[OBS_LI][f] & OBSEVT_XDOP)
						pSatStatus[prn].CS_Type[f] |= CST_LLI;
				}
			}
		}
	}

	for (int k = 0; k < 2; k++)
	{
		for (int i = 0; i < pStruct->m_pOBSData[k]->nsat; i++)
		{
			const int prn = pStruct->m_pOBSData[k]->obs[i].prn;

			// 得到prn卫星的观测值索引
			pStruct->m_SatStatus[k][prn].OBSIndex = i;

			// 信噪比S赋值
			for (int f = 0; f < NFREQ; f++) pStruct->m_SatStatus[k][prn].S[f] = pStruct->m_pOBSData[k]->obs[i].S[f];
		}
	}
}


static void getSatelliteOrbits(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	int prn = 0;
	OBS_DATA* pOBSData = NULL;
	SATSTATUS* pSatStatus = NULL;
	double Pr = 0.0;
	GPSTIME satTime;

	for (int j = 0; j < 2; j++)
	{
		if (j == 0) { pOBSData = pStruct->m_pOBSData[IROVE]; pSatStatus = pStruct->m_SatStatus[IROVE]; }
		else		{ pOBSData = pStruct->m_pOBSData[IBASE]; pSatStatus = pStruct->m_SatStatus[IBASE]; }

		// 计算卫星位置
		for (int i = 0; i < pOBSData->nsat; i++)
		{
			const OBS_DATA_t* pOBSi = &(pOBSData->obs[i]);
			prn = pOBSi->prn;
			pSatStatus[prn].Satsvh = -1;
			if (pSatStatus[prn].bExclude) continue;

			for (int f = 0; f < NFREQ; f++)
			{
				if (pStruct->m_bFreqUsed[f] == false) continue;
				if ((Pr = pOBSi->P[f]) != 0.0) break;
			}

			if (IsEqual(Pr, 0.0)) continue;

			// 得到卫星信号发射时刻,计算卫星位置, 此处只要误差总和小于100m, 引起的卫星准确发射时刻小于1us, 卫星位置精度小于1mm
			satTime = MinusGPSTIMESec(pOBSData->gt, (Pr / CLIGHT));

			EphComputer(&pStruct->m_EphComputer, satTime, prn, EPH_BRDC);

			M31EQU(pStruct->m_EphComputer.m_SatPos, pSatStatus[prn].SatPos);
			M31EQU(pStruct->m_EphComputer.m_SatVel, pSatStatus[prn].SatVel);
			pSatStatus[prn].SatClk = pStruct->m_EphComputer.m_SatClk;
			pSatStatus[prn].SatClkVel = pStruct->m_EphComputer.m_SatClkVel;
			pSatStatus[prn].Satsvh = pStruct->m_EphComputer.m_svh;
			if (pSatStatus[prn].Satsvh != 0) pSatStatus[prn].bExclude |= 4;
		}
	}
}


static void UpdataAmbIndex(int index, int nAmbIndex, int AmbIndex[])
{
	int j;
	for (j = 0; j < nAmbIndex; j++)
	{
		if (AmbIndex[j] < 0)            continue;
		if (index < AmbIndex[j])        AmbIndex[j]--;
		else if (index == AmbIndex[j])  AmbIndex[j] = -1;
	}
}


static bool JudgeDsat(int index, int nAmbUsed, int AmbUsed[])
{
	int i;
	for (i = 0; i < nAmbUsed; i++)
	{
		if (index == AmbUsed[i]) return true;
	}
	return false;
}


static void UpdataState(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	 //假设这里进来的状态矩阵StateX大小为MAXRTKNX，状态方差矩阵StateP大小为MAXRTKNX*MAXRTKNX
	 //状态矩阵只有前n维非0，状态方差矩阵只有大小为n*n维的子矩阵非0.
	 //每个模糊度参数对应的索引i，存放在m_AmbIndex[NSATMAX*NFREQ]中，该数组内空置处存放-1.
	 //在该函数内完成卫星的增减工作：对于减少卫星，将对应行列剔除；对应增加卫星，在n*n维子矩阵最后进行添加.

	int AmbUsed[MAXOBSLIM], nAmbUsed = 0;	// 当前历元待估计模糊度参数，根据共视卫星及相位观测值情况选出
	int i, j = 0, k, prn, num = 0;			// num: StateX中要估计的模糊度参数个数，用于增加星的处理中

	///< 1.将状态矩阵非0行进行剔除，压实矩阵
	for (i = pStruct->m_GLSInfo.IAmb; i < pStruct->m_GLSInfo.EAmb; i++)
	{
		if (pStruct->m_GLSInfo.StateP[i * MAXRTKNX + i] > 0.0) j = i;
	}
	k = j;
	for (i = pStruct->m_GLSInfo.IAmb; i < j; i++)
	{
		if (pStruct->m_GLSInfo.StateP[i * MAXRTKNX + i] > 0.0) continue;
		MatrixDelRC(MAXRTKNX, 1, i + 1, 1, pStruct->m_GLSInfo.StateX);
		MatrixDelRC(MAXRTKNX, MAXRTKNX, i + 1, i + 1, pStruct->m_GLSInfo.StateP);
		UpdataAmbIndex(i, NSATMAX * NFREQ, pStruct->m_GLSInfo.m_AmbIndex);
		j--; //最后一个非0行索引前移
	}

	///< 2.确定当前历元待估计模糊度参数
	for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		if (pMEQ->OBSIndex < 0) continue; // 非共视卫星
		prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
		for (j = 0; j < NFREQ; j++)
		{
			if (!pStruct->m_bFreqUsed[j]) continue;
			if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[j]) continue;
			if (pStruct->m_pOBSData[IROVE]->obs[i].L[j] == 0.0 || pStruct->m_pOBSData[IBASE]->obs[pMEQ->OBSIndex].L[j] == 0.0) continue;
			AmbUsed[nAmbUsed++] = prn - 1 + NSATMAX * j;
		}
	}

	///< 3.处理减少星，原则为剔除对应行、列
	for (i = 0; i < NSATMAX * NFREQ; i++)
	{
		if (pStruct->m_GLSInfo.m_AmbIndex[i] < 0) continue; // 非前一历元估计参数
		else if (pStruct->m_GLSInfo.m_AmbIndex[i] > k)
		{
			pStruct->m_GLSInfo.m_AmbIndex[i] = -1;
			continue;
		}
		num++;
		if (JudgeDsat(i, nAmbUsed, AmbUsed)) continue; // 非减少星
		num--;
		MatrixDelRC(MAXRTKNX, 1, pStruct->m_GLSInfo.m_AmbIndex[i] + 1, 1, pStruct->m_GLSInfo.StateX);
		MatrixDelRC(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.m_AmbIndex[i] + 1, pStruct->m_GLSInfo.m_AmbIndex[i] + 1, pStruct->m_GLSInfo.StateP);
		UpdataAmbIndex(pStruct->m_GLSInfo.m_AmbIndex[i], NSATMAX * NFREQ, pStruct->m_GLSInfo.m_AmbIndex);
	}

	///< 4.处理增加星
	for (i = 0; i < nAmbUsed; i++)
	{
		if (pStruct->m_GLSInfo.m_AmbIndex[AmbUsed[i]] < 0)
		{
			pStruct->m_GLSInfo.m_AmbIndex[AmbUsed[i]] = pStruct->m_GLSInfo.IAmb + num;
			num++;
		}
	}
}


static void ReinitAmb(CRTKPointBase* pStruct, int prn, int frq)
{
	if (!pStruct)return;

	int ambLoc;
	if ((ambLoc = GetAmbLoc(pStruct, frq, prn)) < pStruct->m_GLSInfo.IAmb) return;
	double amb = pStruct->m_SatStatus[IROVE][prn].Amb0[frq];
	double var = pStruct->m_pGNSSOpt->RTK_ISDV_Amb2;
	if (amb == 0.0) var = 0.0;

	// 只有长基线的时候,模糊度单位为m,其它为周
	if (pStruct->m_pGNSSOpt->m_BaselineType != 1) 
		var /= pStruct->m_SatStatus[IROVE][prn].lam[frq] * pStruct->m_SatStatus[IROVE][prn].lam[frq];

	InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, ambLoc, amb, var);
	InitStateX(pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.MKFStateP, MAXRTKNX, ambLoc, amb, var);

	// 重置了模糊度
	pStruct->m_SatStatus[IROVE][prn].bResetAmb[frq] = true;

	pStruct->m_SatStatus[IROVE][prn].lock[frq] = 0;
	//pStruct->m_SatStatus[IROVE][prn].CS_dGF[frq] = 0;
	//pStruct->m_SatStatus[IROVE][prn].CS_nMW[frq] = 0; // MW周跳信息重置

	pStruct->m_SatStatus[IBASE][prn].lock[frq] = 0;
	//pStruct->m_SatStatus[IBASE][prn].CS_dGF[frq] = 0;
	//pStruct->m_SatStatus[IBASE][prn].CS_nMW[frq] = 0; // MW周跳信息重置
}


static void ReinitAllAmb(CRTKPointBase* pStruct, int frq)
{
	if (!pStruct)return;

	// 提供两种重新初始化,1:初始化所有卫星,2:初始化当前观测到的卫星
	int prn, iDF[2];
	if (TFLC_IFIndex3((TFLCTYPE)frq, iDF) == false)return;

	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

		ReinitAmb(pStruct, prn, frq);

		// 重置GF和MW
		if (true/*pStruct->m_pGNSSOpt->pStruct->m_bCS_MW*/)
		{
			// 多频数据对frq直接赋值有Bug，暂时关闭MW赋初值
			//double MW = TFLC_MW_LP(lam, pStruct->m_iFreq, pOBSDatai->L, pOBSDatai->P);
			double MW = 0.0;
			if (MW != 0.0)
			{
				//pStruct->m_SatStatus[IROVE][prn].CS_nMW[frq] = 1;
				//pStruct->m_SatStatus[IROVE][prn].CS_MW[frq] = MW;

				//pStruct->m_SatStatus[IBASE][prn].CS_nMW[frq] = 1;
				//pStruct->m_SatStatus[IBASE][prn].CS_MW[frq] = MW;
			}
			else
			{
				pStruct->m_SatStatus[IROVE][prn].CS_nMW[iDF[0]] = 0;
				pStruct->m_SatStatus[IROVE][prn].CS_MW[iDF[0]] = 0.0;
				pStruct->m_SatStatus[IROVE][prn].CS_nMW[iDF[1]] = 0;
				pStruct->m_SatStatus[IROVE][prn].CS_MW[iDF[1]] = 0.0;

				pStruct->m_SatStatus[IBASE][prn].CS_nMW[iDF[0]] = 0;
				pStruct->m_SatStatus[IBASE][prn].CS_MW[iDF[0]] = 0.0;
				pStruct->m_SatStatus[IBASE][prn].CS_nMW[iDF[1]] = 0;
				pStruct->m_SatStatus[IBASE][prn].CS_MW[iDF[1]] = 0.0;
			}

			pStruct->m_SatStatus[IROVE][prn].CS_dGF[iDF[0]] = 0;
			pStruct->m_SatStatus[IROVE][prn].CS_dGF[iDF[1]] = 0;

			pStruct->m_SatStatus[IBASE][prn].CS_dGF[iDF[0]] = 0;
			pStruct->m_SatStatus[IBASE][prn].CS_dGF[iDF[1]] = 0;
		}
	}
}


static bool AnalyzeAllAmbCS(CRTKPointBase* pStruct, bool bNewCS, int frq, bool bGap)
{
	if (!pStruct)return false;

	const bool bReinitOpt = true;

	pStruct->m_GLSInfo.bReinitAllAmb[frq] = false;
	pStruct->m_GNSSQC.m_bReinitAllAmb[frq] = false;
	pStruct->m_RTKIAR.m_bReinitAllAmb[frq] = false;
	int nrei[2] = { 0 }, nsat[2] = { 0 }, nout[2] = { 0 }, prn = 0;
	OBS_DATA_t* pOBS_rove;

	// 统计流动站周跳个数
	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		pOBS_rove = &(pStruct->m_pOBSData[IROVE]->obs[i]);
		prn = pOBS_rove->prn;

		if (pStruct->m_SatStatus[IROVE][prn].bExclude)		continue;
		if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[frq])	continue;
		if (pOBS_rove->L[frq] == 0.0)continue;

		nsat[0]++;
		if (pStruct->m_SatStatus[IROVE][prn].outc[frq] > 3)	nout[0]++;
		if (pStruct->m_SatStatus[IROVE][prn].CS_Type[frq])	nrei[0]++;
	}

	// 统计基准站周跳个数
	for (int i = 0; i < pStruct->m_pOBSData[IBASE]->nsat; i++)
	{
		pOBS_rove = &(pStruct->m_pOBSData[IBASE]->obs[i]);
		prn = pOBS_rove->prn;

		if (pStruct->m_SatStatus[IBASE][prn].bExclude) continue;
		if (pOBS_rove->L[frq] == 0.0)continue;

		nsat[1]++;
		if (pStruct->m_SatStatus[IBASE][prn].outc[frq] > 3)	nout[1]++;
		if (pStruct->m_SatStatus[IBASE][prn].CS_Type[frq])	nrei[1]++;
	}

	// 数据断链或周跳过多引起的模糊度初始化
	bool b0 = false, b1 = false, b2 = false, b3 = false, b4 = false;
	if (bNewCS)
	{
		b1 = (nrei[0] * 2 + 1 > nsat[0]) && nrei[0] >= 2;
		b2 = nsat[0] - nrei[0] <= 4 && nrei[0] >= 2;
	}
	else
	{
		b0 = (nsat[0] == nout[0]) && (pStruct->m_GLSInfo.nOutEpoch >= 8 || fabs(pStruct->m_OBSGapTime[IROVE]) > 200);
		b1 = (nrei[0] * 1.5 + 1 > nsat[0]) && nrei[0] >= 2;
		b3 = (nsat[1] == nout[1]) && (pStruct->m_GLSInfo.nOutEpoch >= 8 || fabs(pStruct->m_OBSGapTime[IBASE]) > 200);
		b4 = (nrei[1] * 1.5 + 1 > nsat[1]) && nrei[1] >= 2;
	}

	bool bAllInit = b0 || b1 || b2 || b3 || b4;

	// TDCP测速失败也认为是周跳太多，需重置所有模糊度
	if (pStruct->m_SPPoint.m_VAStateX[pStruct->m_SPPoint.m_VAIClkVel] == 0.0) bAllInit = true;

	// 只进行失锁情况的检测
	if (bGap) bAllInit = b0;

	if (bAllInit)
	{
		pStruct->m_GLSInfo.bReinitAllAmb[frq] = true;
		pStruct->m_GNSSQC.m_bReinitAllAmb[frq] = true;   // 赋给残差分析,周跳全部初始化
		pStruct->m_RTKIAR.m_bReinitAllAmb[frq] = true;   // 给IAR设置重新初始化标识

		if (bReinitOpt)
		{
			ReinitAllAmb(pStruct, frq);
			return bAllInit;
		}
	}

	return false;
}


static double SD_OBS(const OBS_DATA_t* pOBS_rove, const OBS_DATA_t* pOBS_base, int freq, int OBSType)
{
	double sd = 0.0;

	if (OBSType == OBS_LI) // 相位
	{
		if (pOBS_rove->L[freq] != 0.0 && pOBS_base->L[freq] != 0.0)
		{
			sd = pOBS_rove->L[freq] - pOBS_base->L[freq];
			if (sd == 0.0) sd = IPS_EPSILON; // 防止等于0
		}
	}
	else if (OBSType == OBS_PI) // 伪距
	{
		if (pOBS_rove->P[freq] != 0.0 && pOBS_base->P[freq] != 0.0)
		{
			sd = pOBS_rove->P[freq] - pOBS_base->P[freq];
			if (sd == 0.0) sd = IPS_EPSILON; // 防止等于0
		}
	}
	else if (OBSType == OBS_DI) // 多普勒
	{
		if (pOBS_rove->D[freq] != 0.0 && pOBS_base->D[freq] != 0.0)
		{
			sd = pOBS_rove->D[freq] - pOBS_base->D[freq];
			if (sd == 0.0) sd = IPS_EPSILON; // 防止等于0
		}
	}
	return sd;
}


static void KFPre_Amb(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	int prn = 0, ambLoc = 0, MaxOutage = pStruct->m_pGNSSOpt->m_MaxOutage;
	bool breset = false;
	pStruct->m_CSD[IROVE].m_bReinitCJByAmb = true;   ///< 只要所有模糊度重新初始化,CJ的状态也初始化
	pStruct->m_CSD[IBASE].m_bReinitCJByAmb = true;

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		for (prn = 1; prn <= NSATMAX; prn++)
		{
			int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pStruct->m_SatStatus[IROVE][prn].sys, &prn);
			if (skip == 1) continue;
			else if (skip == 2) break;

			// 卫星失锁++,要加在这里,提前判断是否要重置模糊度
			pStruct->m_SatStatus[IROVE][prn].outc[f] += pStruct->m_GLSInfo.nOutEpoch;
			pStruct->m_SatStatus[IBASE][prn].outc[f] += pStruct->m_GLSInfo.nOutEpoch;

			if ((ambLoc = GetAmbLoc(pStruct, f, prn)) < pStruct->m_GLSInfo.IAmb) continue;

			// 失锁历元过多,重新初始化,周跳重新初始化
			// pStruct->m_SatStatus[prn].bresetamb[0]的作用是:如果当前模糊度被初始化,但没有参与解算,那么下一时刻继续初始化,李盼没有这么做
			breset = false;
			if (pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] > 0.0)
			{
				if (true/*pStruct->m_SatStatus[IROVE][prn].CS_Fixed[f] == false*/)
				{
					if (pStruct->m_SatStatus[IROVE][prn].outc[f] > MaxOutage || pStruct->m_SatStatus[IROVE][prn].CS_Type[f])
						breset = true;
				}

				if (true/*pStruct->m_SatStatus[IBASE][prn].CS_Fixed[f] == false*/)
				{
					if (pStruct->m_SatStatus[IBASE][prn].outc[f] > MaxOutage || pStruct->m_SatStatus[IBASE][prn].CS_Type[f])
						breset = true;
				}
			}
			else // ambLoc有值，StateP为零说明是重新跟踪卫星
			{
				pStruct->m_SatStatus[IROVE][prn].lock[f] = 0;
			}
			breset = breset || pStruct->m_SatStatus[IROVE][prn].bResetAmb[f];

			//if (fabs(GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt) - 277221.0) < 0.1)
			//{
			//	if (prn == 76 /*&& f == 1*/)
			//		breset = true;
			//}

			if (breset)
			{
				pStruct->m_SatStatus[IROVE][prn].lock[f] = 0;
				InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, ambLoc, 0.0, 0.0);
			}

			// 单历元固定
			if (pStruct->m_pGNSSOpt->m_ARMode == ARMODE_INST && pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] > 0.0)
			{
				InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, ambLoc, 0.0, 0.0);
			}

			// 模糊度随机游走
			if (pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] > 0.0)
			{
				pStruct->m_CSD[IROVE].m_bReinitCJByAmb = false;
				pStruct->m_CSD[IBASE].m_bReinitCJByAmb = false;

				double sdv2 = 1E-8/*pStruct->m_pGNSSOpt->pStruct->m_GSP.RTK_PNSD_Amb[sys_id]*/;

				// 短基线谱密度单位为周,使用1e-5比较好,如果对流层和电离层消的不干净,可以用1e-4
				pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] += sdv2 * pStruct->m_OBSGapTime[IROVE];
			}

			if (pStruct->m_SatStatus[IROVE][prn].outc[f] > MaxOutage)
			{
				pStruct->m_SatStatus[IROVE][prn].lock[f] = 0;
			}

			if (pStruct->m_SatStatus[IBASE][prn].outc[f] > MaxOutage)
			{
				pStruct->m_SatStatus[IBASE][prn].lock[f] = 0;
			}

			if (pStruct->m_pGNSSOpt->m_ARMode == ARMODE_INST && pStruct->m_SatStatus[IROVE][prn].outc[f] > 1)
			{
				pStruct->m_SatStatus[IROVE][prn].lock[f] = 0;
			}

			if (pStruct->m_pGNSSOpt->m_ARMode == ARMODE_INST && pStruct->m_SatStatus[IBASE][prn].outc[f] > 1)
			{
				pStruct->m_SatStatus[IBASE][prn].lock[f] = 0;
			}
		}
	}

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		// 这里不分析比较好,因为是周跳的话,通过GF+MW一般能够确定
		// 而对于验后残差分析得到的周跳,还是分析比较好,因为不一定能够找出正确的周跳
		// 这里只对中断引起的初始化分析
		bool bAllInit = /*AnalyzeAllAmbCS(pStruct, false, f, false)*/false;
		if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
		{
			pStruct->m_pGNSSSol->m_LOG.bReinitAmb[f] = bAllInit;
		}

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->OBSIndex < 0) continue;

			OBS_DATA_t* pOBS_rove = &(pStruct->m_pOBSData[IROVE]->obs[i]);
			OBS_DATA_t* pOBS_base = &(pStruct->m_pOBSData[IBASE]->obs[pMEQ->OBSIndex]);

			prn = pOBS_rove->prn;
			if (pStruct->m_SatStatus[IROVE][prn].bExclude) continue;

			double sdL = SD_OBS(pOBS_rove, pOBS_base, f, OBS_LI);
			double sdP = SD_OBS(pOBS_rove, pOBS_base, f, OBS_PI);

			double lam = pStruct->m_SatStatus[IROVE][prn].lam[f];

			// 模糊度初值,相位减去伪距,以周为单位,方便模糊度固定
			double amb = 0.0;

			if (sdL * sdP != 0.0)
			{
				amb = sdL - sdP / lam;
			}

			pStruct->m_SatStatus[IROVE][prn].Amb0[f] = amb;

			if ((ambLoc = GetAmbLoc(pStruct, f, prn)) < pStruct->m_GLSInfo.IAmb) continue;

			if (pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] == 0.0)
			{
				pStruct->m_SatStatus[IROVE][prn].bResetAmb[f] = true;

				double var = pStruct->m_pGNSSOpt->RTK_ISDV_Amb2 / (lam * lam);
				if (amb == 0.0) var = 0.0;

				pStruct->m_GLSInfo.StateX[ambLoc] = amb;
				pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] = var;
			}
		}
	}
}


static void KFPre_Pos(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	if (pStruct->m_bInitRTK)
	{
		MatrixZero(MAXRTKNX, 1, pStruct->m_GLSInfo.StateX);
		MatrixZero(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP);
		SetValsI(pStruct->m_GLSInfo.m_AmbIndex, -1, NSATMAX * NFREQ);
		pStruct->m_bInitRTK = false;
	}

	if (MatrixNorm2(3, 1, pStruct->m_GLSInfo.StateX) == 0.0)return;

	if (pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_STATIC)return;

	if (pStruct->m_pGNSSOpt->m_PosEstMode == 0) // 白噪声估计
	{
		for (int i = 0; i < 3; i++)
		{
			InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IPos + i, pStruct->m_GLSInfo.m_Pos0[i], pStruct->m_pGNSSOpt->BASE_SPPPosSigma2);
		}
		pStruct->m_GLSInfo.m_StateSolType = pStruct->m_GLSInfo.m_Pos0Type;
	}
	else if (pStruct->m_pGNSSOpt->m_PosEstMode == 1) // 常量随机游走
	{
		for (int i = 0; i < 3; i++)
		{
			pStruct->m_GLSInfo.StateP[(pStruct->m_GLSInfo.IPos + i) * MAXRTKNX + pStruct->m_GLSInfo.IPos + i] += pStruct->m_pGNSSOpt->BASE_PNSD_Pos2 * pStruct->m_OBSGapTime[IROVE];
		}
		pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_RTK_PRE;
	}
	else if (pStruct->m_pGNSSOpt->m_PosEstMode == 3) // 加速度/速度模型（VA-EKF）
	{
		// TODO:
		if (pStruct->m_SPPoint.m_VAEKFSolType > 0 && pStruct->m_OBSGapTime[IROVE] < 5
			&& fabs(GetGPSTIMESow(pStruct->m_OBSTime[IROVE]) - GetGPSTIMESow(pStruct->m_pGNSSSol->m_FilterTime) - pStruct->m_OBSGapTime[IROVE]) < 0.5 )
		{
			for (int i = 0; i < 3; i++)
			{
				pStruct->m_GLSInfo.StateX[i] += pStruct->m_SPPoint.m_VAVel[i] * pStruct->m_OBSGapTime[IROVE];
				for (int j = 0; j < 3; j++)
				{
					pStruct->m_GLSInfo.StateP[i * MAXRTKNX + j] += pStruct->m_SPPoint.m_VAVelP[i * 3 + j];
				}
			}

			pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_RTK_PRE;
		}
		else if (pStruct->m_GLSInfo.m_Pos0Type > SOLTYPE_NONE)
		{
			double	ambmat[MAXOBSLIM * MAXOBSLIM];
			int s1[2] = { 3, 3 };
			int s2[2] = { 0, 0 };
			MatrixCopySub(s1, s2, MAXOBSLIM, MAXOBSLIM, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, MAXOBSLIM, MAXOBSLIM, ambmat, false);
			MatrixZero(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP);
			for (int i = 0; i < 3; i++)
			{
				InitStateX(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX, pStruct->m_GLSInfo.IPos + i, pStruct->m_GLSInfo.m_Pos0[i], pStruct->m_pGNSSOpt->BASE_SPPPosSigma2);
			}
			MatrixCopySub(s2, s1, MAXOBSLIM, MAXOBSLIM, MAXOBSLIM, MAXOBSLIM, ambmat, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, false);

			pStruct->m_GLSInfo.m_StateSolType = pStruct->m_GLSInfo.m_Pos0Type;
		}
		else pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_NONE;
	}
	//else if (pStruct->m_pGNSSOpt->m_PosEstMode == 2) // Dis/Vel驱动模型
}


static void KFPre(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	// 顺序不能调换
	KFPre_Amb(pStruct);
	KFPre_Pos(pStruct);
}


static bool PrepareRTK(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	///< 1. 初始化
	for (int i = 0; i < 2; i++)
	{
		pStruct->m_OBSTime[i] = pStruct->m_pOBSData[i]->gt;

		if (pStruct->m_OBSTimePre[i].GPSWeek < 0)
		{
			pStruct->m_OBSGapTime[i] = fabs(pStruct->m_pGNSSOpt->m_SampleTime[i]);
		}
		else
		{
			pStruct->m_OBSGapTime[i] = fabs(MinusGPSTIME(pStruct->m_OBSTime[i], pStruct->m_OBSTimePre[i]));
		}

		pStruct->m_OBSTimePre[i] = pStruct->m_OBSTime[i];
	}

	pStruct->m_GapTime = fabs(MinusGPSTIME(pStruct->m_pOBSData[IBASE]->gt, pStruct->m_pOBSData[IROVE]->gt));
	pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_NONE;
	pStruct->m_RTKIAR.m_AmbState = ARSTATE_NONE;

	M31EQU(pStruct->m_pGNSSOpt->m_BaseXYZ[pStruct->m_pGNSSOpt->m_BaseSiteID], pStruct->m_BaseXYZ);
	XYZ2LLH(pStruct->m_BaseXYZ, pStruct->m_BaseLLH, 0);

	// 得到outc累加的值
	pStruct->m_GLSInfo.nOutEpoch = RoundNum(fabs(pStruct->m_OBSGapTime[IROVE] / pStruct->m_pGNSSOpt->m_SampleTime[IROVE]));
	if (pStruct->m_pGNSSOpt->m_SampleTime[IROVE] <= 1.5)
	{
		if (fabs(pStruct->m_OBSGapTime[IROVE]) <= 10.0)		 pStruct->m_GLSInfo.nOutEpoch = 1;
		else if (fabs(pStruct->m_OBSGapTime[IROVE]) <= 15.0) pStruct->m_GLSInfo.nOutEpoch = 2;
		else if (fabs(pStruct->m_OBSGapTime[IROVE]) <= 22.0) pStruct->m_GLSInfo.nOutEpoch = 3;
	}

	/// 2. 周跳探测
	DetectCycleSlip(pStruct);

	/// 3. 选取共视卫星
	SelectCommonSats(pStruct);

	/// 5. 进行SPP解算
	M31EQU(pStruct->m_GLSInfo.StateX, pStruct->m_SPPoint.m_PriorPosRTK); // SPP测速也需要先验位置
	pStruct->m_SPPoint.m_PriorPosRTKTime = pStruct->m_pGNSSSol->m_FilterTime;
	if (runOneSPP(&pStruct->m_SPPoint, pStruct->m_pOBSData[IROVE]) == false)return false;

	if (pStruct->m_SPPoint.m_SPPEKFSolType >= GNSSMODE_SPP)
	{
		M31EQU(pStruct->m_SPPoint.m_SPPEKFXYZ, pStruct->m_GLSInfo.m_Pos0);
		M33EQU(pStruct->m_SPPoint.m_SPPEKFXYZP, pStruct->m_GLSInfo.m_PosVar0);
		pStruct->m_GLSInfo.m_Pos0Type = SOLTYPE_SPP_EKF;
	}
	else if (pStruct->m_SPPoint.m_SPPSolType == GNSSMODE_SPP)
	{
		M31EQU(pStruct->m_SPPoint.m_SPPXYZ, pStruct->m_GLSInfo.m_Pos0);
		M33EQU(pStruct->m_SPPoint.m_SPPXYZP, pStruct->m_GLSInfo.m_PosVar0);
		pStruct->m_GLSInfo.m_Pos0Type = SOLTYPE_SPP;
	}
	else
	{
		//M31Zero(pStruct->m_GLSInfo.pStruct->m_Pos0);
		M33Zero(pStruct->m_GLSInfo.m_PosVar0);
		pStruct->m_GLSInfo.m_PosVar0[0] = pStruct->m_GLSInfo.m_PosVar0[4] = pStruct->m_GLSInfo.m_PosVar0[8] = 10000;
		pStruct->m_GLSInfo.m_Pos0Type = SOLTYPE_NONE;
	}

	/// 6. 无差分数据输出单点坐标
	//pStruct->m_GapTime = DTTOL_M + 1;
	if (pStruct->m_GapTime > DTTOL_M)
	{
		if (pStruct->m_pGNSSSol)
		{
			pStruct->m_pGNSSSol->m_time = pStruct->m_OBSTime[IROVE];
			pStruct->m_pGNSSSol->m_nsat = pStruct->m_pOBSData[IROVE]->nsat;
			pStruct->m_pGNSSSol->m_PDOP = pStruct->m_SPPoint.m_SPPDOPs[1];
			pStruct->m_pGNSSSol->m_DDOP = pStruct->m_SPPoint.m_SPPDOPs[1] * pStruct->m_SPPoint.m_SPPDOPs[1];
			pStruct->m_pGNSSSol->m_HDOP = pStruct->m_SPPoint.m_SPPDOPs[2];
			pStruct->m_pGNSSSol->m_VDOP = pStruct->m_SPPoint.m_SPPDOPs[3];
			//pStruct->m_pGNSSSol->pStruct->m_bKS = 1;
			pStruct->m_pGNSSSol->m_pSatStatus = pStruct->m_SatStatus[IROVE];
			pStruct->m_pGNSSSol->m_pOBSData = pStruct->m_pOBSData[IROVE];
			M31EQU(pStruct->m_GLSInfo.m_Pos0, pStruct->m_pGNSSSol->m_XYZPos);
			XYZ2LLH(pStruct->m_GLSInfo.m_Pos0, pStruct->m_pGNSSSol->m_LLHPos, 0);
			M33EQU(pStruct->m_GLSInfo.m_PosVar0, pStruct->m_pGNSSSol->m_XYZPosP);
			pStruct->m_pGNSSSol->m_SolType = pStruct->m_GLSInfo.m_Pos0Type;
			pStruct->m_pGNSSSol->m_AmbState = ARSTATE_NONE;

			// 其他测速模式需补充
			if (pStruct->m_pGNSSOpt->m_PosEstMode == 3)
			{
				pStruct->m_pGNSSSol->m_VelType = pStruct->m_SPPoint.m_VAEKFSolType;
				M31EQU(pStruct->m_SPPoint.m_VAVel, pStruct->m_pGNSSSol->m_XYZVel);
				M33EQU(pStruct->m_SPPoint.m_VAVelP, pStruct->m_pGNSSSol->m_XYZVelP);
			}
			//else
			//{
			//	pStruct->m_pGNSSSol->pStruct->m_VelType = pStruct->m_SPPoint.pStruct->m_dXYZType;
			//	M31EQU(pStruct->m_SPPoint.pStruct->m_dXYZ, pStruct->m_pGNSSSol->pStruct->m_XYZVel);
			//	M33EQU(pStruct->m_SPPoint.pStruct->m_dXYZVar, pStruct->m_pGNSSSol->pStruct->m_XYZVelP);
			//}
		}
		return true;
	}

	// 判断基站坐标有效性
	if (MatrixNorm2(3, 1, pStruct->m_BaseXYZ) < 10) return false;

	/// 7. 卫星部分状态清零
	ZeroSatStatus(pStruct);

	/// 8. 计算卫星位置
	getSatelliteOrbits(pStruct);

	// 数据断链引起状态参数重置
	if (RoundNum(pStruct->m_OBSGapTime[IROVE]) > pStruct->m_pGNSSOpt->m_DataGapTol)
	{
		// 静态情况下不重新初始化以下状态
		if (pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_KINEMA)
		{
			MatrixZero(MAXRTKNX, 1, pStruct->m_GLSInfo.StateX);
			MatrixZero(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP);
			SetValsI(pStruct->m_GLSInfo.m_AmbIndex, -1, NSATMAX* NFREQ);
			InitKF_Pos(pStruct);
		}
	}

	/// 9. 更新状态矩阵
	UpdataState(pStruct);

	/// 10. 预报状态
	KFPre(pStruct);

	return true;
}


static double setMEQ_Trp(CRTKPointBase* pStruct, const double rcvLLH[3], const double* azel)
{
	if (!pStruct)return 0.0;

	// 单历元固定或动对动还是不改对流层效果好
	if (pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_MOVEB || pStruct->m_pGNSSOpt->m_ARMode == ARMODE_INST) return 0.0;

	if (/*pStruct->m_EstTrpType == 0*/1)
	{
		if (TROP_UM == TROP_NONE) return 0;
		TropComputer(&pStruct->m_TropModel, pStruct->m_OBSTime[IROVE], rcvLLH, azel[1], false, TROP_UM/*pStruct->m_pGNSSOpt->pStruct->m_TropModel*/, TROP_GMF/*pStruct->m_pGNSSOpt->pStruct->m_TropMF*/, TROP_JPL/*pStruct->m_pGNSSOpt->pStruct->m_TropGrad*/);
		double Trop = pStruct->m_TropModel.m_ZHDMF * pStruct->m_TropModel.m_ZHD + pStruct->m_TropModel.m_ZWDMF * pStruct->m_TropModel.m_ZWD;
		return Trop;
	}
}


double SD_Variance(int prn, int frq, double el, double S, int OBSType)
{
	int sys = -1;
	satprn2no(prn, &sys);

	double fact = 1.0;
	switch (OBSType)
	{
	case OBS_PI: {fact = 1.0; break; }
	case OBS_LI: {fact = /*0.01*/1.0 / 300.0; break; }
	case OBS_DI: {fact = 0.1; break; }
	default:fact = 1.0;
	}

	//S = 0.0;
	if (S > 0.0)
	{
		//// for test
		//if (S > 50.0)S = 50.0;

		// 以伪距为基准
		double a = 637.4471, b = 0.1583, c = 0.1403;

		switch (sys)
		{
		case SYSGPS: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
		case SYSGLO: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
		case SYSBD2: {a = 84.8071;  b = 0.1288; c = 0.3858; break; }
		case SYSBD3: {a = 84.8071;  b = 0.1288; c = 0.3858; break; }
		case SYSGAL: {a = 383.3415; b = 0.1532; c = 0.3231; break; }
		case SYSQZS: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
		}

		//这个方差基本接近真实方差
		double sigma = fact * (a * exp(-b * S) + c);
		//if (OBSType == OBS_PI && frq == 2) sigma = sigma / 1.5; // L5波段伪距更好
		if (sys == SYSGPS && frq == 2) sigma /= 1.5;
		if (sys == SYSQZS && frq == 2) sigma /= 1.5;
		if (sys == SYSGAL && frq == 2) sigma /= 1.5;
		return (sigma * sigma);

		//// 信噪比随机模型2（20240514）
		//double a = 0.19, b = 427;
		//switch (sys)
		//{
		//case SYSGPS: {a = 0.19; b = 427; break; }
		//case SYSGLO: {a = 0.19; b = 427; break; }
		//case SYSBD2: {a = 0.02; b = 932; break; }
		//case SYSBD3: {a = 0.02; b = 932; break; }
		//case SYSGAL: {a = 0.03; b = 459; break; }
		//case SYSQZS: {a = 0.19; b = 427; break; }
		//}

		//double sigma = fact * (a + b * pow(10, -S / 10.0));
		//if (frq == 0)
		//	sigma = sigma * 1.5; // L5波段观测值更好
		//return (sigma * sigma);
	}
	else
	{
		// 以相位为基准
		double PLR2 = 10000 * (fact * fact);
		double Error_a2 = 9e-6;
		double Error_b2 = 9e-6;

		double varr = /*fact * fact * */PLR2 * (Error_a2 + Error_b2 / (sin(el) * sin(el)));

		return varr;
	}
}


// 外部导入参考星文件
static bool getBaseSats(CRTKPointBase* pStruct, FILE* fpBS)
{

	double secs = 0;
	int i = 0, f = 0, isys = 0, baseSat = 0, iposition = 0;
	char oneline[1024] = { '\0' }, delimiter[2] = ",";

	if (fpBS == NULL) return false;

	while (!feof(fpBS))
	{
		memset(oneline, '\0', 1024);
		iposition = ftell(fpBS);
		fgets(oneline, 1024, fpBS);
		char* p = oneline;
		for (i = 0, p = strtok(p, delimiter); p; p = strtok(NULL, delimiter), i++)
		{
			if (i == 0)
			{
				secs = atof(p);
				if ((GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt) - secs) > 0.02)
				{
					break;
				}
				else if((GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt) - secs) < -0.02)
				{
					fseek(fpBS, iposition, SEEK_SET);
					return false;
				}
			}
			else
			{
				baseSat = satno2prn1(p);
				pStruct->m_BaseSat[f][isys] = (baseSat >= 1 && baseSat <= NSATMAX) ? pStruct->m_SatStatus[IROVE][baseSat].OBSIndex : -1;
				if (++f >= 3)
				{
					f = 0;
					isys++;
				}
			}
		}
		if (isys > 0) break; // 说明已经读取，则退出
	}

	return (isys > 0);
}



static bool SelectBaseSat_P(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	SetValsI(pStruct->m_BaseSat[0], -1, NFREQ * NSYS);

	// for test, 外部输入参考星
	if (0)
	{
		static FILE* fpBSP = NULL;
		if (fpBSP == NULL)
		{
			fopen_s(&fpBSP, "E:\\01Prog\\04CDTH\\02Data\\01TestSet\\202411OneWallTestQuestion\\resi\\ROVE_20241128_06_TH110_BasePrns.txt", "r");
		}
		return getBaseSats(pStruct, fpBSP);
	}

	int prn = 0, isys = 0, nsat[NSYS] = { 0 };
	double el = 0.0, elevs[NSYS] = { 0.0 };
	double azel_rove[2] = { 0.0 }, azel_base[2] = { 0.0 }, rsVec_rove[3] = { 0.0 }, rsVec_base[3] = { 0.0 };
	double XYZ_rove[3], XYZ_base[3], LLH_rove[3], LLH_base[3];

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		SetValsI(nsat, 0, NSYS);
		SetValsD(elevs, 0, NSYS);

		M31EQU(pStruct->m_GLSInfo.StateX + pStruct->m_GLSInfo.IPos, XYZ_rove);
		M31EQU(pStruct->m_BaseXYZ, XYZ_base);
		XYZ2LLH(XYZ_rove, LLH_rove, 0);
		XYZ2LLH(XYZ_base, LLH_base, 0);

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

			// 流动站和基站必须同时存在相位观测值
			if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[f]) continue;
			if (pMEQ->v_P[f] == 0.0) continue;
			//if (prn == 84)continue;

			isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
			nsat[isys]++;
			// 计算卫地距和卫地单位向量
			if ((geodist(pStruct->m_SatStatus[IROVE][prn].SatPos, XYZ_rove, rsVec_rove)) <= 0.0) continue;
			if ((geodist(pStruct->m_SatStatus[IBASE][prn].SatPos, XYZ_base, rsVec_base)) <= 0.0) continue;

			// 检查是否满足高度角(验后残差略过),不要换顺序
			if (satazel(LLH_rove, rsVec_rove, azel_rove) < pStruct->m_pGNSSOpt->m_ElevMask/* && bPrioriRes*/) continue;
			if (satazel(LLH_base, rsVec_base, azel_base) < pStruct->m_pGNSSOpt->m_ElevMask/* && bPrioriRes*/) continue;

			pStruct->m_SatStatus[IROVE][prn].azel[0] = azel_rove[0];
			pStruct->m_SatStatus[IROVE][prn].azel[1] = azel_rove[1];
			pStruct->m_SatStatus[IBASE][prn].azel[0] = azel_base[0];
			pStruct->m_SatStatus[IBASE][prn].azel[1] = azel_base[1];
			el = pStruct->m_SatStatus[IROVE][prn].azel[1];

			if (pStruct->m_pOBSData[IROVE]->obs[i].S[f] < 40.0)
				el = el / 2.0;

			//if (isys == ISYSGPS && prn != 04)continue;
			//if (isys == ISYSBD3 && prn != 70)continue;
			//if (isys == ISYSBD2 && prn != 42)continue;
			//if (isys == ISYSGAL && prn != 106)continue;

			// 选择高度角最高的一颗卫星
			if (elevs[isys] < el)
			{
				elevs[isys] = el;
				pStruct->m_BaseSat[f][isys] = i;
			}
		}

		for (int k = 0; k < NSYS; k++)
		{
			if (!(Index2Sys(k) & pStruct->m_pGNSSOpt->m_SatSYS)) continue;

			if (nsat[k] < 2)
			{
				pStruct->m_BaseSat[f][k] = -1;
			}
		}
	}

	return true;
}


static void setOBSEvent(CRTKPointBase* pStruct, int prn, int OBSEvent)
{
	if (!pStruct)return;

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		for (int i = 0; i < 3; i++)
		{
			pStruct->m_SatStatus[IROVE][prn].OBSEvent[i][f] |= OBSEvent;
		}
	}
}


static int setMEQBase_P(CRTKPointBase* pStruct, const int iter)
{
	if (!pStruct)return 0;

	int prn = 0, f = 0, nValid = 0;
	double dist_rove = 0.0, azel_rove[2] = { 0.0 }, rsVec_rove[3] = { 0.0 }, dtrp_rove = 0.0, CorrP_rove[NFREQ] = { 0.0 };
	double dist_base = 0.0, azel_base[2] = { 0.0 }, rsVec_base[3] = { 0.0 }, dtrp_base = 0.0, CorrP_base[NFREQ] = { 0.0 };

	const double* StateX = (iter == 0 && pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED) ? pStruct->m_GLSInfo.StateXa : pStruct->m_GLSInfo.StateX;

	double XYZ_rove[3]; M31EQU(StateX + pStruct->m_GLSInfo.IPos, XYZ_rove);
	double XYZ_base[3]; M31EQU(pStruct->m_BaseXYZ, XYZ_base);
	double LLH_rove[3], LLH_base[3];
	XYZ2LLH(XYZ_rove, LLH_rove, 0); XYZ2LLH(XYZ_base, LLH_base, 0);

	pStruct->m_GLSInfo.nMeasP = 0;
	pStruct->m_GLSInfo.nMEQ = pStruct->m_pOBSData[IROVE]->nsat;

	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		pMEQ->bMEQ = false;

		// 流动站与基站没有匹配数据
		if (pMEQ->OBSIndex < 0) continue;

		const OBS_DATA_t* pOBS_rove = &(pStruct->m_pOBSData[IROVE]->obs[i]);
		const OBS_DATA_t* pOBS_base = &(pStruct->m_pOBSData[IBASE]->obs[pMEQ->OBSIndex]);
		prn = pOBS_rove->prn;

		pStruct->m_SatStatus[IROVE][prn].azel[0] = 0.0;
		pStruct->m_SatStatus[IROVE][prn].azel[1] = 0.0;

		for (f = 0; f < NFREQ; f++)
		{
			pMEQ->v_P[f] = 0.0;
			pMEQ->R_P[f] = 0.0;

			pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_PI][f] = false;
		}

		// 卫星不健康
		if (pStruct->m_SatStatus[IROVE][prn].Satsvh != 0 || pStruct->m_SatStatus[IBASE][prn].Satsvh != 0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_SSVH);
			continue;
		}

		// 排除卫星
		if (pStruct->m_SatStatus[IROVE][prn].bExclude || pStruct->m_SatStatus[IBASE][prn].bExclude)
		{
			setOBSEvent(pStruct, prn, OBSEVT_DELE);
			continue;
		}

		// 计算卫地距和卫地单位向量
		if ((dist_rove = geodist(pStruct->m_SatStatus[IROVE][prn].SatPos, XYZ_rove, rsVec_rove)) <= 0.0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_XMEQ);
			continue;
		}
		if ((dist_base = geodist(pStruct->m_SatStatus[IBASE][prn].SatPos, XYZ_base, rsVec_base)) <= 0.0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_XMEQ);
			continue;
		}

		// 检查是否满足高度角(验后残差略过),不要换顺序
		if (satazel(LLH_rove, rsVec_rove, azel_rove) < pStruct->m_pGNSSOpt->m_ElevMask)
		{
			setOBSEvent(pStruct, prn, OBSEVT_ELEV);
			continue;
		}
		if (satazel(LLH_base, rsVec_base, azel_base) < pStruct->m_pGNSSOpt->m_ElevMask)
		{
			setOBSEvent(pStruct, prn, OBSEVT_ELEV);
			continue;
		}

		// 得到修正以后的观测值
		//CorrMeasurement(prn, azel_rove, XYZ_rove, 0, pOBS_rove, CorrL_rove, CorrP_rove, vdcb_rove)
		for (int f = 0; f < NFREQ; f++)
		{
			CorrP_rove[f] = pOBS_rove->P[f];
			CorrP_base[f] = pOBS_base->P[f];
		}

		// 赋值流动站的方向向量
		M31EQU_1(rsVec_rove, pMEQ->H_XYZ);

		// 计算流动站对流层延迟
		dtrp_rove = setMEQ_Trp(pStruct, LLH_rove, azel_rove);

		// 基站每次计算其实是一样的,里面的GMF计算很耗时,这样可以加快速度
		if (pMEQ->Trp_base == 0.0)
		{
			pMEQ->Trp_base = setMEQ_Trp(pStruct, LLH_base, azel_base);
		}

		dtrp_base = pMEQ->Trp_base;

		// 计算预报距离
		dist_rove += (-CLIGHT * pStruct->m_SatStatus[IROVE][prn].SatClk + dtrp_rove);
		dist_base += (-CLIGHT * pStruct->m_SatStatus[IBASE][prn].SatClk + dtrp_base);

		// 得到站间单差观测值
		for (int f = 0; f < NFREQ; f++)
		{
			//if (pStruct->m_bFreqUsed[f] == false || f == 1) continue; // TH数据L2波段伪距不好，暂时排除

			if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[f]) continue;

			if (CorrP_rove[f] == 0.0 || CorrP_base[f] == 0.0)
			{
				pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_PI][f] |= OBSEVT_XMEQ;
				continue;
			}

			// 根据信噪比去掉相位观测值不好,可以考虑去掉伪距和多普勒
			if (pOBS_rove->S[f] > 0.0 && pOBS_rove->S[f] < pStruct->m_pGNSSOpt->m_SNRThres[f])
			{
				pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_PI][f] |= OBSEVT_SNR;
				continue;
			}

			if (CorrP_rove[f] != 0.0 && CorrP_base[f] != 0.0)
			{
				//if (pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_PI][f] & OBSEVT_XDOP)continue; // 一致性
				//if (pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_PI][f] & OBSEVT_Inno_v)continue; // 验前
				//if (pStruct->m_SPPoint.m_PosSolType == GNSSMODE_SPP && (pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_PI][f] & OBSEVT_Resi_v))continue; // 验后

				pMEQ->v_P[f] = (CorrP_rove[f] - CorrP_base[f]) - (dist_rove - dist_base);
				pMEQ->R_P[f] = SD_Variance(prn, f, azel_rove[1], pOBS_rove->S[f], OBS_PI);

				// 有时候残差值恰好为0
				if (pMEQ->v_P[f] == 0.0)
				{
					pMEQ->v_P[f] = IPS_EPSILON;
				}

				pMEQ->bMEQ = true;
			}
		}

		if (pMEQ->bMEQ)
		{
			pStruct->m_SatStatus[IROVE][prn].azel[0] = azel_rove[0];
			pStruct->m_SatStatus[IROVE][prn].azel[1] = azel_rove[1];
			pStruct->m_SatStatus[IBASE][prn].azel[0] = azel_base[0];
			pStruct->m_SatStatus[IBASE][prn].azel[1] = azel_base[1];
			nValid++;
		}
	}

	// 此时知道了哪些卫星可用,可以选择参考星了,必须放在这里
	// 第一次迭代就确定还是每次迭代都重新选择
	if (iter >= 0)
	{
		if (SelectBaseSat_P(pStruct) == false) return 0;
	}

	return nValid;
}


static bool AnalyzePrioriResi(CRTKPointBase* pStruct, int type)
{
	if (!pStruct)return false;

	int prn, f, i, isys = 0;

	if (type == OBS_PI)
	{
		double PThres = 6.0, vPs[MAXOBSLIM];
		int nBadP = 0, nvPs = 0;

		// 确定验前阈值
		for (f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if (i == pStruct->m_BaseSat[f][isys]) continue;
				if (pStruct->m_BaseSat[f][isys] < 0)continue;

				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;

				if (pMEQ->v_P[f] != 0)
				{
					//vPs[nvPs++] = fabs(pMEQ->v_P[f]);
					vPs[nvPs++] = pMEQ->v_P[f];
					if (fabs(pMEQ->v_P[f]) > PThres) nBadP++;
				}
			}
		}

		double val = pStruct->m_GLSInfo.StateP[0] + pStruct->m_GLSInfo.StateP[MAXRTKNX + 1] + pStruct->m_GLSInfo.StateP[2 * MAXRTKNX + 2];
		if (val > 900) // 先验位置不可靠，设置宽松阈值
		{
			PThres = 20.0;
		}
		else if (nvPs > 0 && nBadP > (nvPs / 2))
		{
			PThres = GetDynamicThres(vPs, nvPs, 0.7, 6.0, 20.0, true);
		}

		// 剔除伪距粗差
		for (f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if (i == pStruct->m_BaseSat[f][isys]) continue;
				if (pStruct->m_BaseSat[f][isys] < 0)continue;

				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;

				if (nvPs > 2)
				{
					//if (pMEQ->v_P[f] != 0)
					//{
					//	char sprn[5];
					//	satprn2nos(prn, sprn);
					//	printf("%8.1f,%3s(%3d),%d,%8.3f,%8.3f,%5.1f,%8.3f,%8.3f\n",
					//		GetGPSTIMESow(pStruct->m_OBSTime[IROVE]),
					//		sprn, prn, f, pMEQ->v_P[f], pMEQ->R_P[f],
					//		pStruct->m_SatStatus[IROVE][prn].S[f],
					//		pStruct->m_SatStatus[IROVE][prn].azel[0] * R2D,
					//		pStruct->m_SatStatus[IROVE][prn].azel[1] * R2D);
					//}
					if ((pMEQ->v_P[f] != 0 && fabs(pMEQ->v_P[f]) > PThres) && nBadP < nvPs)
					{
						pMEQ->v_P[f] = 0.0;
					}
				}
			}
		}
	}

	if (type == OBS_LI)
	{
		double LThres = 6.0, vLs[MAXOBSLIM];
		int nBadL = 0, nvLs = 0;

		for (f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if (i == pStruct->m_BaseSat[f][isys]) continue;
				if (pStruct->m_BaseSat[f][isys] < 0)continue;

				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
				if (pMEQ->v_L[f] != 0)
				{
					//vLs[nvLs++] = fabs(pMEQ->v_L[f]);
					vLs[nvLs++] = pMEQ->v_L[f];
					if (fabs(pMEQ->v_L[f]) > LThres)nBadL++;
				}
			}
		}

		if (nvLs > 0 && nBadL > (nvLs / 2))
		{
			LThres = GetDynamicThres(vLs, nvLs, 0.7, 6.0, 20.0, true);
		}

		for (f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if (i == pStruct->m_BaseSat[f][isys]) continue;
				if (pStruct->m_BaseSat[f][isys] < 0)continue;

				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;

				if (nvLs > 2)
				{
					if ((pMEQ->v_L[f] != 0 && fabs(pMEQ->v_L[f]) > LThres))
					{
						pMEQ->v_L[f] = 0.0;
					}
				}
			}
		}
	}

	return true;
}


static bool setMEQ_P(CRTKPointBase* pStruct, const int iter)
{
	if (!pStruct)return false;

	int nValid = setMEQBase_P(pStruct, iter);  // 站间单差，选择参考星
	if (iter > 0 && nValid < pStruct->m_nMinSatThres)
	{
		return false;
	}

	//const double* StateX = (iter == 0 && pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED) ? pStruct->m_GLSInfo.StateXa : pStruct->m_GLSInfo.StateX;
	int prn = 0, isys = 0, f = 0, prn_base = 0, BaseSat = -1;
	double vP = 0.0, vP_base = 0.0;

	// 1.得到双差残差
	for (f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		isys = ISYSNON;

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

			// 先选择基准星
			if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
			{
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if ((BaseSat = pStruct->m_BaseSat[f][isys]) >= 0)
				{
					prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;
					if (pStruct->m_GLSInfo.MEQ[BaseSat].v_P[f] == 0.0) { BaseSat = -1; continue; }
					vP_base = pStruct->m_GLSInfo.MEQ[BaseSat].v_P[f];
					pStruct->m_GLSInfo.MEQ[BaseSat].v_P[f] = 0.0; // 基准星残差一定要设为0,不然验前残差分析会纳入进去
				}
			}

			if (BaseSat < 0)
			{
				pMEQ->v_P[f] = 0.0;
			}

			if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

			vP = pMEQ->v_P[f];
			pMEQ->v_P[f] = 0.0;

			if (vP != 0.0 && vP_base != 0.0)
			{
				pMEQ->v_P[f] = vP - vP_base;
			}
		}
	}

	// 2.验前残差检验,微秒钟跳的时候,Pv很小,而Lv会很大
	if (iter > 0) AnalyzePrioriResi(pStruct, OBS_PI);

	// 3.统计观测值个数与状态赋值
	for (f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		isys = ISYSNON;

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

			// 先选择基准星
			if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
			{
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if ((BaseSat = pStruct->m_BaseSat[f][isys]) < 0) continue;
				prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;

				pStruct->m_SatStatus[IROVE][prn_base].OBSValid[OBS_PI][f] = pStruct->m_SatStatus[IBASE][prn_base].OBSValid[OBS_PI][f] = true;
			}

			if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

			// 伪距验前残差不通过
			if (pMEQ->v_P[f] != 0.0) { pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_PI][f] = pStruct->m_SatStatus[IBASE][prn].OBSValid[OBS_PI][f] = true; pStruct->m_GLSInfo.nMeasP++; }
		}
	}

	if (iter > 0)
	{
		if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
		{
			pStruct->m_pGNSSSol->m_LOG.RTKnMeas[0] = pStruct->m_GLSInfo.nMeasP;
		}
	}

	// 4.设置最小观测数
	if (iter > 0 && (pStruct->m_GLSInfo.nMeasP < 4))
	{
		return false;
	}

	return true;
}


static bool SelectBaseSat_L(CRTKPointBase* pStruct)
{
	return true;
	if (!pStruct)return false;

	SetValsI(pStruct->m_BaseSat[0], -1, NFREQ * NSYS);

	// for test, 外部输入参考星
	if (0)
	{
		static FILE* fpBSL = NULL;
		if (fpBSL == NULL)
		{
			fopen_s(&fpBSL, "E:\\01Prog\\04CDTH\\02Data\\01TestSet\\202411OneWallTestQuestion\\resi\\ROVE_20241128_06_TH110_BasePrns.txt", "r");
		}
		return getBaseSats(pStruct, fpBSL);
	}

	int prn = 0, isys = 0, nsat[NSYS] = { 0 }, BasePrn_f[NSYS];
	double el = 0.0, elevs[NSYS] = { 0.0 }, elevs_cs[NSYS] = { 0.0 };
	double azel_rove[2] = { 0.0 }, azel_base[2] = { 0.0 }, rsVec_rove[3] = { 0.0 }, rsVec_base[3] = { 0.0 };
	double XYZ_rove[3], XYZ_base[3], LLH_rove[3], LLH_base[3];

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		SetValsI(nsat, 0, NSYS);
		SetValsI(BasePrn_f, 0, NSYS);
		SetValsD(elevs, 0, NSYS);
		SetValsD(elevs_cs, 0, NSYS);

		M31EQU(pStruct->m_GLSInfo.StateX + pStruct->m_GLSInfo.IPos, XYZ_rove);
		M31EQU(pStruct->m_BaseXYZ, XYZ_base);
		XYZ2LLH(XYZ_rove, LLH_rove, 0);
		XYZ2LLH(XYZ_base, LLH_base, 0);

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			if (pStruct->m_GLSInfo.MEQ[i].OBSIndex < 0) continue;
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

			// 流动站和基站必须同时存在相位观测值
			if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[f]) continue;
			if (pMEQ->v_L[f] == 0.0) continue;

			isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
			nsat[isys]++;
			// 计算卫地距和卫地单位向量
			if ((geodist(pStruct->m_SatStatus[IROVE][prn].SatPos, XYZ_rove, rsVec_rove)) <= 0.0) continue;
			if ((geodist(pStruct->m_SatStatus[IBASE][prn].SatPos, XYZ_base, rsVec_base)) <= 0.0) continue;

			// 检查是否满足高度角(验后残差略过),不要换顺序
			if (satazel(LLH_rove, rsVec_rove, azel_rove) < pStruct->m_pGNSSOpt->m_ElevMask/* && bPrioriRes*/) continue;
			if (satazel(LLH_base, rsVec_base, azel_base) < pStruct->m_pGNSSOpt->m_ElevMask/* && bPrioriRes*/) continue;

			pStruct->m_SatStatus[IROVE][prn].azel[0] = azel_rove[0];
			pStruct->m_SatStatus[IROVE][prn].azel[1] = azel_rove[1];
			pStruct->m_SatStatus[IBASE][prn].azel[0] = azel_base[0];
			pStruct->m_SatStatus[IBASE][prn].azel[1] = azel_base[1];
			el = pStruct->m_SatStatus[IROVE][prn].azel[1];

			if (pStruct->m_pOBSData[IROVE]->obs[i].S[f] < 40.0)
				el = el / 2.0;

			//if (isys == ISYSGPS && prn != 04)continue;
			//if (isys == ISYSBD3 && prn != 70)continue;
			//if (isys == ISYSBD2 && prn != 42)continue;
			//if (isys == ISYSGAL && prn != 106)continue;

			// 选择高度角最高的一颗卫星
			if (elevs[isys] < el)
			{
				elevs[isys] = el;
				BasePrn_f[isys] = i;
			}

			if (pStruct->m_GNSSQC.m_bReinitAllAmb[f])continue;
			if (pStruct->m_SatStatus[IROVE][prn].CS_Type[f]) continue;
			if (pStruct->m_SatStatus[IBASE][prn].CS_Type[f]) continue;
			if (pStruct->m_SatStatus[IROVE][prn].bResetAmb[f])continue;
			if (pStruct->m_SatStatus[IBASE][prn].bResetAmb[f])continue;

			// 选择无周跳下,高度角最高的一颗卫星
			if (elevs_cs[isys] < el)
			{
				elevs_cs[isys] = el;
				pStruct->m_BaseSat[f][isys] = i;
			}
		}

		for (int k = 0; k < NSYS; k++)
		{
			if (!(Index2Sys(k) & pStruct->m_pGNSSOpt->m_SatSYS)) continue;

			if (pStruct->m_BaseSat[f][k] < 0 && nsat[k] >= 2)
			{
				pStruct->m_BaseSat[f][k] = BasePrn_f[k];
			}
		}
	}

	return true;
}


static int setMEQBase_L(CRTKPointBase* pStruct, const int iter)
{
	if (!pStruct)return 0;

	int prn = 0, f = 0, nValid = 0;
	double dist_rove = 0.0, azel_rove[2] = { 0.0 }, rsVec_rove[3] = { 0.0 }, dtrp_rove = 0.0, CorrL_rove[NFREQ] = { 0.0 };
	double dist_base = 0.0, azel_base[2] = { 0.0 }, rsVec_base[3] = { 0.0 }, dtrp_base = 0.0, CorrL_base[NFREQ] = { 0.0 };
	double* lam = NULL;

	const double* StateX = (iter == 0 && pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED) ? pStruct->m_GLSInfo.StateXa : pStruct->m_GLSInfo.StateX;

	double XYZ_rove[3]; M31EQU(StateX + pStruct->m_GLSInfo.IPos, XYZ_rove);
	double XYZ_base[3]; M31EQU(pStruct->m_BaseXYZ, XYZ_base);
	double LLH_rove[3], LLH_base[3];
	XYZ2LLH(XYZ_rove, LLH_rove, 0); XYZ2LLH(XYZ_base, LLH_base, 0);

	pStruct->m_GLSInfo.nMeasL = 0;
	pStruct->m_GLSInfo.nMEQ = pStruct->m_pOBSData[IROVE]->nsat;

	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		pMEQ->bMEQ = false;

		// 流动站与基站没有匹配数据
		if (pMEQ->OBSIndex < 0) continue;

		const OBS_DATA_t* pOBS_rove = &(pStruct->m_pOBSData[IROVE]->obs[i]);
		const OBS_DATA_t* pOBS_base = &(pStruct->m_pOBSData[IBASE]->obs[pMEQ->OBSIndex]);
		prn = pOBS_rove->prn;
		lam = pStruct->m_SatStatus[IROVE][prn].lam;

		pStruct->m_SatStatus[IROVE][prn].azel[0] = 0.0;
		pStruct->m_SatStatus[IROVE][prn].azel[1] = 0.0;

		for (f = 0; f < NFREQ; f++)
		{
			pMEQ->v_L[f] = 0.0;
			pMEQ->R_L[f] = 0.0;
			pMEQ->tdv_L[f] = 0.0;

			pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f] = false;
		}

		// 卫星不健康
		if (pStruct->m_SatStatus[IROVE][prn].Satsvh != 0 || pStruct->m_SatStatus[IBASE][prn].Satsvh != 0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_SSVH);
			continue;
		}

		// 排除卫星
		if (pStruct->m_SatStatus[IROVE][prn].bExclude || pStruct->m_SatStatus[IBASE][prn].bExclude)
		{
			setOBSEvent(pStruct, prn, OBSEVT_DELE);
			continue;
		}

		// 计算卫地距和卫地单位向量
		if ((dist_rove = geodist(pStruct->m_SatStatus[IROVE][prn].SatPos, XYZ_rove, rsVec_rove)) <= 0.0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_XMEQ);
			continue;
		}
		if ((dist_base = geodist(pStruct->m_SatStatus[IBASE][prn].SatPos, XYZ_base, rsVec_base)) <= 0.0)
		{
			setOBSEvent(pStruct, prn, OBSEVT_XMEQ);
			continue;
		}

		// 检查是否满足高度角(验后残差略过),不要换顺序
		if (satazel(LLH_rove, rsVec_rove, azel_rove) < pStruct->m_pGNSSOpt->m_ElevMask)
		{
			setOBSEvent(pStruct, prn, OBSEVT_ELEV);
			continue;
		}
		if (satazel(LLH_base, rsVec_base, azel_base) < pStruct->m_pGNSSOpt->m_ElevMask)
		{
			setOBSEvent(pStruct, prn, OBSEVT_ELEV);
			continue;
		}

		// 得到修正以后的观测值
		for (int f = 0; f < NFREQ; f++)
		{
			CorrL_rove[f] = pOBS_rove->L[f];
			CorrL_base[f] = pOBS_base->L[f];
		}

		// 赋值流动站的方向向量
		M31EQU_1(rsVec_rove, pMEQ->H_XYZ);

		// 计算流动站对流层延迟
		dtrp_rove = setMEQ_Trp(pStruct, LLH_rove, azel_rove);

		// 基站每次计算其实是一样的,里面的GMF计算很耗时,这样可以加快速度
		if (pMEQ->Trp_base == 0.0)
		{
			pMEQ->Trp_base = setMEQ_Trp(pStruct, LLH_base, azel_base);
		}

		dtrp_base = pMEQ->Trp_base;

		// 计算预报距离
		dist_rove += (-CLIGHT * pStruct->m_SatStatus[IROVE][prn].SatClk + dtrp_rove);
		dist_base += (-CLIGHT * pStruct->m_SatStatus[IBASE][prn].SatClk + dtrp_base);

		// 得到站间单差观测值(相位未减去模糊度)
		for (int f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false || lam[f] == 0.0) continue;

			if (!pStruct->m_SatStatus[IROVE][prn].OBSUsed[f]) continue;

			if (CorrL_rove[f] == 0.0 || CorrL_base[f] == 0.0)
			{
				pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_LI][f] |= OBSEVT_XMEQ;
				continue;
			}

			// 根据信噪比去掉相位观测值不好,可以考虑去掉伪距和多普勒
			if (pOBS_rove->S[f] > 0.0 && pOBS_rove->S[f] < pStruct->m_pGNSSOpt->m_SNRThres[f])
			{
				pStruct->m_SatStatus[IROVE][prn].OBSEvent[OBS_LI][f] |= OBSEVT_SNR;
				continue;
			}

			if (CorrL_rove[f] != 0.0 && CorrL_base[f] != 0.0)
			{
				pMEQ->v_L[f] = lam[f] * (CorrL_rove[f] - CorrL_base[f]) - (dist_rove - dist_base);
				pMEQ->R_L[f] = SD_Variance(prn, f, azel_rove[1], pOBS_rove->S[f], OBS_LI); // 单差观测值方差

				// 有时候残差值恰好为0
				if (pMEQ->v_L[f] == 0.0)
				{
					pMEQ->v_L[f] = IPS_EPSILON;
				}

				pMEQ->bMEQ = true;
			}
		}

		if (pMEQ->bMEQ)
		{
			pStruct->m_SatStatus[IROVE][prn].azel[0] = azel_rove[0];
			pStruct->m_SatStatus[IROVE][prn].azel[1] = azel_rove[1];
			pStruct->m_SatStatus[IBASE][prn].azel[0] = azel_base[0];
			pStruct->m_SatStatus[IBASE][prn].azel[1] = azel_base[1];
			nValid++;
		}
	}

	// 此时知道了哪些卫星可用,可以选择参考星了,必须放在这里
	// 第一次迭代就确定还是每次迭代都重新选择
	if (iter >= 0)
	{
		if (SelectBaseSat_L(pStruct) == false) return 0;
	}

	return nValid;
}


static bool setMEQ_L(CRTKPointBase* pStruct, const int iter)
{
	if (!pStruct)return false;

	int nValid = setMEQBase_L(pStruct, iter);  // 站间单差，选择参考星
	if (iter > 0 && nValid < pStruct->m_nMinSatThres)
	{
		return false;
	}

	const double* StateX = (iter == 0 && pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED) ? pStruct->m_GLSInfo.StateXa : pStruct->m_GLSInfo.StateX;
	int prn = 0, isys = 0, f = 0, ambLoc = 0, prn_base = 0, BaseSat = -1;
	double vL = 0.0, vL_base = 0.0, * lam = NULL, * lam_base = NULL;

	// 1.得到双差残差
	for (f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		isys = ISYSNON;

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
			lam = pStruct->m_SatStatus[IROVE][prn].lam;

			// 先选择基准星
			if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
			{
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if ((BaseSat = pStruct->m_BaseSat[f][isys]) >= 0)
				{
					prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;
					lam_base = pStruct->m_SatStatus[IROVE][prn_base].lam; // 基准星波长不可能为0,在setMeasureBase中已做判断
					if ((ambLoc = GetAmbLoc(pStruct, f, prn_base)) < pStruct->m_GLSInfo.IAmb) { BaseSat = -1; continue; }
					else if (pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] <= 0.0) { BaseSat = -1; continue; }
					else if (pStruct->m_GLSInfo.MEQ[BaseSat].v_L[f] == 0.0) { BaseSat = -1; continue; }
					vL_base = pStruct->m_GLSInfo.MEQ[BaseSat].v_L[f] /*- StateX[ambLoc] * lam_base[f]*/;  // pStruct->m_GLSInfo.StateX存储的是单差模糊度,这里减去模糊度,得到残差
					pStruct->m_GLSInfo.MEQ[BaseSat].v_L[f] = 0.0; // 基准星残差一定要设为0,不然验前残差分析会纳入进去
				}
			}

			if (BaseSat < 0)
			{
				pMEQ->v_L[f] = 0.0;
			}

			if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

			vL = pMEQ->v_L[f];
			pMEQ->v_L[f] = 0.0;

			if ((ambLoc = GetAmbLoc(pStruct, f, prn)) < pStruct->m_GLSInfo.IAmb)   vL = 0.0;
			else if (pStruct->m_GLSInfo.StateP[ambLoc * MAXRTKNX + ambLoc] <= 0.0) vL = 0.0;

			if (vL != 0.0 && vL_base != 0.0)
			{
				//vL = vL - StateX[ambLoc] * lam[f];// 减去模糊度得到非参考星真正的单差残差
				pMEQ->v_L[f] = vL - vL_base;    // 得到双差残差
			}
		}
	}

	// 2.验前残差检验,微秒钟跳的时候,Pv很小,而Lv会很大
	if (iter > 0) AnalyzePrioriResi(pStruct, OBS_LI);

	// 3.统计观测值个数与状态赋值
	for (f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		isys = ISYSNON;

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
			lam = pStruct->m_SatStatus[IROVE][prn].lam;

			// 先选择基准星
			if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
			{
				isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
				if ((BaseSat = pStruct->m_BaseSat[f][isys]) < 0) continue;
				prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;

				pStruct->m_SatStatus[IROVE][prn_base].OBSValid[OBS_LI][f] = pStruct->m_SatStatus[IBASE][prn_base].OBSValid[OBS_LI][f] = true;
			}

			if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

			// 相位验前残差不通过
			if (pMEQ->v_L[f] != 0.0) { pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f] = pStruct->m_SatStatus[IBASE][prn].OBSValid[OBS_LI][f] = true; pStruct->m_GLSInfo.nMeasL++; }

			// for test
			//if (pMEQ->v_L[f] != 0.0 && iter == 0 && (pMEQ->GLSPosL[f] - 1) >= 0)
			//{
			//	char sprn[5];
			//	satprn2nos(prn, sprn);
			//	printf("%3s(%3d),%d,%8.3f\n", sprn, prn, f, pMEQ->v_L[f]);
			//}
		}
	}

	if (iter > 0)
	{
		if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
		{
			pStruct->m_pGNSSSol->m_LOG.RTKnMeas[1] = pStruct->m_GLSInfo.nMeasL;
		}
	}

	// 4.设置最小观测数
	if ((pStruct->m_GLSInfo.nMeasL < 4) && iter > 0)
	{
		return false;
	}

	return true;
}


// 计算相位三差
static bool settd_VL(CRTKPointBase* pStruct)
{
	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		int prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
		for (int f = 0; f < NFREQ; f++)
		{
			//计算相位三差
			int sys = pStruct->m_SatStatus[IROVE][prn].sys_id;
			int index = pStruct->m_BaseSat[f][sys];
			if (index == -1)continue;
			int basePrn = pStruct->m_pOBSData[IROVE]->obs[index].prn;//当前参考星的prn号
			if (pStruct->m_SatStatus[IROVE][prn].DD_L[f] != 0 && pMEQ->v_L[f] != 0 && basePrn == pStruct->m_PreBasePrn[f][sys])
			{
				pMEQ->tdv_L[f] = pMEQ->v_L[f] - pStruct->m_SatStatus[IROVE][prn].DD_L[f];
				if (pMEQ->tdv_L[f] == 0.0|| fabs(pMEQ->tdv_L[f])< pStruct->m_SatStatus[IROVE][prn].lam[f] * 0.0001) pMEQ->tdv_L[f] = pStruct->m_SatStatus[IROVE][prn].lam[f] * 0.0001;
			}
		}
	}
	// 清空
	for (int i = 0; i < NSATMAX; i++)
	{
		for (int f = 0; f < NFREQ; f++)
		{
			pStruct->m_SatStatus[IROVE][i + 1].DD_L[f] = 0.0;
		}
	}
	//清空
	SetValsI(pStruct->m_PreBasePrn[0], -1, NFREQ * NSYS);

	// 赋值 上一历元的双差残差
	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		int prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
		for (int f = 0; f < NFREQ; f++)
		{
			if (pMEQ->v_L[f] != 0.0)
			{
				pStruct->m_SatStatus[IROVE][prn].DD_L[f] = pMEQ->v_L[f];
			}
		}
	}
	//赋值 上一历元双差参考星的prn
	for (int i = 0; i < NSYS; i++)
	{
		for (int f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_BaseSat[f][i] >= 0)
			{
				pStruct->m_PreBasePrn[f][i] = pStruct->m_pOBSData[IROVE]->obs[pStruct->m_BaseSat[f][i]].prn;
			}
		}
	}
	return true;
}


static bool FormMEQ(CRTKPointBase* pStruct, const int type)
{
	if (!pStruct)return false;

	SetValsI(pStruct->m_GLSInfo.StateIndex, 0, MAXRTKNX);
	int HI[MAXRTKNX] = { 0 }, nHI = 0;
	for (int i = 0; i < pStruct->m_GLSInfo.LStateConst; i++)
	{
		if (pStruct->m_GLSInfo.StateP[i * MAXRTKNX + i] > 0.0)
		{
			HI[i] = nHI;
			pStruct->m_GLSInfo.StateIndex[nHI++] = i;
		}
	}

	pStruct->m_GLSInfo.nStateIndex = nHI;
	if (type == OBS_PI)		pStruct->m_GLSInfo.nMeas = pStruct->m_GLSInfo.nMeasP;
	else if (type == OBS_LI)pStruct->m_GLSInfo.nMeas = pStruct->m_GLSInfo.nMeasL;
	else return false;

	// 初始化 GLS 各变量
	InitGLS(&pStruct->m_GLS, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.nMeas, false);
	SetGLSXP(&pStruct->m_GLS, pStruct->m_GLSInfo.StateIndex, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX);

	// 在抗差迭代中需要清空，否则FormMEQ及后续解算中可能计算上次抗差的GLSPosL、GLSPosP
	for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
	{
		GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
		for (int f = 0; f < NFREQ; f++)
		{
			if (type == OBS_PI)pMEQ->GLSPosP[f] = 0;
			if (type == OBS_LI)pMEQ->GLSPosL[f] = 0;
		}
	}

	int prn = 0, prn_base = 0, isys = ISYSNON, BaseSat = -1;
	int ambLoc_rove = 0, ambLoc_base = 0;
	int Hi = 0, xi = 0, yi = 0, Hi_iOBS[MAXOBSLIM] = { 0 }, nHi_iOBS = 0;
	int ri[NSYS] = { 0 }, rl[NSYS] = { 0 };  // 非差协方差转单差协方差时,R赋值所用
	GNSS_MEQ* pMEQ = NULL, * pMEQ_base = NULL;

	// 在形成观测方程的时候,按照相位,伪距,多普勒的顺序填充观测矩阵
	// 相位中按照L1-L3的顺序填充观测矩阵

	// 先处理相位观测值H,R
	if (type == OBS_LI)
	{
		for (int f = 0; f < NFREQ; f++) // 不同频率
		{
			SetValsI(ri, 0, NSYS);
			SetValsI(rl, 0, NSYS);

			if (pStruct->m_bFreqUsed[f] == false) continue;

			isys = ISYSNON;

			for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				pMEQ = pStruct->m_GLSInfo.MEQ + i;
				if (pMEQ->bMEQ == false) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

				if (pStruct->m_SatStatus[IROVE][prn].sys_id != isys)  // 不同系统区分参考星
				{
					isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
					if ((BaseSat = pStruct->m_BaseSat[f][isys]) < 0) continue;
					pMEQ_base = pStruct->m_GLSInfo.MEQ + BaseSat;
					if (pMEQ_base->bMEQ == false) continue;
					prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;
					if ((ambLoc_base = GetAmbLoc(pStruct, f, prn_base)) < pStruct->m_GLSInfo.IAmb)return false;
					ri[isys] = Hi;
					rl[isys] = 0;
				}

				if (BaseSat < 0 || i == BaseSat) continue;

				if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f])
				{
					// 双差方向余弦填充H矩阵
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 0]] = pMEQ->H_XYZ[0] - pMEQ_base->H_XYZ[0];
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 1]] = pMEQ->H_XYZ[1] - pMEQ_base->H_XYZ[1];
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 2]] = pMEQ->H_XYZ[2] - pMEQ_base->H_XYZ[2];

					if ((ambLoc_rove = GetAmbLoc(pStruct, f, prn)) < pStruct->m_GLSInfo.IAmb)return false;

					// 站间单差模糊度系数
					if (HI[ambLoc_base] > 0 && HI[ambLoc_rove] > 0)
					{
						pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[ambLoc_rove]] = pStruct->m_SatStatus[IROVE][prn].lam[f];
						pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[ambLoc_base]] = -pStruct->m_SatStatus[IROVE][prn].lam[f];
					}

					// 双差残差
					pStruct->m_GLS.m_Inno[Hi] = pMEQ->v_L[f]; // 这里的v已经是双差残差

					// 流动站的观测方差,后面要转成星间单差的协方差阵
					pStruct->m_GLS.m_R[Hi * pStruct->m_GLS.m_nMeas + Hi] = pMEQ->R_L[f];

					Hi_iOBS[nHi_iOBS++] = i;
					pMEQ->GLSPosL[f] = Hi + 1; // prn卫星在GLS中的位置,+1是因为在计算验后残差时需要判断
					Hi++;
					rl[isys]++;
				}
			}

			// 转换为双差观测方差
			for (isys = 0; isys < NSYS; isys++)
			{
				if (rl[isys] != 0)
				{
					BaseSat = pStruct->m_BaseSat[f][isys];
					pMEQ_base = pStruct->m_GLSInfo.MEQ + BaseSat;
					pMEQ = pStruct->m_GLSInfo.MEQ;

					for (int i = 0; i < rl[isys]; i++)
						for (int j = 0; j <= i; j++)
						{
							xi = ri[isys] + i;
							yi = ri[isys] + j;
							pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] += pMEQ_base->R_L[f];
							if (pMEQ[Hi_iOBS[xi]].RFactL[f] > 0.0) 
								pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] *= xsqrt(pMEQ[Hi_iOBS[xi]].RFactL[f]);
							if (pMEQ[Hi_iOBS[yi]].RFactL[f] > 0.0) 
								pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] *= xsqrt(pMEQ[Hi_iOBS[yi]].RFactL[f]);
							pStruct->m_GLS.m_R[yi * pStruct->m_GLS.m_nMeas + xi] = pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi];
						}
				}
			}
		}
	}

	// 再处理伪距观测值
	if (type == OBS_PI)
	{
		for (int f = 0; f < NFREQ; f++)
		{
			SetValsI(ri, 0, NSYS);
			SetValsI(rl, 0, NSYS);

			if (pStruct->m_bFreqUsed[f] == false) continue;

			isys = ISYSNON;

			for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				pMEQ = pStruct->m_GLSInfo.MEQ + i;
				if (pMEQ->bMEQ == false) continue;
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

				if (pStruct->m_SatStatus[IROVE][prn].sys_id != isys)  // 不同系统区分参考星
				{
					isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
					if ((BaseSat = pStruct->m_BaseSat[f][isys]) < 0) continue;
					pMEQ_base = pStruct->m_GLSInfo.MEQ + BaseSat;
					if (pMEQ_base->bMEQ == false) continue;
					ri[isys] = Hi;
					rl[isys] = 0;
				}

				if (BaseSat < 0 || i == BaseSat) continue;

				if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_PI][f])
				{
					// 双差方向余弦填充H矩阵
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 0]] = pMEQ->H_XYZ[0] - pMEQ_base->H_XYZ[0];
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 1]] = pMEQ->H_XYZ[1] - pMEQ_base->H_XYZ[1];
					pStruct->m_GLS.m_H[Hi * pStruct->m_GLS.m_nState + HI[pStruct->m_GLSInfo.IPos + 2]] = pMEQ->H_XYZ[2] - pMEQ_base->H_XYZ[2];

					// 双差残差
					pStruct->m_GLS.m_Inno[Hi] = pMEQ->v_P[f];

					double val = pStruct->m_GLSInfo.StateP[0] + pStruct->m_GLSInfo.StateP[MAXRTKNX + 1] + pStruct->m_GLSInfo.StateP[2 * MAXRTKNX + 2];
					if (val <= 900) // 暂时性举措，但注意对于中断后首历元不应调用，利用方差判断是否为中断后首历元
					{
						if (pStruct->m_SatStatus[IROVE][prn].S[f] > 35 && fabs(pMEQ->v_P[f]) > 3 && fabs(pMEQ->v_P[f]) > (pMEQ->R_P[f]))
						{
							pMEQ->R_P[f] = fabs(pMEQ->v_P[f]);
						}
						if (pStruct->m_SatStatus[IROVE][prn].S[f] > 45 && fabs(pMEQ->v_P[f]) > 2 && fabs(pMEQ->v_P[f]) > (pMEQ->R_P[f]))
						{
							pMEQ->R_P[f] = fabs(pMEQ->v_P[f]);
						}
					}

					// 流动站的观测方差,后面要转成星间单差的协方差阵
					pStruct->m_GLS.m_R[Hi * pStruct->m_GLS.m_nMeas + Hi] = pMEQ->R_P[f];

					Hi_iOBS[nHi_iOBS++] = i;
					pMEQ->GLSPosP[f] = Hi + 1; // prn卫星在GLS中的位置,+1是因为在计算验后残差时需要判断

					Hi++;
					rl[isys]++;
				}
			}

			// 转换为双差观测方差
			for (isys = 0; isys < NSYS; isys++)
			{
				BaseSat = pStruct->m_BaseSat[f][isys];
				pMEQ_base = pStruct->m_GLSInfo.MEQ + BaseSat;
				pMEQ = pStruct->m_GLSInfo.MEQ;

				for (int i = 0; i < rl[isys]; i++)
					for (int j = 0; j <= i; j++)
					{
						xi = ri[isys] + i;
						yi = ri[isys] + j;

						pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] += pMEQ_base->R_P[f];
						if (pMEQ[Hi_iOBS[xi]].RFactP[f] > 0.0)
							pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] *= xsqrt(pMEQ[Hi_iOBS[xi]].RFactP[f]);
						if (pMEQ[Hi_iOBS[yi]].RFactP[f] > 0.0)
							pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi] *= xsqrt(pMEQ[Hi_iOBS[yi]].RFactP[f]);
						pStruct->m_GLS.m_R[yi * pStruct->m_GLS.m_nMeas + xi] = pStruct->m_GLS.m_R[xi * pStruct->m_GLS.m_nMeas + yi];
					}
			}
		}
	}

	// 方程最小数判断
	if (Hi < 1)
	{
		return false;
	}

	return true;
}


static bool AnalyzePosterResi(CRTKPointBase* pStruct, int iMKF, int type)
{
	if (!pStruct)return false;

	int prn = 0, j;
	pStruct->m_GNSSQC.m_MIKF = iMKF + 1;
	pStruct->m_GNSSQC.m_nvP = 0;
	pStruct->m_GNSSQC.m_nvL = 0;
	pStruct->m_GNSSQC.m_RMSP = 0.0;
	pStruct->m_GNSSQC.m_RMSL = 0.0;

	// 得到验后残差
	if (type == OBS_PI)
	{
		SetValsD(pStruct->m_GNSSQC.m_v_P, 0, MAXOBSLIM);
		SetValsD(pStruct->m_GNSSQC.m_normv_P, 0, MAXOBSLIM);
		SetValsI(pStruct->m_GNSSQC.m_Pindex, 0, MAXOBSLIM);

		int nvP = 0;

		if (setMEQ_P(pStruct, 0) == false) return true;

		for (int f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			// 这里输入的Pv不能有0
			for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
				if (pMEQ->bMEQ == false) continue;

				j = pMEQ->GLSPosP[f] - 1;
				if (pMEQ->v_P[f] != 0 && j >= 0)
				{
					pStruct->m_GNSSQC.m_v_P[nvP] = pMEQ->v_P[f];
					if (pStruct->m_GLS.m_R[j * pStruct->m_GLS.m_nMeas + j] > 0.0)
						pStruct->m_GNSSQC.m_normv_P[nvP] = fabs(pMEQ->v_P[f]) / xsqrt(pStruct->m_GLS.m_R[j * pStruct->m_GLS.m_nMeas + j]);
					pStruct->m_GNSSQC.m_RMSP += pMEQ->v_P[f] * pMEQ->v_P[f];
					pStruct->m_GNSSQC.m_Pindex[nvP] = f * 1000 + prn;
					nvP++;
				}
			}
		}

		if (nvP > 0)
		{
			pStruct->m_GNSSQC.m_nvP = nvP;
			pStruct->m_GNSSQC.m_RMSP = xsqrt(pStruct->m_GNSSQC.m_RMSP / nvP);
		}
	}
	else if (type == OBS_LI)
	{
		SetValsD(pStruct->m_GNSSQC.m_v_L, 0, MAXOBSLIM);
		SetValsD(pStruct->m_GNSSQC.m_normv_L, 0, MAXOBSLIM);
		SetValsI(pStruct->m_GNSSQC.m_Lindex, 0, MAXOBSLIM);

		int nvL = 0;

		if (setMEQ_L(pStruct, 0) == false) return true;

		for (int f = 0; f < NFREQ; f++)
		{
			if (pStruct->m_bFreqUsed[f] == false) continue;

			// 这里输入的Lv不能有0
			for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
			{
				prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
				GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
				if (pMEQ->bMEQ == false) continue;

				j = pMEQ->GLSPosL[f] - 1;
				if (pMEQ->v_L[f] != 0 && j >= 0)
				{
					pStruct->m_GNSSQC.m_v_L[nvL] = pMEQ->v_L[f];
					if (pStruct->m_GLS.m_R[j * pStruct->m_GLS.m_nMeas + j] > 0.0)
						pStruct->m_GNSSQC.m_normv_L[nvL] = fabs(pMEQ->v_L[f]) / xsqrt(pStruct->m_GLS.m_R[j * pStruct->m_GLS.m_nMeas + j]);
					pStruct->m_GNSSQC.m_RMSL += pMEQ->v_L[f] * pMEQ->v_L[f];
					pStruct->m_GNSSQC.m_Lindex[nvL] = f * 1000 + prn;
					nvL++;
				}
			}
		}

		if (nvL > 0)
		{
			pStruct->m_GNSSQC.m_nvL = nvL;
			pStruct->m_GNSSQC.m_RMSL = xsqrt(pStruct->m_GNSSQC.m_RMSL / nvL);
		}
	}

	if (runAnalyzeResi(&pStruct->m_GNSSQC, true)) return true;

	for (int f = 0; f < NFREQ; f++)
	{
		if (pStruct->m_bFreqUsed[f] == false) continue;

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;

			int index = pMEQ->AmbFlag[f];
			if (index > 0)
			{
				int ambLoc;
				if ((ambLoc = GetAmbLoc(pStruct, f, prn)) < pStruct->m_GLSInfo.IAmb) continue;

				if (index == 1)
				{
					pStruct->m_GLSInfo.MKFStateP[ambLoc * MAXRTKNX + ambLoc] *= 8.0;
					pStruct->m_GLSInfo.MKFStateP[ambLoc * MAXRTKNX + ambLoc] += 0.000225; //SQR(dlam/1.0); //SQR(0.025);
				}
				else if (index == 2)
				{
					pStruct->m_GLSInfo.MKFStateP[ambLoc * MAXRTKNX + ambLoc] *= 10.0;
					pStruct->m_GLSInfo.MKFStateP[ambLoc * MAXRTKNX + ambLoc] += 4e-4; //SQR(dlam/1.0); //SQR(0.025);
				}
				else if (index == 9) // 通过验后分析发现周跳
				{
					pMEQ->RFactL[f] = 1.0;

					ReinitAmb(pStruct, prn, f);
				}
			}
		}
	}

	return false;
}


static bool runSubRTK(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	// GNSSMEQ部分信息清零
	zeroGNSSMEQ(pStruct->m_pOBSData[IROVE]->nsat, pStruct->m_GLSInfo.MEQ);

	const int nit = 2, nMKF = 10;
	int it, iMKF = 0;
	bool flag = false, bSuc[2] = { false }; // 0 = 伪距， 1 = 相位

	// 1.伪距观测更新
	if (pStruct->m_bUseP)
	{
		iMKF = 0;
		for (it = 1; iMKF <= nit; it++)
		{
			// 每次滤波时,都用最开始的方差,可以加快收敛,采用index,加快赋值速度
			EQU_StateP(pStruct->m_GLSInfo.MKFStateP, pStruct->m_GLSInfo.StateP, MAXRTKNX);

			flag = false;
			if (setMEQ_P(pStruct, it) == false) break;
			if (FormMEQ(pStruct, OBS_PI) == false)	break;
			if (GLS(&pStruct->m_GLS) == false)	break;

			// 得到GLS后的估计值和方差
			GetGLSXP(&pStruct->m_GLS, pStruct->m_GLSInfo.StateIndex, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX);

			flag = true;

			// 验后残差分析发现问题
			if (AnalyzePosterResi(pStruct, iMKF, OBS_PI) == false)
			{
				if (++iMKF < nMKF)
					Equ(pStruct->m_GLSInfo.StateIndex, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.StateX, MAXRTKNX);
				it = 1; // 重新迭代
			}
			else // 多系统RTKSBL参数太多,迭代计算会很慢,非线性程度不高,迭代一次也可以,只要验后残差没问题,直接退出
			{
				break;
			}
		}

		// 更新结果
		if (flag)
		{
			double azel[2][NSATMAX + 1] = { {0.0} }, DOPs[4] = { 0.0 };
			int isys = ISYSNON, BaseSat = 0, prn = 0, prn_base = 0;
			for (int f = 0; f < NFREQ; f++)
			{
				if (pStruct->m_bFreqUsed[f] == false) continue;

				isys = ISYSNON;

				for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
				{
					GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
					if (pMEQ->bMEQ == false) continue;
					prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

					// 先处理基准星
					if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
					{
						isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
						if ((BaseSat = pStruct->m_BaseSat[f][isys]) >= 0)
						{
							prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;
							azel[0][prn_base] = pStruct->m_SatStatus[IROVE][prn_base].azel[0];
							azel[1][prn_base] = pStruct->m_SatStatus[IROVE][prn_base].azel[1];
						}
					}

					if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

					if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_PI][f] && (pMEQ->GLSPosP[f] - 1) >= 0)
					{
						azel[0][prn] = pStruct->m_SatStatus[IROVE][prn].azel[0];
						azel[1][prn] = pStruct->m_SatStatus[IROVE][prn].azel[1];
					}
				}
			}

			// 卫星构型太差或验后残差太大或环境差时位置拉偏太多，认为伪距更新不可靠
			ComputeDOP(azel[0], azel[1], pStruct->m_pGNSSOpt->m_ElevMask, DOPs, NULL, false, false);
			double dxyz[3] = { 0.0 };
			M31_M31(pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.StateX, dxyz);
			// pStruct->m_GNSSQC.m_MedP >= 3.0 判断条件对于长时间复杂环境效果不好、会一直拒绝伪距更新
			if (/*DOPs[1] > 5.0 ||*/ /*pStruct->m_GNSSQC.m_MedP >= 3.0 || */(pStruct->m_bGoodEpoch == false && MatrixNorm2(3, 1, dxyz) >= 3.0))
			{
				flag = false;
				EQU_StateXP(pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.MKFStateP, pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX);
			}
			else
			{
				bSuc[OBS_PI] = true;
				EQU_StateXP(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.MKFStateP, MAXRTKNX);
				pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_RTD;
				if (pStruct->m_pGNSSSol)
				{
					pStruct->m_pGNSSSol->m_PDOP = DOPs[1];
					pStruct->m_pGNSSSol->m_DDOP = DOPs[1] * DOPs[1];
					pStruct->m_pGNSSSol->m_HDOP = DOPs[2];
					pStruct->m_pGNSSSol->m_VDOP = DOPs[3];
				}
			}
		}
	}

	// 2.相位观测更新
	if (pStruct->m_bUseL)
	{
		iMKF = 0;
		for (it = 1; iMKF <= nit; it++)
		{
			// 每次滤波时,都用最开始的方差,可以加快收敛,采用index,加快赋值速度
			EQU_StateP(pStruct->m_GLSInfo.MKFStateP, pStruct->m_GLSInfo.StateP, MAXRTKNX);

			flag = false;

			if (setMEQ_L(pStruct, it) == false)		break;
			if (FormMEQ(pStruct, OBS_LI) == false)	break;
			if (GLS(&pStruct->m_GLS) == false)		break;

			// 得到GLS后的估计值和方差
			GetGLSXP(&pStruct->m_GLS, pStruct->m_GLSInfo.StateIndex, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX);

			flag = true;

			// 验后残差分析发现问题
			if (AnalyzePosterResi(pStruct, iMKF, OBS_LI) == false)
			{
				if (++iMKF < nMKF) 
					Equ(pStruct->m_GLSInfo.StateIndex, pStruct->m_GLSInfo.nStateIndex, pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.StateX, MAXRTKNX);
				it = 1; // 重新迭代
			}
			else // 多系统RTKSBL参数太多,迭代计算会很慢,非线性程度不高,迭代一次也可以,只要验后残差没问题,直接退出
			{
				break;
			}
		}

		// 2.判断更新结果
		if (flag)
		{
			double azel[2][NSATMAX + 1] = { {0.0} }, DOPs[4] = { 0.0 };
			int isys = ISYSNON, BaseSat = 0, prn = 0, prn_base = 0;
			for (int f = 0; f < NFREQ; f++)
			{
				if (pStruct->m_bFreqUsed[f] == false) continue;

				isys = ISYSNON;

				for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
				{
					GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
					if (pMEQ->bMEQ == false) continue;
					prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

					// 先处理基准星
					if (isys != pStruct->m_SatStatus[IROVE][prn].sys_id) // 系统或频率不同时换参考星
					{
						isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
						if ((BaseSat = pStruct->m_BaseSat[f][isys]) >= 0)
						{
							prn_base = pStruct->m_pOBSData[IROVE]->obs[BaseSat].prn;

							pStruct->m_SatStatus[IROVE][prn_base].bResetAmb[f] = false;
							pStruct->m_SatStatus[IROVE][prn_base].lock[f]++;
							pStruct->m_SatStatus[IROVE][prn_base].outc[f] = 0;

							pStruct->m_SatStatus[IBASE][prn_base].lock[f]++;
							pStruct->m_SatStatus[IBASE][prn_base].outc[f] = 0;

							azel[0][prn_base] = pStruct->m_SatStatus[IROVE][prn_base].azel[0];
							azel[1][prn_base] = pStruct->m_SatStatus[IROVE][prn_base].azel[1];
						}
					}

					if (BaseSat < 0 || i == BaseSat) continue; // 基准星不需要星间单差

					if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f] && (pMEQ->GLSPosL[f] - 1) >= 0)
					{
						pStruct->m_SatStatus[IROVE][prn].bResetAmb[f] = false;
						pStruct->m_SatStatus[IROVE][prn].lock[f]++;
						pStruct->m_SatStatus[IROVE][prn].outc[f] = 0;

						pStruct->m_SatStatus[IBASE][prn].lock[f]++;
						pStruct->m_SatStatus[IBASE][prn].outc[f] = 0;

						azel[0][prn] = pStruct->m_SatStatus[IROVE][prn].azel[0];
						azel[1][prn] = pStruct->m_SatStatus[IROVE][prn].azel[1];
					}
				}
			}

			// 计算DOP值
			ComputeDOP(azel[0], azel[1], pStruct->m_pGNSSOpt->m_ElevMask, DOPs, NULL, false, false);
			double dxyz[3] = { 0.0 };
			M31_M31(pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.StateX, dxyz);
			if (/*DOPs[1] > 5.0 ||*/ (pStruct->m_bGoodEpoch == false && MatrixNorm2(3, 1, dxyz) >= 3.0))
			{
				flag = false;
				EQU_StateXP(pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.MKFStateP, pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, MAXRTKNX);
			}
			else
			{
				bSuc[OBS_LI] = true;
				pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_RTK_FLO;
			}
		}
	}

	flag = bSuc[0] ? bSuc[0] : bSuc[1];

	if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
	{
		pStruct->m_pGNSSSol->m_LOG.RTKSolType[0] = bSuc[0];
		pStruct->m_pGNSSSol->m_LOG.RTKSolType[1] = bSuc[1];
	}

	return flag;
}


static bool UpdateFloatSolution(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	// 观测更新失败直接采用速度递推
	if (pStruct->m_GLSInfo.m_StateSolType <= SOLTYPE_RTK_PRE)
	{
		// 待测试!!!!
		M31EQU(pStruct->m_GLSInfo.MKFStateX + pStruct->m_GLSInfo.IPos, pStruct->m_GLSInfo.StateX);
		double	ambmat[MAXOBSLIM * MAXOBSLIM];
		int s1[2] = { 3, 3 };
		int s2[2] = { 0, 0 };
		MatrixCopySub(s1, s2, MAXOBSLIM, MAXOBSLIM, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, MAXOBSLIM, MAXOBSLIM, ambmat, false);
		MatrixZero(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP);
		MatrixCopySub(s2, s2, 3, 3, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.MKFStateP, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, false);
		MatrixCopySub(s2, s1, MAXOBSLIM, MAXOBSLIM, MAXOBSLIM, MAXOBSLIM, ambmat, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, false);

		pStruct->m_RTKPredictEpoch++;
	}
	else
	{
		pStruct->m_RTKPredictEpoch = 0;
	}

	// 结果赋值
	if (pStruct->m_pGNSSSol)
	{
		pStruct->m_pGNSSSol->m_FilterTime = pStruct->m_OBSTime[IROVE];
		pStruct->m_pGNSSSol->m_SolType = pStruct->m_GLSInfo.m_StateSolType;
		if (pStruct->m_RTKPredictEpoch > pStruct->m_pGNSSOpt->m_RTKPredictMAXEpoch)
			pStruct->m_pGNSSSol->m_SolType = SOLTYPE_NONE; // 速度预报太久，结果不输出
		pStruct->m_pGNSSSol->m_time = pStruct->m_OBSTime[IROVE];
		pStruct->m_pGNSSSol->m_nsat = pStruct->m_pOBSData[IROVE]->nsat;
		for (int i = 0; i < NSYS; i++)pStruct->m_pGNSSSol->m_ngnss[i] = pStruct->m_pOBSData[IROVE]->ngnss[i];
		//pStruct->m_pGNSSSol->pStruct->m_PDOP = pStruct->m_DOPs[1]; // runSubRTK中已赋值
		//pStruct->m_pGNSSSol->pStruct->m_DDOP = SQR(pStruct->m_DOPs[1]);
		//pStruct->m_pGNSSSol->pStruct->m_HDOP = pStruct->m_DOPs[2];
		//pStruct->m_pGNSSSol->pStruct->m_VDOP = pStruct->m_DOPs[3];
		pStruct->m_pGNSSSol->m_pSatStatus = pStruct->m_SatStatus[IROVE];
		pStruct->m_pGNSSSol->m_pOBSData = pStruct->m_pOBSData[IROVE];
		M31EQU(pStruct->m_GLSInfo.StateX + pStruct->m_GLSInfo.IPos, pStruct->m_pGNSSSol->m_XYZPos);
		int s11[2] = { pStruct->m_GLSInfo.IPos, pStruct->m_GLSInfo.IPos }, s12[2] = { 0 };
		MatrixCopySub(s11, s12, pStruct->m_GLSInfo.LPos, pStruct->m_GLSInfo.LPos, MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.StateP, 3, 3, pStruct->m_pGNSSSol->m_XYZPosP, false);
		XYZ2LLH(pStruct->m_pGNSSSol->m_XYZPos, pStruct->m_pGNSSSol->m_LLHPos, 0);

		// 其他测速模式需补充
		if (pStruct->m_pGNSSOpt->m_PosEstMode == 3)
		{
			pStruct->m_pGNSSSol->m_VelType = pStruct->m_SPPoint.m_VAEKFSolType;
			M31EQU(pStruct->m_SPPoint.m_VAVel, pStruct->m_pGNSSSol->m_XYZVel);
			M33EQU(pStruct->m_SPPoint.m_VAVelP, pStruct->m_pGNSSSol->m_XYZVelP);
		}
	}

	return true;
}


static void runRTKIAR(CRTKPointBase* pStruct)
{
	if (!pStruct)return;

	if (pStruct->m_pGNSSOpt->m_ARMode == 0) return;

	pStruct->m_RTKIAR.m_pGLSInfo = &pStruct->m_GLSInfo;
	pStruct->m_RTKIAR.m_GLS = &pStruct->m_GLS;
	for (int i = 0; i < 2; i++)
	{
		pStruct->m_RTKIAR.m_SatStatus[i] = pStruct->m_SatStatus[i];
		pStruct->m_RTKIAR.m_pOBSData[i] = pStruct->m_pOBSData[i];
	}
	runOneIAR(&pStruct->m_RTKIAR);
}


static bool UpdateSolution(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	if (pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED)
	{
		ComputeDOP(NULL, NULL, pStruct->m_pGNSSOpt->m_ElevMask, pStruct->m_GLSInfo.DOPs, pStruct->m_SatStatus[IROVE], true, false);
	}

	// 模糊度固定满足一定条件时传递位置信息
	if ((pStruct->m_pGNSSOpt->m_ARMode == 1 || pStruct->m_pGNSSOpt->m_ARMode == 3) && pStruct->m_RTKIAR.m_AmbState == ARSTATE_FIXED)
	{
		if (pStruct->m_RTKIAR.m_nFixed >= 7 && pStruct->m_GLSInfo.DOPs[1] < 3.0 /*&& pStruct->m_RTKIAR.m_ARRatio > 3.0*/)
		{
			for (int i = pStruct->m_GLSInfo.IPos; i < pStruct->m_GLSInfo.LPos; i++)
			{
				pStruct->m_GLSInfo.StateX[i] = pStruct->m_GLSInfo.StateXa[i];
				for (int j = 0; j < MAXRTKNX; j++)
				{
					if (j == i)pStruct->m_GLSInfo.StateP[i * MAXRTKNX + j] = pStruct->m_GLSInfo.StatePa[i * MAXRTKNX + j];
					else
					{
						pStruct->m_GLSInfo.StateP[i * MAXRTKNX + j] = 0.0/*pStruct->m_GLSInfo.StatePa.at(i, j)*/;
						pStruct->m_GLSInfo.StateP[j * MAXRTKNX + i] = 0.0/*pStruct->m_GLSInfo.StatePa.at(j, i)*/;
					}
				}
			}
		}
	}

	// 结果赋值
	if (pStruct->m_RTKIAR.m_AmbState <= ARSTATE_FLOAT && pStruct->m_RTKIAR.m_AmbState_WL[1] == ARSTATE_FIXED)
		pStruct->m_RTKIAR.m_AmbState = pStruct->m_RTKIAR.m_AmbState_WL[1];
	else if (pStruct->m_RTKIAR.m_AmbState <= ARSTATE_FLOAT && pStruct->m_RTKIAR.m_AmbState_WL[0] == ARSTATE_FIXED)
		pStruct->m_RTKIAR.m_AmbState = pStruct->m_RTKIAR.m_AmbState_WL[0];

	const double* StateX = (pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FLOFIX) ? pStruct->m_GLSInfo.StateXa : pStruct->m_GLSInfo.StateX;
	const double* StateP = (pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FLOFIX) ? pStruct->m_GLSInfo.StatePa : pStruct->m_GLSInfo.StateP;
	if (pStruct->m_pGNSSSol)
	{
		// pos
		if (pStruct->m_RTKIAR.m_AmbState == ARSTATE_FLOFIX)pStruct->m_pGNSSSol->m_SolType = SOLTYPE_RTK_FLOFIX;
		else if (pStruct->m_RTKIAR.m_AmbState > ARSTATE_FLOFIX)
		{
			if (pStruct->m_RTKIAR.m_AmbState == ARSTATE_HOLD)pStruct->m_pGNSSSol->m_SolType = SOLTYPE_RTK_HOLD;
			else pStruct->m_pGNSSSol->m_SolType = SOLTYPE_RTK_FIXED;
		}
		else pStruct->m_pGNSSSol->m_SolType = pStruct->m_GLSInfo.m_StateSolType;
		M31EQU(StateX + pStruct->m_GLSInfo.IPos, pStruct->m_pGNSSSol->m_XYZPos);
		int s1[2] = { pStruct->m_GLSInfo.IPos, pStruct->m_GLSInfo.IPos }, s2[2] = { 0 };
		MatrixCopySub(s1, s2, pStruct->m_GLSInfo.LPos, pStruct->m_GLSInfo.LPos, MAXRTKNX, MAXRTKNX, StateP, 3, 3, pStruct->m_pGNSSSol->m_XYZPosP, false);
		XYZ2LLH(pStruct->m_pGNSSSol->m_XYZPos, pStruct->m_pGNSSSol->m_LLHPos, 0);

		// vel
		if (pStruct->m_GLSInfo.LVel > 0)
		{
			M31EQU(StateX + pStruct->m_GLSInfo.IVel, pStruct->m_pGNSSSol->m_XYZVel);
			int s12[2] = { pStruct->m_GLSInfo.IVel, pStruct->m_GLSInfo.IVel }, s22[2] = { 0 };
			MatrixCopySub(s12, s22, pStruct->m_GLSInfo.LVel, pStruct->m_GLSInfo.LVel, MAXRTKNX, MAXRTKNX, StateP, 3, 3, pStruct->m_pGNSSSol->m_XYZVelP, false);
		}
		else
		{
			// 其他测速模式需补充
			if (pStruct->m_pGNSSOpt->m_PosEstMode == 3)
			{
				M31EQU(pStruct->m_SPPoint.m_VAVel, pStruct->m_pGNSSSol->m_XYZVel);
				M33EQU(pStruct->m_SPPoint.m_VAVelP, pStruct->m_pGNSSSol->m_XYZVelP);
				pStruct->m_pGNSSSol->m_VelType = pStruct->m_SPPoint.m_VAEKFSolType;
			}
			//else
			//{
			//	M31EQU(pStruct->m_SPPoint.pStruct->m_SPPVel, pStruct->m_pGNSSSol->pStruct->m_XYZVel);
			//	M33EQU(pStruct->m_SPPoint.pStruct->m_SPPVelP, pStruct->m_pGNSSSol->pStruct->m_XYZVelP);
			//}
		}

		// other
		pStruct->m_pGNSSSol->m_pSatStatus = pStruct->m_SatStatus[IROVE];
		pStruct->m_pGNSSSol->m_nsat = pStruct->m_pOBSData[IROVE]->nsat;
		pStruct->m_pGNSSSol->m_AmbState = pStruct->m_RTKIAR.m_AmbState;
		pStruct->m_pGNSSSol->m_ARRatio = pStruct->m_RTKIAR.m_ARRatio;
		pStruct->m_pGNSSSol->m_pOBSData = pStruct->m_pOBSData[IROVE];
		double dXYZ[3]; M31_M31(pStruct->m_pGNSSSol->m_XYZPos, pStruct->m_BaseXYZ, dXYZ);
		Vxyz2enu(pStruct->m_BaseLLH, dXYZ, pStruct->m_pGNSSSol->m_BLENU);

		for (int i = 0; i < NSYS; i++)
		{
			pStruct->m_pGNSSSol->m_ngnss[i] = pStruct->m_pOBSData[IROVE]->ngnss[i];
			pStruct->m_pGNSSSol->m_vgnss[i] = 0;
			pStruct->m_pGNSSSol->m_fgnss[i] = 0;
		}

		// 输出模糊度固定的PDOP值
		pStruct->m_pGNSSSol->m_PDOP_IAR = 0.0;
		if (pStruct->m_RTKIAR.m_AmbState >= ARSTATE_FIXED)
		{
			pStruct->m_pGNSSSol->m_PDOP_IAR = pStruct->m_GLSInfo.DOPs[1];
		}

		for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
		{
			GNSS_MEQ* pMEQ = pStruct->m_GLSInfo.MEQ + i;
			if (pMEQ->bMEQ == false) continue;
			int prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
			int isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

			pStruct->m_pGNSSSol->m_vgnss[isys]++;

			// 参与模糊度固定的卫星数
			for (int f = 0; f < NFREQ; f++)
			{
				if (pStruct->m_bFreqUsed[f] == false) continue;

				if (pStruct->m_SatStatus[IROVE][prn].AR_Status[f] >= ARSTATE_FIXED)
				{
					pStruct->m_pGNSSSol->m_fgnss[isys]++;
					break;
				}
			}
		}
	}

	return true;
}


// for test
double checkDFAmbConsist(CRTKPointBase* pStruct, int prn, int f1, int f2)
{
	// 注意bprn为参考星，bXYZ为基站位置
	double* lam = pStruct->m_SatStatus[IROVE][prn].lam, damb1, damb2, ddamb = 9999;
	int irove, ibase, rambLoc1, rambLoc2;

	if ((rambLoc1 = GetAmbLoc(pStruct, f1, prn)) < 0)return ddamb;
	if ((rambLoc2 = GetAmbLoc(pStruct, f2, prn)) < 0)return ddamb;

	// 非参考星站间单差
	if ((irove = pStruct->m_SatStatus[IROVE][prn].OBSIndex) < 0)return ddamb;
	if ((ibase = pStruct->m_GLSInfo.MEQ[irove].OBSIndex) < 0)return ddamb;

	// 得到站间单差观测值
	if (lam[f1] == 0.0 || lam[f2] == 0.0)return ddamb;
	damb1 = lam[f1] * (pStruct->m_pOBSData[IROVE]->obs[irove].L[f1] - pStruct->m_pOBSData[IBASE]->obs[ibase].L[f1] - pStruct->m_GLSInfo.StateXa[rambLoc1]);
	damb2 = lam[f2] * (pStruct->m_pOBSData[IROVE]->obs[irove].L[f2] - pStruct->m_pOBSData[IBASE]->obs[ibase].L[f2] - pStruct->m_GLSInfo.StateXa[rambLoc2]);
	ddamb = damb1 - damb2;
	return ddamb;
}

static bool GetReferInfo(FILE* fp, GPSTIME gt, double* refXYZ) // 得到参考真值的数据
{
	static bool bInit = true;
	char oneline[1024] = { '\0' };
	int i = 0;
	bool flag = false;
	char delimiter[2] = " ";
	GPSTIME truTime;
	InitGPSTIME(&truTime);
	double XYZ[3] = { 0.0 };

	if (bInit)
	{
		while (!feof(fp) && bInit)
		{
			memset(oneline, '\0', 1024);
			fgets(oneline, 1024, fp);
			if (strncmp(oneline, "week", 4) == 0)bInit = false;
		}
	}
	if (bInit)return false;


	while (!feof(fp))
	{
		memset(oneline, '\0', 1024);
		fgets(oneline, 1024, fp);

		char* p = oneline;

		for (i = 0, p = strtok(p, delimiter); p; p = strtok(NULL, delimiter), i++)
		{
			if (i == 0) // GPS周
			{
				truTime.GPSWeek = atoi(p);
				// 这里后续需要加容错，以及切周
			}
			else if (i == 1) // GPS周内秒
			{
				double secs = atof(p);
				truTime.secsOfWeek = (int)secs;
				truTime.fracOfSec = secs - truTime.secsOfWeek;
			}
			else if (i == 16) // X坐标
			{
				XYZ[0] = atof(p);
			}
			else if (i == 17) // Y坐标
			{
				XYZ[1] = atof(p);
			}
			else if (i == 18) // Z坐标
			{
				XYZ[2] = atof(p);
			}
		}
		double dt = MinusGPSTIME(truTime, gt);
		if (fabs(dt) < 0.1)
		{
			flag = true;
			break;
		}
		else if (dt > 0.0)
		{
			rewind(fp);
			bInit = true;
			flag = false;
			break;
		}
		else if (dt < 0.0)
		{
			continue;
		}	
	}

	if (flag && refXYZ)M31EQU(XYZ, refXYZ);
	return flag;
}
bool runOneRTK(CRTKPointBase* pStruct)
{
	if (!pStruct)return false;

	// check license
	if (!CheckDogPermission(&pStruct->m_pOBSData[IROVE]->gt)) return false;

	if (IsGPSTIMEEqual(pStruct->m_pOBSData[IROVE]->gt, pStruct->m_OBSTime[IROVE])) return false; // 防止同一历元重复计算（return true改为false）
	if (pStruct->m_pOBSData[IROVE]->nsat < 5) return false; // 防止连续测速失败

	if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
	{
		InitLOGDATA(&pStruct->m_pGNSSSol->m_LOG);
		pStruct->m_pGNSSSol->m_LOG.gt = pStruct->m_pOBSData[IROVE]->gt;
		pStruct->m_pGNSSSol->m_LOG.nSat = pStruct->m_pOBSData[IROVE]->nsat;
	}

	// 1. 准备RTK
	if (PrepareRTK(pStruct) == false) return false;
	if (pStruct->m_GLSInfo.m_StateSolType == SOLTYPE_NONE)return false; // 更新失败
	if (pStruct->m_GapTime > DTTOL_M) return true;
	MatrixZero(MAXRTKNX, 1, pStruct->m_GLSInfo.MKFStateX);
	MatrixZero_diag(MAXRTKNX, MAXRTKNX, pStruct->m_GLSInfo.MKFStateP);
	EQU_StateXP(pStruct->m_GLSInfo.StateX, pStruct->m_GLSInfo.StateP, pStruct->m_GLSInfo.MKFStateX, pStruct->m_GLSInfo.MKFStateP, MAXRTKNX);

	// 2.计算浮点解
	double rXYZ[3] = {0.0};
	static FILE* ref_fp = NULL;
	if (!ref_fp)fopen_s(&ref_fp, pStruct->m_RefFn, "rb");
	if (ref_fp) // 动态模式
	{
		GetReferInfo(ref_fp, pStruct->m_OBSTime[IROVE], rXYZ);//与CGNSSApplication.c中的GetReferInfo功能不一样，本函数实现从ref_fp中寻找"想要时间"对应的真值
	}
	else if (pStruct->m_RefXYZ[0] != 0 && pStruct->m_RefXYZ[1] != 0 && pStruct->m_RefXYZ[2] != 0) // 静态模式
	{
		M31EQU(pStruct->m_RefXYZ, rXYZ);
	}
	double dt = fabs(MinusGPSTIME(pStruct->m_OBSTime[IROVE], pStruct->m_OBSTime[IBASE]));
	if (rXYZ[0] != 0 && rXYZ[1] != 0 && rXYZ[2] != 0 && dt < 1.0)
	{
		// GNSSMEQ部分信息清零
		zeroGNSSMEQ(pStruct->m_pOBSData[IROVE]->nsat, pStruct->m_GLSInfo.MEQ);
		M31EQU(rXYZ, pStruct->m_GLSInfo.StateX);
		pStruct->m_GLSInfo.m_StateSolType = SOLTYPE_RTK_FLO;
		setMEQ_P(pStruct, 0); // 计算双差伪距
		setMEQ_L(pStruct, 0); // 计算双差相位
		settd_VL(pStruct);// 计算三差相位
	}
	//runSubRTK(pStruct);
	// 3. 更新浮点解状态,为固定解准备
	//UpdateFloatSolution(pStruct);
	//if (pStruct->m_GLSInfo.m_StateSolType <= SOLTYPE_RTK_PRE)return true; // 更新失败

	// 4. 进入RTK固定解
	//runRTKIAR(pStruct);

	// 5. 更新固定解
	//UpdateSolution(pStruct);

	return true;
}
