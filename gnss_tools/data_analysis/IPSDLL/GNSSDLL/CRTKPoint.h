/**
* @file
* @brief    Real-time Kinematic Positioning (RTK)
* @details  相对定位 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CRTKPOINT_H
#define GNSSTK_CRTKPOINT_H

#include "CGNSSMember.h"
#include "CSPPoint.h"
#include "CCycleSlipDetection.h"
#include "CGNSSQualityControl.h"
#include "CGNSSIAR.h"

typedef struct tagCRTKPointBase
{
	///< 外部初始化信息, 调用InitRTK初始化
	CGNSSOption*			m_pGNSSOpt;						///< GNSS opt
	CGNSSSolution*			m_pGNSSSol;						///< GNSS solution
	OBS_DATA*				m_pOBSData[2];					///< 流动站/基准站数据体


	///< 其他变量
	CEphemerisComputer		m_EphComputer;					///< 星历计算器	
	CSPPoint				m_SPPoint;						///< 流动站/基准站SPP
	CTroposphere			m_TropModel;					///< 对流层模型
	CCycleSlipDetection		m_CSD[2];						///< 流动站/基准站的周跳,钟跳探测
	CGNSSQualityControl     m_GNSSQC;						///< GNSS质量控制
	CGLSEstimator			m_GLS;							///< 广义最小二乘估计器
	CGNSSIAR				m_RTKIAR;						///< 模糊度固定
	GNSS_GLSINFO			m_GLSInfo;						///< 解算信息
	SATSTATUS				m_SatStatus[2][NSATMAX + 1];	///< 流动站/基准站卫星状态, 存储每颗卫星的信息
	GPSTIME					m_OBSTime[2];					///< 基站、流动站观测时间
	GPSTIME					m_OBSTimePre[2];				///< 基站、流动站前个历元的观测时间

	bool					m_bInitRTK;						///< 初始化标识
	bool					m_bFreqUsed[NFREQ];				///< 当前所使用的频率
	bool					m_bUseP;						///< 观测更新时，采用伪距或相位更新
	bool					m_bUseL;
	bool					m_bGoodEpoch;					///< 通过周跳比例等标识判断当前环境优劣
	double					m_OBSGapTime[2];				///< 基站、流动站当前时间间隔
	double					m_GapTime;				        ///< 流动站与基准站的时间差
	double					m_BaseXYZ[3];					///< 基站坐标
	double					m_BaseLLH[3];					///< 基站坐标
	int						m_BaseSat[NFREQ][NSYS];			///< 星间单差基准星,不同系统不同频率的基准星不同,存的Prn号(不是prn号，是第i个观测卫星)
	int						m_PreBasePrn[NFREQ][NSYS];		///< 上一历元星间单差基准星，存prn号。
	int						m_nMinSatThres;					///< 站间单差最小卫星数
	int						m_RTKPredictEpoch;				///< RTK递推历元数
	
	char					m_RefFn[1024];					///< 流动站真值坐标文件名（动态）
	double					m_RefXYZ[3];					///< 流动站真值坐标（静态）
} CRTKPointBase;

extern void InitCRTKPointBase(CRTKPointBase* pStruct);

extern bool InitRTK(CRTKPointBase* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, OBS_DATA* pROBSData, OBS_DATA* pBOBSData);

extern bool runOneRTK(CRTKPointBase* pStruct);

#endif
