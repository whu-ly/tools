/**
* @file
* @brief    General Least Square Estimator or Kalman Filter
* @details  广义最小二乘估计器,KF滤波器 \n
*           Note that all memory allocation is static, so it is important to pay attention to the memory dimension of MAXOBSLIM
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_LSESTIMATOR_H
#define BASETK_LSESTIMATOR_H

#include "BaseMatrix.h"


///< 静态数据，需注意MAXOBSLIM和MAXSPPNX，后者根据SPP实际情况设置
typedef struct tagCLSEstimator
{
    ///< 输入
    double  m_B[MAXOBSLIM * MAXLSQNX];		///< 观测矩阵
    double  m_P[MAXOBSLIM * MAXOBSLIM];		///< 观测权阵,不是协防差阵
    double  m_L[MAXOBSLIM];				    ///< 观测值

    ///< 输出
    double  m_StateX[MAXLSQNX];			    ///< 估计的状态
    double  m_StateP[MAXLSQNX * MAXLSQNX];	///< 估计的状态协方差
    double  m_V[MAXOBSLIM];				    ///< 改正数 V = BX - W

    double m_Sigma;			                ///< 单位权中误差(处理过后的,与协因数矩阵相乘后,得到状态协方差阵)
    double m_RMS;			                ///< 验后残差RMS

    bool m_bInit;			                ///< 是否初始化LSQ
    bool m_bPDiagonal;		                ///< 观测权阵P是否为对角线
    int  m_nState;			                ///< 状态数
    int  m_nMeas;			                ///< 观测数
    double  m_W[MAXLSQNX];				    ///< BTPL
    double m_SigmaPre;		                ///< 当前sigma不能用时,用前一个代入     

} CLSEstimator;

void InitCLSEstimator(CLSEstimator* pStruct);
bool InitLSQ(CLSEstimator* pStruct, int nState, int nMeas, bool bPDiagonal);
void ClearLSQ(CLSEstimator* pStruct);
//bool LSQ(CLSEstimator* pStruct, bool bPostEst = false);
bool LSQ(CLSEstimator* pStruct, bool bPostEst);


typedef struct tagCGLSEstimator
{
    bool m_bMDB;			                ///< 是否需要MDB
    bool m_bInnoChisq;		                ///< 是否需要新息卡方分布
    bool m_bResiChisq;		                ///< 是否需要残差卡方分布

    ///< 输入
    double  m_StateXp[MAXGLSNX];	        ///< 估计的状态, nS x 1
    double  m_StatePp[MAXGLSNX * MAXGLSNX]; ///< 估计的状态协方差, nS x nS
    double  m_H[MAXOBSLIM * MAXGLSNX];	    ///< 观测矩阵, nM x nS
    double  m_R[MAXOBSLIM * MAXOBSLIM];	    ///< 观测协防差阵, nM x nM
    double  m_Inno[MAXOBSLIM];		        ///< 新息, nM x 1

    ///< 输出
    double  m_StateX[MAXGLSNX];			    ///< 估计的状态, nS x 1
    double  m_StateP[MAXGLSNX * MAXGLSNX];	///< 估计的状态协方差, nS x nS
    double  m_dXl;			                ///< 状态变化量的模,作为迭代收敛的判断

    double m_InnoChisq;		                ///< 新息卡方分布值
    double m_ResiChisq;		                ///< 残差卡方分布值
    double m_InnoSigma0;	                ///< 新息sigma0
    double m_ResiSigma0;	                ///< 残差sigma0
    double m_RMS;			                ///< 验后残差RMS

    bool m_bInit;			                ///< 是否初始化LSQ

    int m_nState;			                ///< 状态数
    int m_nMeas;			                ///< 观测数

    ///< 中间变量
    double  m_Rp[MAXOBSLIM * MAXOBSLIM];    ///< 预报观测值的协防差阵, nM x nM
    double  m_K[MAXGLSNX * MAXOBSLIM];      ///< Kalman滤波增益矩阵, nS x nM
    bool m_bPDiagonal;		                ///< 观测权阵P是否为对角线

} CGLSEstimator;

void InitCGLSEstimator(CGLSEstimator* pStruct);
void ClearGLS(CGLSEstimator* pStruct);
bool InitGLS(CGLSEstimator* pStruct, int nState, int nMeas, bool bPDiagonal);
bool GLS(CGLSEstimator* pStruct);

void SetGLSXP(CGLSEstimator* pStruct, int* index, int size, double* X, double* P, int row);
void GetGLSXP(CGLSEstimator* pStruct, int* index, int size, double* X, double* P, int row);


#endif
