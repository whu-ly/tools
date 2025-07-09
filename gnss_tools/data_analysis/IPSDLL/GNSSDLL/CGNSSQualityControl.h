/**
* @file
* @brief    GNSS Quality Control
* @details  GNSS数据质量控制,验前/验后残差分析与质量控制 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CGNSSQUALITYCONTROL_H
#define GNSSTK_CGNSSQUALITYCONTROL_H

#include "CGNSSMember.h"


/* GNSS Residual Analysis ----------------------------------------------------
* note:
* ----------
* GNSS残差分析,根据KF的验前残差和验后残差,综合分析发现观测值中存在的异常
* 对伪距和相位分别分析,最终采取降权抗差的方法抵御观测异常的影响
*
*-----------------------------------------------------------------------------*/
typedef struct tagCGNSSQualityControl
{
    ///< 外部初始化信息
    int             m_PMode;                                // 0:SPP,1:PPP,2:RTK,3:PPPTC,4:RTKTC
    SATSTATUS*      m_pSatStatus;
    CGNSSOption*    m_pGNSSOpt;
    OBS_DATA*       m_pOBSDataRove;
    GNSS_MEQ*       m_pMEQ;


    ///< 残差分析输入
    //double          m_Chisqr_Sigam0;                        // 验后残差的卡方分布检验量
    //int             m_Chisqr_size;                          // 验后残差的卡方分布的自由度
    bool            m_bReinitAllAmb[NFREQ];                 // 所有模糊度是否重新初始化
    bool            m_bFoundNewCS;                          // 通过残差分析发现新的周跳
    int             m_MIKF;                                 // 一次解算中,当前是第i次MKF

    double          m_v_L[MAXOBSLIM];                       // 相位(L)的残差
    double          m_v_P[MAXOBSLIM];                       // 伪距(P)的残差
    double          m_normv_L[MAXOBSLIM];                   // 相位标准化残差
    double          m_normv_P[MAXOBSLIM];                   // 伪距标准化残差
    int             m_Lindex[MAXOBSLIM];                    // 相位残差索引
    int             m_Pindex[MAXOBSLIM];                    // 伪距残差索引
    int             m_nvL;                                  // 相位残差数量
    int             m_nvP;                                  // 伪距残差数量

    double          m_RMSL;                                 // 相位残差标准差
    double          m_RMSP;                                 // 伪距残差标准差
    //double          m_MedP;                                 // 伪距残差中位数

    ///< 验后残差指标变量
    //int             m_WorstSat[NFREQ][5];                   // 0:norm_vP,1:vP,2:norm_vL,3:vL,4:norm_vD
    //int             m_SatBadIndex[NSATMAX + 1][NFREQ][5];   // 0:norm_vP,1:vP,2:norm_vL,3:vL,4:norm_vD
} CGNSSQualityControl;

extern void InitCGNSSQualityControl(CGNSSQualityControl* pStruct);

extern bool InitQC(CGNSSQualityControl* pStruct, int PMode, CGNSSOption* pGNSSOpt, SATSTATUS* pSatStatus, OBS_DATA* pOBSDataRove, GNSS_MEQ* pMEQ);

extern bool runAnalyzeResi(CGNSSQualityControl* pStruct, bool bPosteriori); // 返回true表明分析没有问题


#endif
