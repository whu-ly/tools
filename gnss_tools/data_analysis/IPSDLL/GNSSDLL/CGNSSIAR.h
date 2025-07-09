/**
* @file
* @brief    GNSS Integer Ambiguity Resolution
* @details  GNSS模糊度固定 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CGNSSIAR_H
#define GNSSTK_CGNSSIAR_H

#include "CLSEstimator.h"
#include "CILSEstimator.h"
#include "GNSSSDC.h"
#include "CGNSSMember.h"


typedef struct tagCGNSSIAR
{
    // 外部初始化信息
    CGNSSOption*        m_pGNSSOpt;				            ///< GNSS Options
    CGLSEstimator*		m_GLS;					            ///< 为Hold模式所用的估计器
    SATSTATUS*          m_SatStatus[2];			            ///< 卫星状态, 存储每颗卫星的信息
    OBS_DATA*           m_pOBSData[2];			            ///< 数据体
    GNSS_GLSINFO*       m_pGLSInfo;				            ///< GLS解算信息
    CGNSSSolution*      m_pGNSSSol;                         ///< 
    bool				m_bFreqUsed[NFREQ];		            ///< 哪些频率可用

    // 其他变量
    bool                m_bUseILS;                          ///< 开启ILS估计（PAR）
    bool                m_bUseBIE;                          ///< 开启BIE估计，若与m_bUseILS同时开启，则先执行ILS、失败后执行BIE
    bool                m_bFixWL;                           ///< 是否固定宽巷模糊度
    bool				m_bReinitAllAmb[NFREQ];	            ///< 外部是否全部初始化模糊度
    //bool				m_PARMode;				            ///< PAR方式, false:内置模糊度固定

    double				m_ARRatio;				            ///< ARRatio值
    //double			m_ARRatioPre;		                ///< 前一固定历元的ARRatio值
    double				m_ADOP;					            ///< ADOP
    double				m_SR_ADOP;				            ///< ADOP成功率,上界
    double				m_SR_Bootstrap;			            ///< BS成功率,下界
    double				m_ADOPFull;				            ///< 进入到LAMBDA中的浮点模糊度的ADOP值
    double				m_SR_ADOPFull;			            ///< 进入到LAMBDA中的浮点模糊度的ADOP成功率
    double				m_SR_BootstrapFull;		            ///< 进入到LAMBDA中的浮点模糊度的Bootstrapping成功率
    int					m_AmbState;				            ///< 模糊度状态
    int					m_AmbStatePre;			            ///< 前一个模糊度状态
    int					m_nFixed;				            ///< 固定的卫星数(双频时也是卫星数,非模糊度个数)
    int					m_nPara[4];				            ///< 存储参数个数,0:非模糊度参数(小于模糊度参数起始索引),1:基准星,2:有效的单差模糊度,3:所有的单差模糊度
    int					m_nSatARHold;			            ///< 满足Hold的卫星个数
    int					m_nARHold;				            ///< 连续满足Hold的历元数
    int					m_BasePrns[NFREQ][NSYS];            ///< 基准星

    CILSEstimator       m_ILS;                              ///< 整数估计器

    int			        m_CommonPrnsPre[MAXOBS];		    ///< 前一历元共有的卫星prn
    int			        m_nCommonPrnsPre;		            ///< m_CommonPrnsPre 中元素数量
    char		        m_PrnFrq_iAmb[NSATMAX * NFREQ];	    ///< 由 Frq Prn 得到 FLOAmb 中模糊度索引,从1开始,使用时减1
    int			        m_iAmb_PrnFrq[MAXOBSLIM];		    ///< 由 FLOAmb 中模糊度索引得到 Frq Prn，储存方式为 1000*Frq+Prn
    int                 m_niAmb_PrnFrq;                     ///< m_iAmb_PrnFrq 中元素数量
    int			        m_PARIndex[3][MAXOBSLIM];			///< 部分模糊度固定PAR的指标排序：信噪比、高度角和模糊度方差
    double				m_FLOAmb[MAXOBSLIM];			    ///< 存储当前有效的双差模糊度
    double				m_FLOAmbP[MAXOBSLIM * MAXOBSLIM];   ///< 存储当前有效的双差模糊度
    double				m_FIXAmb[MAXOBSLIM];				///< 存储当前有效的双差模糊度
    double				m_HoldStateX[MAXRTKNX];			    ///< 存储当前所有的有效状态
    double				m_HoldStateP[MAXRTKNX* MAXRTKNX];	///< 存储当前所有的状态方差
    double				m_HoldD[MAXOBSLIM * MAXRTKNX];		///< 单差转双差矩阵

    bool                m_bILSFixed;                        ///< ILS解算标识，true=解算成功
    bool                m_bBIEFixed;                        ///< BIE解算标识，true=解算成功
    bool                m_bFixQualityPre;                   ///< ILS固定质量，固定较差时置为false
    bool                m_bFixQuality;                      ///< ILS固定质量，固定较差时置为false
    bool                m_bAmbFixed[MAXOBSLIM];             ///< 模糊度是否固定成功，与m_nPara[2]联合使用，用于参数更新
    bool                m_bUseILSconsBIE;                   ///< 是否使用ILS约束BIE的候选解搜索   

    // 宽巷模糊度固定相关
    char		        m_Prn_iAmb_WL[NSATMAX];	            ///< 应用于宽巷固定中
    int			        m_iAmb_Prn_WL[MAXOBSLIM];			///< 应用于宽巷固定中
    int                 m_niAmb_Prn_WL;                     ///< m_iAmb_Prn_WL 中元素数量
    int					m_AmbState_WL[2];		            ///< 超宽巷，宽巷模糊度状态

} CGNSSIAR;

extern void InitCGNSSIAR(CGNSSIAR* pStruct);

extern bool InitIAR(CGNSSIAR* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, bool bFreqUsed[NFREQ]);

extern bool runOneIAR(CGNSSIAR* pStruct);

#endif
