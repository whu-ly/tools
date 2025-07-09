/**
* @file
* @brief    Members Class of GNSS Application
* @details  GNSS App的成员类 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CGNSSMEMBER_H
#define GNSSTK_CGNSSMEMBER_H

#include "GNSSSDC.h"


/**
* @brief       GNSS配置
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
typedef struct tagCGNSSOption
{
    ///< GNSS处理方式
    double    m_SampleTime[2];             ///< 数据采样率，需手动设置，默认1s
    int       m_PMode;                     ///< 解算模式
    int       m_BaselineType;              ///< (0:short baseline(<10km),1:long baseline(>10km),2:short to long baseline)
    int       m_SatSYS;                    ///< 卫星系统
    bool      m_bFreqUsed[NFREQ];          ///< 使用L1,L2,L3哪些频率
    double    m_BaseXYZ[MAXBASESITE][3];   ///< 基站坐标
    int       m_BaseSiteID;                ///< 当前所使用的基准站ID
    //double    m_ProcTime;                  ///< 数据处理频率，需手动设置，默认与m_SampleTime[IROVE]相同
    //bool      m_bDownProcTime;             ///< 是否降低数据频率，降低后将采用“低频观测更新+高频速度预报”形式处理数据
    int       m_RTKPredictMAXEpoch;		   ///< RTK滤波预报最大历元数

    ///< GNSS预处理参数
    double    m_ElevMask;                  ///< 截止高度角
    double    m_ErrorPLR[NSYS];            ///< code/phase error ratio
    double    m_Error_a;
    double    m_Error_b;
    double    m_Error_EF[NSYS];            ///< error factor: GPS GLO BDS GAL
    double    m_SNRThres[NFREQ];
    bool      m_bCJ;                       ///< 钟跳探测与修复
    bool      m_bCS_LLI;                   ///< LLI周跳探测
    bool      m_bCS_GF;                    ///< GF周跳探测
    bool      m_bCS_MW;                    ///< MW周跳探测

    ///< GNSS误差改正

    ///< GNSS 模糊度固定
    int       m_ARMode;                     ///< 模糊度固定方式,[0:OFF,1:CONT,2:INST,3:HOLD]
    int       m_ARType;                     ///< 模糊度固定类型,[0:OFF,1:PPPAR_FCB,2:PPPAR_IRC]
    int       m_ARMinLock;                  ///< 模糊度固定需要连续进行浮点解算的最少历元数
    int       m_ARAmbLock[NFREQ];           ///< NL/WL/EWL或L1/L2/L5连续跟踪的最少历元数
    double    m_AR_PDOP;			        ///< 固定成整数的卫星PDOP,小于该阈值,固定失败
    int       m_AR_nSat;                    ///< 固定成整数的卫星个数阈值,小于该阈值,固定失败
    double    m_AR_ADOP;                    ///< ADOP阈值,大于该阈值,固定失败
    double    m_FixConsistCheck;
    double    m_ARFixAmbRMS;                ///< 固定模糊度残差RMS
    int       m_ARFix_nFixEpoch;            ///< 某颗卫星连续固定成同一整数的历元数,大于该阈值,进入ARHold
    int       m_ARHold_nSat;                ///< 连续固定成同一整数的卫星个数,大于该阈值,进入ARHold
    double    m_ARHold_ADOP;                ///< ADOP阈值,小于该阈值,进入ARHold
    double    m_ARHold_SRBS;                ///< BootStrapping成功率阈值,大于该阈值,进入ARHold
    double    m_ARHold_Ratio;               ///< ARRatio值,大于该阈值,进入ARHold
    int       m_ARHold_nFixEpoch;           ///< 某颗卫星连续固定成同一整数的历元数, 大于该阈值, 进入ARHold
    int       m_ARHold_nEpoch;              ///< 满足上述ARHold所有条件的连续历元数, 大于该阈值, 进入ARHold

    ///< GNSS大气误差修正

    ///< GNSS数据质量控制
    double    m_DataGapTol;                 ///< GNSS数据最大中断时间,超过以后状态初始化
    int       m_MaxOutage;                  ///< 卫星失锁阈值, 超过模糊度初始化
    double	  m_DDOPThres;			        ///< QC-多普勒历元差阈值(m)
    double    m_DCodeThres;			        ///< QC-伪距历元差减去多普勒阈值(m)
    double    m_DPhaseThres;		        ///< QC-相位历元差减去多普勒阈值(m)

    ///< GNSS参数估计策略
    int		  m_PosEstMode;			        ///< 0:白噪声估计,1:常量随机游走,2:Dis/Vel驱动模型,3:加速度/速度模型(VA-EKF)

    ///< 统计假设检验

    ///< GNSS误差模型参数
    ///< pos
    double BASE_ISDV_Vel2;       ///< (m/s)2
    double BASE_ISDV_Acc2;       ///< (m/s^2)2
    double BASE_PNSD_Pos2;       ///<
    double BASE_PNSD_Vel2;
    double BASE_PNSD_Acc2;
    double BASE_SPPPosSigma2;    ///< 使用SPP位置做为预报值时的方差

    ///< clk
    double BASE_ISDV_ClkVel2;
    double BASE_PNSD_ClkVel2;
    double BASE_SPPClkSigma2;    ///< 使用SPP钟差做为预报值时的方差
    double BASE_SPPClkVelSigma2;

    double RTK_ISDV_Amb2;        ///< m2

    GNSS_Device_TYPE    m_GNSSDeviceType;     ///< GNSS设备类型

} CGNSSOption;

extern void InitCGNSSOption(CGNSSOption* pStruct);


/**
* @brief       GNSS解算结果
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
typedef struct tagCGNSSSolution
{
    CGNSSOption* m_pGNSSOpt;

    GPSTIME m_time;                         ///< GPS 时间, 非钟面时
    GPSTIME m_FilterTime;                   ///< 滤波时间，始终和StateX保持一致，十分注意

    double  m_XYZPos[3];
    double  m_XYZPosP[9];
    double  m_LLHPos[3];
    double  m_ENUPosP[9];
    double  m_XYZVel[3];
    double  m_XYZVelP[9];
    double  m_ENUVel[3];
    double  m_ENUVelP[9];
    double  m_Att[3];                       ///< 双天线得到的姿态,北向起算的航向角
    double  m_AttP[9];
    double  m_Rel[9];
    double  m_BLENU[3];
    int     m_nsat;
    int     m_ngnss[NSYS];
    int     m_vgnss[NSYS];                  ///< visible
    int     m_fgnss[NSYS];                  ///< Fixed
    double  m_DDOP;                         ///< 双差DOP值,流动站和基站DOP之积
    double  m_PDOP;
    double  m_PDOP_IAR;                     ///< 模糊度固定卫星的PDOP值
    double  m_HDOP;
    double  m_VDOP;
    int     m_AmbState;
    int     m_VelType;                      ///< 速度解状态，需配合不同测速模式 m_PosEstMode 理解
    int     m_SolType;                      ///< 位置解状态
    double  m_ratioScore[3];                ///< 部分法固定中，三种迭代模式中ratio最高值
    double  m_ARRatio;
    double  m_RMSL;                         ///< 相位验后RMS
    double  m_RMSP;                         ///< 伪距验后RMS
    char    m_SolGGA[MAXSIZE];
    char    m_SolRMC[MAXSIZE];
    char    m_SolLOG[MAXSIZE];

    SATSTATUS* m_pSatStatus;
    OBS_DATA* m_pOBSData;                   ///< 数据体
    bool   m_bUseLOG;                       ///< 是否记录LOG信息
    LOG_DATA m_LOG;                         ///< log文件

} CGNSSSolution;

extern void InitCGNSSSolution(CGNSSSolution* pStruct);

extern bool WriteSolution(CGNSSSolution* pStruct);


#endif
