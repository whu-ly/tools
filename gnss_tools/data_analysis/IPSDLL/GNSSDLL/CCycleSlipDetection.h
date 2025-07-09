/**
* @file
* @brief    GNSS Cycle Slip Detection
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


#ifndef GNSSTK_CCYCLESLIPDETECTION_H
#define GNSSTK_CCYCLESLIPDETECTION_H

#include "CGNSSMember.h"

/* Data Detection ------------------------------------------------------------
* note: ???之处需要修改
* ----------
* 周跳, 钟跳, 粗差探测
* 对于周跳修复,以下不同时发生的周跳还没解决
* G01 ---- +-----------
* G02 ----  +----------
* G03 ----   +---------
* 目前最多中断5个历元会修复,否则不进行修复
*
* 周跳修复需要注意的点:
* 1. 惯导递推的位移精度优于历元间差分计算的位移时,约束是有效的,可以提高ratio值
* 2. 电离层约束的方差对周跳估计有明显的影响
* 3. 相位/伪距的方差对周跳估计有影响
* 4. 伪距精度好时,自身就可以估计电离层,电离层方差就可以大点
*
* 2018/06/21:
* 1. sagnac之前没改,影响很大,
* 2. 低高度角卫星的对流层延迟由于投影函数的变化,还是很明显,干湿分量都要改正,但是湿分量只有PPP浮点
*    解估计的准确些,其它诸如PPPTC,固定解之类的都用模型改正
* 3. 静态数据还是不加静止约束好些
* 4. 周跳修复完成以后,再次周跳探测,此时的周跳阈值比较关键,需要和中断时长联系,此时的电离层延迟怎么修正,
*    使用建模的值还是再次算个L4?
* 5. 电离层方差很重要,用固定值1cm-3cm有时候还好些,电离层建模长度有影响,实质还是反映在电离层建模精度
* 6. 固定周跳值时的策略,现在把不经过ratio检验的作为第二备选,但有时候还是直接固定的好
* 7. 周跳固定时,用高度角的PAR和内置的PAR有好有差,两者应结合
*
* 2018/06/28:
* 1. BDS GEO卫星的GF2周跳阈值改成和其它卫星一样,原来的固定值偏小,导致实际动态数据经常周跳
*
*-----------------------------------------------------------------------------*/
typedef struct tagCCycleSlipDetection
{
    ///< 外部初始化信息
    OBS_DATA*       m_pOBSData;				    ///< 观测数据体
    GPSTIME         m_OBSTime;					///< 观测时间
    CGNSSOption*    m_pGNSSOpt;				    ///< GNSS opt
    SATSTATUS*      m_pSatStatus;				///< 卫星状态
    double          m_OBSGapTime;				///< 断链的时间长度(s)
    bool            m_bFreqUsed[NFREQ];		    ///< 使用哪些频率
    int             m_nOutEpoch;				///< 丢失的历元数
    double          m_SampleTime;               ///< 数据采样率

    ///< 其他信息
    OBS_DATA_t_PL   m_OBSDataPre[NSATMAX + 1];  ///< 钟跳修复时用
    bool            m_bReinitCJByAmb;			///< 只要所有模糊度重新初始化,CJ的状态也初始化
    double          m_GFThres;					///< GF1周跳阈值
    double          m_GF2Thres;				    ///< GF2周跳阈值
    double          m_MWThres;					///< MW周跳阈值
    TFLCTYPE        m_iFreq[2];				    ///< 指定两个频率组合观测值
    int             m_iDF;						///< 指定的两个频率的信息所存位置
    int             m_TotalClkJump;			    ///< 总的钟跳累加(ms)
    //int             m_MaxOutage;				///< 最大丢失历元数阈值
} CCycleSlipDetection;

extern void InitCCycleSlipDetection(CCycleSlipDetection* pStruct);

extern bool InitCS(CCycleSlipDetection* pStruct, CGNSSOption* pGNSSOpt, SATSTATUS* pSatStatus, int index);

extern bool runOneCSD(CCycleSlipDetection* pStruct);


//// 周跳,钟跳探测函数
//void RepairRcvClockJump();				///< 钟跳修复
//bool runOneCSDSub();					///< 双频周跳探测
//void DetectCS_LLI();					///< LLI周跳探测,建议不用
//void DetectCS_GF();						///< 二阶差分GF周跳探测
//void DetectCS_MW();						///< MW组合周跳探测


#endif
