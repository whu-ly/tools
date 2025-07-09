/**
* @file
* @brief    Ephemeris Computer
* @details  星历计算器 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CEPHEMERISCOMPUTER_H
#define GNSSTK_CEPHEMERISCOMPUTER_H


#include "GNSSSDC.h"


///< Store GNSS ephemeris and calculate satellite position
typedef struct tagCEphemerisComputer
{
    // nav info
    bool    m_bNavEph[NSYS];
    GPSEPH  m_GPSEphDatas[NSATGPS][MAXEPHNUM];
    GLOEPH  m_GLOEphDatas[NSATGLO][MAXEPHNUM];
    GPSEPH  m_BD2EphDatas[NSATBD2][MAXEPHNUM];
    GPSEPH  m_BD3EphDatas[NSATBD3][MAXEPHNUM];
    GPSEPH  m_GALEphDatas[NSATGAL][MAXEPHNUM];
    GPSEPH  m_QZSEphDatas[NSATQZS][MAXEPHNUM];

    // other info
    double  m_GPSION[8];/* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    GPSTIME m_gt;                       ///< 卫星信号发射时刻,经过传播距离修正,未经卫星钟差修正
    int     m_prn;                      ///< 卫星prn号
    bool    m_bSimulateRTDS;            ///< 模拟实时数据流
    GPSEPH  m_GPSEph;
    GLOEPH  m_GLOEph;
    GPSEPH  m_BD2Eph;
    GPSEPH  m_BD3Eph;
    GPSEPH  m_GALEph;
    GPSEPH  m_QZSEph;

    // res info
    double  m_SatPos[3];                ///< ECEF, 卫星质心位置
    double  m_SatVel[3];                ///< ECEF, 卫星质心速度
    double  m_SatClk;                   ///< 卫星钟差
    double  m_SatClkVel;                ///< 卫星钟速
    double  m_SatVar;                   ///< 卫星综合误差
    int     m_svh;                      ///< svh为0表明卫星健康 >0表示有问题，=-1表明没有星历计算
} CEphemerisComputer;

extern void InitCEphemerisComputer(CEphemerisComputer* pStruct);

extern bool EphComputer(CEphemerisComputer* pStruct, const GPSTIME gt, const int prn, const EPH_TYPE ephType);


#endif
