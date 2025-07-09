/**
* @file
* @brief    Troposphere
* @details  对流层 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CTROPOSPHERE_H
#define GNSSTK_CTROPOSPHERE_H

#include "GNSSSDC.h"


///< Troposphere
typedef struct tagCTroposphere
{
    // Attribute
    double m_STD;
    double m_ZHD;
    double m_ZWD;

    double m_ZHDMF;
    double m_ZWDMF;
    double m_GRDMF;

    double m_TropVar;

    // Members
    TROP_TYPE m_ZTDType;
    TROP_TYPE m_MFType;
    TROP_TYPE m_MFGType;

    GPSTIME m_utc;

    double m_LLH[3];

    double m_elev;

    double m_T;   // Temperature(K)
    double m_P;   // Pressure(mbar)
    double m_e;   // water vapor pressure(mbar)

    // hydrostatic mapping function and derivative
    // wet mapping function and derivative
    // 其中导数为了计算Bern梯度模型
    double m_mfh[2];
    double m_mfw[2];

    double m_mfg; // grad map function 湿分量梯度投影函数

    double m_Pnm[10][10]; // 勒让德球谐系数,9度9阶

} CTroposphere;

extern void InitCTroposphere(CTroposphere* pStruct);

//bool TropComputer(const GPSTIME gt, const double LLH[3], const double elev, bool bSPP, TROP_TYPE ZTDType = TROP_SS, TROP_TYPE MFType = TROP_NMF, TROP_TYPE MFGType = TROP_NONE);
extern bool TropComputer(CTroposphere* pStruct, const GPSTIME gt, const double LLH[3], const double elev, bool bSPP, TROP_TYPE ZTDType, TROP_TYPE MFType, TROP_TYPE MFGType);

#endif
