/**
* @file
* @brief    Ionosphere
* @details  电离层 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CIONOSPHERE_H
#define GNSSTK_CIONOSPHERE_H

#include "GNSSSDC.h"


///< Ionosphere
typedef struct tagCIonosphere
{
    // Attribute
    double m_STD;           // (m)
    double m_ZTD;
    double m_MF;
    double m_IonoVar;

    // Members
    GPSTIME m_gt;
    double  m_LLH[3];
    double* m_KloCoeff;     // ALPHA, BETA {a0,a1,a2,a3,b0,b1,b2,b3}
    double* m_azel;		    // azimuth/elevation angle {az,el} (rad)
    double  m_lam;
    double  m_GPSL1Lam;
} CIonosphere;

extern void InitCIonosphere(CIonosphere* pStruct);

//extern bool IonoComputer(GPSTIME gt, const double LLH[3], double* azel, double lam, IONO_TYPE ionoType = IONO_KLOB, double* KloCoeff = NULL);
extern bool IonoComputer(CIonosphere* pStruct, GPSTIME gt, const double LLH[3], double* azel, double lam, IONO_TYPE ionoType, double* KloCoeff);

#endif
