/**
* @file
* @brief    Struct, Define and Const (SDC)
* @details  This file contains Struct, Define and Const (SDC) \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_BASESDC_H
#define BASETK_BASESDC_H

#include <math.h>
#include <string.h>
#include <stdio.h>


#ifndef bool
#define bool char
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif


/*============================ start define ======================================*/

#define IPS_EPSILON         (2.2204460492503131e-016)               ///< epsilon == DBL_EPSILON
#define MAXSIZE             (1024)
#define PI                  (3.1415926535897932384626433832795)     ///< pi
#define PI_2                (6.2831853071795864769252867665590)     ///< 2*pi
#define PI_12               (1.5707963267948966192313216916398)     ///< 0.5*pi
#define D2R                 (0.0174532925199432957692369076849)     ///< deg to rad
#define R2D                 (57.295779513082322864647721871734)     ///< rad to deg
#define CLIGHT              (299792458.0)                           ///< speed of light (m/s)
#define AS2R                (D2R/3600.0)                            ///< arc sec to radian

#define NFREQ				3										///< number of carrier frequencies
#define NSYS				6										///< number of constellations, GGCCEJ

#define MAXOBS				60										///< 支持最大卫星数，超过则不读40
#define MAXOBSLIM			120										///< 支持最大观测值数，通过选星算法限制，用于SPP/SPPEKF/SPVEKF/RTK中
#define MAXSPPNX			(3+NSYS*NFREQ)							///< SPP、SPPEKF中支持最大状态数，3个位置参数+若干个钟差参数
#define MAXSPVNX			(6+NSYS)								///< SPV、SPVEKF中支持最大状态数，3个速度参数+3个加速度参数+若干个钟速参数
#define MAXDISNX		    (3+NSYS+MAXOBSLIM)                      ///< TDCP中支持最大状态数，3个速度参数+若干个钟速参数+若干个周跳参数
#define MAXRTKNX			(3+MAXOBSLIM)							///< RTK中支持最大状态数，3个位置参数+若干个模糊度参数

// 因全采用静态数组，需理清关系，LSQ估计器仅用于SPP或SPV，故MAXLSQNX=max(MAXSPPNX,MAXSPVNX)，MAXGLSNX同理
#define MAXLSQNX			MAXDISNX								///< LSQ估计中支持最大状态数，max(MAXSPPNX,MAXSPVNX)
#define MAXGLSNX			MAXRTKNX								///< GLS估计中支持最大状态数，max(MAXSPPNX,MAXSPVNX,MAXRTKNX)

#define IPSSVERSION         "IPSDLLV310T7"                          ///< IPS_SV版本号V3.08


/*============================ end define ========================================*/


/*============================ start const =======================================*/

static const double gs_WGS84_a = 6378137.0;                         ///< earth semimajor axis (WGS84) (m)
static const double gs_WGS84_b = 6356752.31425;                     ///< earth semimajor axis (WGS84) (m)
static const double gs_WGS84_FE = 1.0 / 298.257223563;              ///< earth flattening (WGS84)
static const double gs_WGS84_e2 = 0.00669437999014;
static const double gs_WGS84_e = 0.08181919084262;					///< earth first eccentricity
static const double gs_WGS84_GME = 3.986004418E+14;                 ///< earth gravitational constant (m^3/s^2) fM
static const double gs_WGS84_J2 = 1.08263E-03;                      ///<
static const double gs_WGS84_J4 = -2.37091222E-6;                   ///<
static const double gs_WGS84_J6 = 6.08347E-9;                       ///<
static const double gs_WGS84_OMGE = 7.2921151467E-5;                ///< earth angular velocity (IS-GPS) (rad/s)
static const double gs_WGS84_Ge = 9.7803267714;                     ///< gravity at equator (m/s^2)
static const double gs_WGS84_Gp = 9.8322011865;                     ///< gravity at polar   (m/s^2)
static const double gs_WGS84_Gm = 9.7976446561;                     ///< Mean value (normal) gravity (m/s^2)
static const double gs_WGS84_Galph = 0.00193336138707;              ///< gravity formula constant

static const double gs_CGCS2000_a = 6378137.0;                      ///< earth semimajor axis (WGS84) (m)
static const double gs_CGCS2000_b = 6356752.314140356;              ///< earth semimajor axis (WGS84) (m)
static const double gs_CGCS2000_FE = 1.0 / 298.257222101;           ///< earth flattening (WGS84)
static const double gs_CGCS2000_e2 = 0.00669438002290;
static const double gs_CGCS2000_e = 0.08181919104282;				///< earth first eccentricity
static const double gs_CGCS2000_GME = 3.986004418E+14;              ///< earth gravitational constant (m^3/s^2) fM
static const double gs_CGCS2000_OMGE = 7.2921150E-5;                ///< earth angular velocity (IS-GPS) (rad/s)

static const double gs_min = D2R / 60.0;                            ///< 一个单位角分 对应的 弧度
static const double gs_sec = D2R / 3600.0;                          ///< 一个单位角秒 对应的 弧度
static const double gs_dps = D2R;                                   ///< 1度/秒 对应的 弧度/秒
static const double gs_dph = D2R / 3600.0;                          ///< 1度/小时 对应的 弧度/秒
static const double gs_dpss = D2R;                                  ///< 1度/xsqrt(秒) 对应的 弧度/xsqrt(秒)
static const double gs_dpsh = D2R / 60.0;                           ///< 1度/xsqrt(小时) 对应的 弧度/xsqrt(秒), 角度随机游走系数
static const double gs_ppm = 1e-6;                                  ///< part per million(百万分之一)

/*============================ end const =========================================*/


#endif
