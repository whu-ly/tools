/**
* @file
* @brief    Interface for Basic Time operation
* @details  Date/Time algorithms \n
*           The GPS Toolbox: The Remondi Date/Time Algorithms, GPS Solutions
*           sourcecode from: http://www.ngs.noaa.gov/gps-toolbox/bwr-c.txt
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_BASETIME_H
#define BASETK_BASETIME_H

#include "BaseSDC.h"


/*============================ start struct ======================================*/

typedef struct tagGPSTIME   ///< GPS时,不能改动,否则和IPSDLL中定义不一致,导致错误
{
    int         GPSWeek;
    int         secsOfWeek;
    double      fracOfSec;

} GPSTIME;
extern void InitGPSTIME(GPSTIME* ot);
extern void InitGPSTIME2(int week, double sec, GPSTIME* ot);
extern double MinusGPSTIME(GPSTIME gt1, GPSTIME gt2);
extern GPSTIME MinusGPSTIMESec(GPSTIME gt1, double sec);
extern GPSTIME AddGPSTIMESec(GPSTIME gt1, double sec);
extern bool IsGPSTIMEEqual(GPSTIME gt1, GPSTIME gt2);
extern double GetGPSTIMESow(GPSTIME gt);

typedef struct tagMJD
{
    long        mjd;
    double      fracOfDay;

}  MJD;

typedef struct tagYDOYHMS
{
    int         year;
    int         yday; ///< dayOfYear
    int         hour;
    int         min;
    double      sec;

}  YDOYHMS;

typedef struct tagYMDHMS
{
    int         year;
    int         month;
    int         day;
    int         hour;
    int         min;
    double      sec;

}  YMDHMS;


/*============================= end struct =======================================*/
	
/*
* @brief       GPS时间转换为MJD
* @param[in]   t           GPSTIME  GPS时
* @return      MJD         约化儒略日
* @note
*/
extern MJD GPST2MJD(GPSTIME t);


/*
* @brief       GPS时间转换为YDOYHMS
* @param[in]   t           GPSTIME  GPS时
* @return      YDOYHMS     年/年积日/HMS
* @note
*/
extern YDOYHMS GPST2YDHMS(GPSTIME t);


/*
* @brief       GPS时间转换为YMDHMS
* @param[in]   t           GPSTIME  GPS时
* @return      YMDHMS      年/月/日/HMS
* @note
*/
extern YMDHMS GPST2YMDHMS(GPSTIME t);


/*
* @brief       MJD转换为GPS时间
* @param[in]   t           MJD      约化儒略日
* @return      GPSTIME     GPS时
* @note
*/
extern GPSTIME MJD2GPST(MJD t);


/*
* @brief       YDOYHMS转换为GPS时间
* @param[in]   t           YDOYHMS  年/年积日/HMS
* @return      GPSTIME     GPS时
* @note
*/
extern GPSTIME YDHMS2GPST(YDOYHMS t);


/*
* @brief       YMDHMS转换为GPS时间
* @param[in]   t           YMDHMS  年/月/日/HMS
* @return      GPSTIME     GPS时
* @note
*/
extern GPSTIME YMDHMS2GPST(YMDHMS t);


/*
* @brief       utc 时间转换到GPS时
* @param[in]   utc         GPSTIME  utc时间
* @return      GPSTIME     GPS时间
* @note        ignore slight time offset under 100 ns
*/
extern GPSTIME UTC2GPST(GPSTIME utc);


/*
* @brief       GPS 时间转换到UTC时
* @param[in]   gt          GPSTIME  GPS时间
* @return      GPSTIME     utc时间
* @note        ignore slight time offset under 100 ns
*/
extern GPSTIME GPST2UTC(GPSTIME gt);


/*
* @brief       GPS时转BDS时
* @param[in]   gt          GPSTIME  GPS时间
* @param[in]   bSecCorr    bool     true:只进行14s改正, false:BDS周改正
* @return      GPSTIME     BDS时间
* @note
*/
extern GPSTIME GPST2BDST(GPSTIME gpst, bool bSecCorr);


/*
* @brief       BDS时转GPS时
* @param[in]   gt          GPSTIME  BDS时间
* @param[in]   bSecCorr    bool     true:只进行14s改正, false:BDS周改正
* @return      GPSTIME     GPS时间
* @note
*/
extern GPSTIME BDST2GPST(GPSTIME bdst, bool bSecCorr);


/*
* @brief       string 转时间
* @param[in]   s           char     字符串 string ("... yyyy mm dd hh mm ss ...")
* @param[in]   iPos        int      起始位置
* @param[in]   nCount      int      长度
* @return      GPSTIME     时间
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern GPSTIME str2time(const char* s, int iPos, int nCount);

#endif
