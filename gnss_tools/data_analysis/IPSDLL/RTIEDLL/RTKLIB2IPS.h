/**
* @file
* @brief    RTKLIB to IPS format
* @details  RTKLIB格式数据转换为IPS格式数据 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.2
* @par      Copyright(c) 2012-2021 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2021/05/12, Xianlu Tao, new \n
*/


#ifndef RTIETK_RTKLIB2IPS_H
#define RTIETK_RTKLIB2IPS_H

#include "rtklib.h"
#include "GNSSSDC.h"

/**
* @brief       rtklib gtime_t转程序ips gt
* @param[in]   no          src     rtklib gtime_t
* @param[out]  no          n       ips    gt
* @note
*/
extern void ConvertTime(const gtime_t src, GPSTIME* dst);


/**
* @brief       rtklib nav_t转程序ipseph和ipsgeph
* @param[in]   no          src     rtklib  nav数组
* @param[out]  no          n       ipseph  eph数组
* @param[out]  no          dst     ipsgeph geph数组
* @note
*/
extern void ConvertNav(const nav_t *src, GPSEPH *dst, GLOEPH *dst_GLO, int nEph[]);


/**
* @brief       rtklib obsd_t转程序rt_ipsobs
* @param[in]   no          src     rtklib obs数组
* @param[in]   no          n       rtklib obs卫星数
* @param[out]  no          dst     rt_ips obs数组
* @return      int         rt_ipsobs卫星数
* @note
*/
extern void ConvertOBS(const obsd_t* src, int n, OBS_DATA* dst, int* nOBS);


#endif // !RTKLIB2IPS_H
