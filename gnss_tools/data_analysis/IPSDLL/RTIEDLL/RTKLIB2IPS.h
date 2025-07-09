/**
* @file
* @brief    RTKLIB to IPS format
* @details  RTKLIB��ʽ����ת��ΪIPS��ʽ���� \n
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
* @brief       rtklib gtime_tת����ips gt
* @param[in]   no          src     rtklib gtime_t
* @param[out]  no          n       ips    gt
* @note
*/
extern void ConvertTime(const gtime_t src, GPSTIME* dst);


/**
* @brief       rtklib nav_tת����ipseph��ipsgeph
* @param[in]   no          src     rtklib  nav����
* @param[out]  no          n       ipseph  eph����
* @param[out]  no          dst     ipsgeph geph����
* @note
*/
extern void ConvertNav(const nav_t *src, GPSEPH *dst, GLOEPH *dst_GLO, int nEph[]);


/**
* @brief       rtklib obsd_tת����rt_ipsobs
* @param[in]   no          src     rtklib obs����
* @param[in]   no          n       rtklib obs������
* @param[out]  no          dst     rt_ips obs����
* @return      int         rt_ipsobs������
* @note
*/
extern void ConvertOBS(const obsd_t* src, int n, OBS_DATA* dst, int* nOBS);


#endif // !RTKLIB2IPS_H
