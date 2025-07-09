/**
* @file
* @brief    GNSS Integer Ambiguity Resolution
* @details  GNSS后处理文件 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_CGNSSAPPLICATION_H
#define GNSSTK_CGNSSAPPLICATION_H


#include "BaseSDC.h"


/*
* @brief       计时器(Timer)
* @return      double      秒长(ms)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void StartTimer();
extern double EndTimer();


/**
* @brief		按照默认配置比较真值文件(HW_TRJ格式)和结果文件(NMEA格式)
* @param[in]	refPoss		double		真值坐标（静态数据需赋值）
* @param[in]	truFile		char[]		真值文件名（动态数据需赋值）
* @param[in]	errFile		char[]		结果文件名
* @param[in]	outFile		char[]		输出文件名
* return		void
* @note         refPoss或者truFile均无效，则取固定解平均值作为真值
* @internals
*/
extern void GetPosErrs(double* refPoss, char truFile[], char errFile[], char outFile[]);


#endif