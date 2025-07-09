/**
* @file
* @brief    GNSS Integer Ambiguity Resolution
* @details  GNSS�����ļ� \n
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
* @brief       ��ʱ��(Timer)
* @return      double      �볤(ms)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void StartTimer();
extern double EndTimer();


/**
* @brief		����Ĭ�����ñȽ���ֵ�ļ�(HW_TRJ��ʽ)�ͽ���ļ�(NMEA��ʽ)
* @param[in]	refPoss		double		��ֵ���꣨��̬�����踳ֵ��
* @param[in]	truFile		char[]		��ֵ�ļ�������̬�����踳ֵ��
* @param[in]	errFile		char[]		����ļ���
* @param[in]	outFile		char[]		����ļ���
* return		void
* @note         refPoss����truFile����Ч����ȡ�̶���ƽ��ֵ��Ϊ��ֵ
* @internals
*/
extern void GetPosErrs(double* refPoss, char truFile[], char errFile[], char outFile[]);


#endif