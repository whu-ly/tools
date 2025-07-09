/**
* @file
* @brief    Package of Basic Math Tools.
* @details  This file contains the most used math tools in the IPS. \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, Wuhan University. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_BASEMATH_H
#define BASETK_BASEMATH_H


#include "BaseSDC.h"


/**
* @brief        搜索 vector 最小值的位置
* @param[in]    vec     double[]    input vector
* @param[in]    n       int         vector 维数
* @param[out]   pos     int*        最小值的位置, 从0开始
* @param[in]    bAbs    bool        是否取绝对值
* @return       double  最小值
* @note         无
*/
extern double getMinPos(const double vec[], int n, int* pos, bool bAbs);


/**
* @brief        搜索 vector 最大值的位置
* @param[in]    vec     double[]    input vector
* @param[in]    n       int         vector 维数
* @param[out]   pos     int*        最大值的位置, 从0开始
* @param[in]    bAbs    bool        是否取绝对值
* @return       double  最大值
* @note         无
*/
extern double getMaxPos(const double vec[], int n, int* pos, bool bAbs);


/**
* @brief       开方运算, 解决输入小于0的时候, sqrt报错
* @param[in]   val		double		输入值
* @return      double	返回开方值, 若输入小于0,返回0值
* @note
*/
extern double xsqrt(const double val);


/**
* @brief       初始化状态向量
* @param[in]   StateX	double[]    状态向量
* @param[in]   StateP   double[]    状态方差阵
* @param[in]   row      int			方阵维数
* @param[in]   i        int			第i个初始化
* @param[in]   x        double		状态值
* @param[in]   var      double		状态方差
* @return      void
* @note        StateX[i] = x，StateP[i][i] = var, i行和i列其它元素置0
*/
extern void InitStateX(double StateX[], double StateP[], int row, int i, double x, double var);


/**
* @brief       计算序列均值的递推公式
* @param[in]   avg		double*     输入i-1时刻的均值,输出i时刻的均值
* @param[in]   i		int         当前为i时刻,i=1开始
* @param[in]   data		double      i时刻新进入的数据
* @return      void
* @note        先调用var, 再调用avg
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void SequenceAverage(double* avg, int i, double data);


/**
* @brief       计算序列均值、方差和均方根误差
* @param[in]   data		double*		数据序列
* @param[in]   n		int			数据序列维数
* @param[out]  mean		double*		序列均值
* @param[out]  rms		double*     序列均方根误差
* @param[out]  var		double*     序列方差,默认为NULL
* @param[in]   bessel	bool        方差计算是否用白塞尔公式，默认为false
* @return      void
* @note
*/
extern void Mean_RMS_Var(const double* data, const int n, double* mean, double* rms, double* var, bool bessel);


/**
* @brief       勒让德球谐函数 Legendre Spherical harmonics function
* @param[in]   n       int			度(degree)
* @param[in]   m       int			阶(order)
* @param[in]   x       double		value  x -> [0,1], x = sin(lat)
* @param[out]  P       double*		P[n+1][m+1], i>=j 有值
* @return      void
* @note        推荐使用 Pnm(9,9,x,P[0]);
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void Pnm(int n, int m, double x, double* P);


/**
* @brief       四舍五入取整函数
* @param[in]   a       double		待取整数字
* @return      int
* @internals
*/
extern int RoundNum(double a);


/**
* @brief       冒泡排序函数，从小到大
* @param[in]   vec      double*		待排序数组
* @param[in]   n		int			数组维数
* @param[in]   bMin2Max bool		true=从小到大
* @return      void
* @internals
*/
extern void SortArr(double* vec, int n, bool bMin2Max);


/**
* @brief       判断两个数字是否相等
* @param[in]   a		double		待比较数字1
* @param[in]   b		double		待比较数字2
* @return      bool
* @internals
*/
extern bool IsEqual(double a, double b);


/**
* @brief       交换两个数字的值
* @param[in]   a		float/int*	待交换数字1
* @param[in]   b		float/int*	待交换数字2
* @return      void
* @internals
*/
extern void SwapF(float *a, float *b);
extern void SwapI(int *a, int *b);


/**
* @brief       计算a的小数部分
* @param[in]   a		double		待处理数字
* @return      double
* @internals
*/
extern double GetFrac(double a);


#endif
