/**
* @file
* @brief    Interface for Basic Common Functions
* @details  基础公共的函数 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_BASECMNFUNC_H
#define BASETK_BASECMNFUNC_H


/**
* @brief       ECEF转向ENU系的旋转矩阵
* @param[in]   LLH       double     纬度,经度,大地高
* @param[out]  R_E2L     double     ECEF->ENU的旋转矩阵
* @return
* @note
*/
extern void Rxyz2enu(const double LLH[3], double R_E2L[9]);


/**
* @brief       ENU转向ECEF系的旋转矩阵
* @param[in]   LLH       double     纬度,经度,大地高
* @param[out]  R_L2E     double     ENU->ECEF的旋转矩阵
* @return
* @note
*/
extern void Renu2xyz(const double LLH[3], double R_L2E[9]);


/**
* @brief       ECEF的向量转到ENU系下
* @param[in]   LLH       double     纬度,经度,大地高
* @param[in]   v_xyz     double     ECEF系下向量
* @param[out]  v_enu     double     ENU系下向量
* @return
* @note
*/
extern void Vxyz2enu(const double LLH[3], const double v_xyz[3], double v_enu[3]);


/**
* @brief       WGS84坐标系: XYZ坐标转LLH坐标
* @param[in]   XYZ          double   XYZ(m) 坐标
* @param[in]   CoorSys      int      坐标系统: 0:WGS84, 1:CGCS2000
* @param[out]  LLH          double   Lat[-90,90],Lon[-180,180],Hgt (deg,m)
* @return      void
* @note
*/
extern void XYZ2LLH(const double XYZ[3], double LLH[3], int CoorSys);


/**
* @brief       WGS84坐标系: LLH坐标转XYZ坐标
* @param[in]   LLH          double   Lat[-90,90],Lon[-180,180],Hgt (deg,m)
* @param[in]   CoorSys      int      坐标系统: 0:WGS84, 1:CGCS2000
* @param[out]  XYZ          double   XYZ(m) 坐标
* @return      void
* @note
*/
extern void LLH2XYZ(const double LLH[3], double XYZ[3], int CoorSys);


/**
* @brief       WGS84坐标转换到CGCS2000坐标,坐标对应的是框架历元
* @param[in]   XYZ_WGS84         double  WGS84坐标(m)
* @param[out]  XYZ_CGCS2000      double  CGCS2000坐标(m)
* @return      void
* @note        WGS84对准到ITRF2000框架,CGCS2000对准到ITRF1997框架,无站速坐标框架转换
*/
extern void WGS84_CGCS2000(const double XYZ_WGS84[3], double XYZ_CGCS2000[3]);


/**
* @brief       CGCS2000坐标转换到WGS84坐标,坐标对应的是框架历元
* @param[in]   XYZ_CGCS2000      double  CGCS2000坐标(m)
* @param[out]  XYZ_WGS84         double  WGS84坐标(m)
* @return      void
* @note        WGS84对准到ITRF2000框架,CGCS2000对准到ITRF1997框架,无站速坐标框架转换
*/
extern void CGCS2000_WGS84(const double XYZ_CGCS2000[3], double XYZ_WGS84[3]);


/**
* @brief       读取复制字符串
* @param[in]   src          char     源字符串,如果末位有'\n',自动除去
* @param[in]   nPos         int      读取的初始位置(从0开始)
* @param[in]   nCount       int      读取的字符串数
* @param[out]  dst          char     目标字符串, 末尾已用\0填充
* @return      void
* @note
*/
extern void xstrmid(const char* src, const int nPos, const int nCount, char* dst);


/**
* @brief       字符串转换成数字
* @param[in]   src        char     源字符串
* @param[in]   nPos       int      读取的初始位置(从0开始)
* @param[in]   nCount     int      读取的字符串数
* @return      double     转换得到的数字
* @note
*/
extern double str2num(const char* src, int nPos, int nCount);


#endif
