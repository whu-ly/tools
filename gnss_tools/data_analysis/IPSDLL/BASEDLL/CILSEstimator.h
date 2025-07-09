/**
* @file
* @brief    Integer Ambiguity Resolution, LAMBDA method
* @details  整数模糊度固定 \n
*           Note that all memory allocation is static, so it is important to pay attention to the memory dimension of MAXOBSLIM
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_ILSESTIMATOR_H
#define BASETK_ILSESTIMATOR_H

#include "BaseMatrix.h"


#define NCANDS  500


///< 整数最小二乘估计器
typedef struct tagCILSEstimator
{
    // 1: ILS method based on search-and-shrink [DEFAULT]
    // 2: ILS method based enumeration in search
    // 3: integer rounding method
    // 4: integer bootstrapping method
    // 5: PAR with the input P0 of user-defined success rate
    // 6: ILS method with Ratio Test (uses search-and shrink)
    // 7: Clambda
    // 8: ENUlambda
    int    m_method;

    double m_Float[MAXOBSLIM];              ///< Float ambiguities (must be a column!), m_nfixed * 1
    double m_QFloat[MAXOBSLIM * MAXOBSLIM]; ///< Variance/covariance matrix of float ambiguities, m_nfixed * m_nfixed
    double m_ARThres;                       ///< 外部指定的模糊度固定的ratio阈值
    double m_FTThresBound[2];               ///< FTThres的上下限
    double m_FTThres;                       ///< 内部计算得到的基于失败率的ratio阈值
    double m_ADOPThres;                     ///< ADOP判断阈值, 通过外部设置
    double m_SRBSThres;                     ///< SRBS判断阈值, 通过外部设置
    double m_ARRatio;                       ///< 计算得到的ratio值

    // output
    double m_afixed[MAXOBSLIM];             ///< Array of size (n) with the estimated integer, m_nfixed
                                            ///< candidates, sorted according to the corresponding squared norms, best candidate first.
                                            ///< For integer rounding and bootstrapping: ncands = 1

    // ADOP && Success Rate
    double m_ADOP;
    double m_SR_ADOP;
    double m_SR_Bootstrap;
    double m_ADOPFull;
    double m_SR_ADOPFull;
    double m_SR_BootstrapFull;

    // inner variables for lambda
    int     m_nfixed;                       ///< Number of fixed solutions
    double  m_sqnorm[1 * NCANDS];           ///< Distance between integer candidate and float parameter vectors, 1 * m_ncands
    double  m_zfixed[MAXOBSLIM * NCANDS];   ///< estimated integers in Z-transformation space, m_nfixed * m_ncands
    double  m_L[MAXOBSLIM * MAXOBSLIM];     ///< L matrix (from LtDL-decomposition of Qzhat), m_nfixed * m_nfixed
    double  m_D[MAXOBSLIM];                 ///< D matrix (from LtDL-decomposition of Qzhat), m_nfixed * 1
    double  m_zhat[MAXOBSLIM];              ///< Transformed ambiguities (optional), m_nfixed * 1
    double  m_Qzhat[MAXOBSLIM * MAXOBSLIM]; ///< Variance-covariance matrix of decorrelated ambiguities, m_nfixed * m_nfixed
    double  m_Z[MAXOBSLIM * MAXOBSLIM];     ///< Z-transformation matrix, m_nfixed * m_nfixed
    double  m_iZt[MAXOBSLIM * MAXOBSLIM];   ///< inv(Z')-transformation matrix, m_nfixed * m_nfixed
    double  m_incr[MAXOBSLIM];              ///< remove integer numbers from float solution, m_nfixed * 1
    int     m_ncands;                       ///< Number of requested candidates
    double  m_P0;                           ///< with method 5 (PAR) : Minimum required success rate, [DEFAULT = 0.995]
                                            ///< with method 6 (ILS + Ratio test) : Fixed failure rate, (available options : 0.01 or 0.001)[DEFAULT = 0.001]
    double  m_Ps;                           ///< Bootstrapped success rate
    int     m_FFRT;                         ///< 1:ILS method with Ratio Test, 0:not
    double  m_mu;                           ///< Threshold value used for Ratio Test

    //int     m_BIEncands;                  ///< number of candidates to be involved
    int     m_BIEWeightMethod;              ///< 0 = GBIE，1 = LBIE，2 = TBIE
    //bool    m_bBIESearchCons;             ///< BIE中搜索模糊度时是否进行搜索起始点约束
    double  m_BIEOptZFixed[MAXOBSLIM];      ///< 约束值，一般从ILS中获取
} CILSEstimator;

extern void InitCILSEstimator(CILSEstimator* pStruct);
extern bool InitLambda(CILSEstimator* pStruct, int n);
extern double SR_Bootstrap(const double D[], int nfixed);
extern double SR_Rounding(double flo, double sigma);
extern double ComputeADOP(const double D[], int nfixed);
extern double SR_ADOP(const double D[], int nfixed);
extern bool LAMBDA(CILSEstimator* pStruct, bool bCons); ///<lambda integer least-square estimation
extern bool ssearch(const double ahat[], const double L[], const double D[], int nfixed, int* ncands, double afixed[], double sqnorm[], double* conDist, double conZcond[], int* gpNum);

#endif
