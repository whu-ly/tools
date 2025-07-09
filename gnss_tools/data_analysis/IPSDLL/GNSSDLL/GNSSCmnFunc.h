/**
* @file
* @brief    Interface for GNSS Common Functions
* @details  GNSS相关的公共函数 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_GNSSCMNFUNC_H
#define GNSSTK_GNSSCMNFUNC_H

#include "GNSSSDC.h"

/**
 * @brief       GNSS卫星号转程序prn号
 * @param[in]   no          char     卫星号(Gno,Rno,Cno,Eno,Jno)
 * @return      int         程序自定义卫星序号, 1,2 int来区分各卫星号(1,2,....), 0:error
 * @note
 */
extern int satno2prn1(const char* no);


/**
* @brief       GNSS卫星号转程序prn号
* @param[in]   no          int      系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   no          int      每个系统内的卫星号(1,2,3,4.....)
* @return      int         程序自定义卫星序号, 1,2 int来区分各卫星号(1,2,....), 0:error
* @note                    输入BD3卫星号的从1开始
*/
extern int satno2prn2(const int sys, const int no);


/**
* @brief       程序prn号转GNSS卫星号
* @param[in]   prn         int      程序自定义卫星序号, 由一个int来区分各卫星号
* @param[out]  sys         int      系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @return      int         每个系统内的卫星号(1,2,3,4.....)
* @note                    返回的BD3卫星号的从1开始
*/
extern int satprn2no(const int prn, int* sys);


/**
* @brief       程序prn号转GNSS卫星字符号
* @param[in]   prn         int      程序自定义卫星序号, 由一个int来区分各卫星号
* @return      string      Gnn,Rnn,Cnn,Enn,Jnn, error: Nnn
* @note                    返回的BD3卫星号的从18开始
*/
extern void satprn2nos(const int prn, char* str);


/**
* @brief       NSATMAX循环时,对于单系统加快速度
* @param[in]   SatSYS      int      m_pGNSSOpt->m_SatSYS
* @param[in]   sys         int      当前prn的卫星系统
* @param[out]  prn         int      当前prn卫星
* @return      int         0:OK, 1:continue, 2:break
* @note
*/
extern int SkipPrn(const int SatSYS, const int sys, int* prn);


/**
* @brief       根据prn号确定BDS卫星的类型
* @param[in]   prn         int      prn号
* @return      int         1:GEO,2:IGSO,3:MEO,0:error
* @note        BD2和BD3都可以用
*/
extern int BDSSatelliteType(int prn);


/**
* @brief       根据系统类型返回索引号
* @param[in]   sys         int      卫星系统类型,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @return      int         系统索引号,[0,NSYS-1]
* @note
*/
extern int Sys2Index(int sys);


/**
* @brief       根据索引号返回系统类型
* @param[in]   index       int      系统索引号,[0,NSYS-1]
* @return      int         卫星系统类型,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @note
*/
extern int Index2Sys(int index);


/**
* @brief       计算卫地距(包含sagnac效应改正)
* @param[in]   rs          double   ECEF系下的卫星坐标(m)
* @param[in]   rr          double   ECEF系下的测站坐标(m)
* @param[out]  e           double   卫地单位向量, 测站指向卫星
* @return      double      卫地距(包含sagnac效应改正), -1.0:error, 没有卫星位置
* @note
*/
extern double geodist(const double rs[3], const double rr[3], double e[3]);


/**
* @brief       计算卫星方位角和高度角
* @param[in]   LLH         double   测站经纬度
* @param[in]   e           double   ECEF系下, 测站指向卫星的单位向量
* @param[out]  azel        double   azimuth/elevation {az,el} (rad)
* @return      double      elevation angle (rad)
* @note        (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
*/
extern double satazel(const double LLH[3], const double e[3], double* azel);


/**
* @brief       初始化Sat的基本信息,主要是prn,sys,sys_id
* @param[in]   pSatStatus  SATSTATUS   卫星状态结构体
* @param[in]   GLO_iFreq   int         GLO频率数,NULL的话用内部的
* @return
* @note
*/
//extern void initSatStatus(SATSTATUS* pSatStatus, int* GLO_iFreq = NULL);
extern bool initSatStatus(SATSTATUS* pSatStatus, int* GLO_iFreq);


/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   sys         int       系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   GLO_iFreq   int       GLO卫星的频数
* @param[out]  frq         double    frq[3]三个频点上的频率
* @return      bool        true or false
* @note
*/
//extern bool getGNSSFrequency(const int sys, double frq[NFREQ], const int GLO_iFreq = 0);
extern bool getGNSSFrequency(const int sys, double frq[NFREQ], const int GLO_iFreq);


/**
* @brief       计算卫星载波频率或波长
* @param[in]   sys         int      卫星系统, SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   frq         int      频率 (0:L1,B1,E1; 1:L2,B2,E5a; 2:L5,B3,E5b; 3:E5a+b)
* @param[in]   GLO_iFreq   int      GLO卫星的频率号, 从导航文件中读取
* @param[in]   bfrq        bool     true: 给出频率(HZ), false: 给出波长(m)
* @return      double      载波频率你或波长(HZ,m) (0.0: error)
* @note
*/
//extern double getGNSSWavelen(const int sys, const int frq, const int GLO_iFreq, const bool bfrq = false);
extern double getGNSSWavelen(const int sys, const int frq, const int GLO_iFreq, const bool bfrq);


/**
* @brief       计算DOP值(dilution of precision)
* @param[in]   Azim        double    Azim[NSATMAX+1], 卫星方位角(rad)
* @param[in]   Elev        double    Elev[NSATMAX+1], 卫星高度角(rad)
* @param[in]   ElevMask    double    截止高度角(rad)
* @param[in]   pSatStatus  SATSTATUS 卫星状态,PPP/RTK中计算参与解算卫星的DOPs值用,默认为NULL
* @param[out]  DOPs        double    DOP值 {GDOP,PDOP,HDOP,VDOP}
* @return      bool        true or false
* @note
*/
//extern bool ComputeDOP(const double* Azim, const double* Elev, double ElevMask, double DOPs[4], const SATSTATUS* pSatStatus = NULL, bool bIAR = false, bool bIARPAR = false);
extern bool ComputeDOP(const double* Azim, const double* Elev, double ElevMask, double DOPs[4], const SATSTATUS* pSatStatus, bool bIAR, bool bIARPAR);

/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @return      double      三频组合观测值的波长(m)
* @note
*/
extern double TFLC_lam(const double frq[NFREQ], const int i, const int j, const int k);


/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   L           double    三频L观测值(cycle)
* @return      double      三频L观测值的组合(m), 0:false
* @note
*/
extern double TFLC_ZD_L(const double frq[NFREQ], const int i, const int j, const int k, const double L[3]);


/**
* @brief       获取单差三频L观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   Li          double    三频L观测值(cycle)
* @param[in]   Lj          double    基准的三频L观测值(cycle)
* @return      double      单差三频L观测值的组合(m), 0:false
* @note
*/
extern double TFLC_SD_L(const double frq[NFREQ], const int i, const int j, const int k, const double Li[3], const double Lj[3]);


/**
* @brief       获取非差三频P观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   P           double    三频P观测值(m)
* @return      double      三频P观测值的组合(m), 0:false
* @note
*/
extern double TFLC_ZD_P(const double frq[NFREQ], const int i, const int j, const int k, const double P[3]);


/**
* @brief       获取单差三频P观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   Pi          double    三频P观测值(m)
* @param[in]   Pj          double    基准的三频P观测值(m)
* @return      double      单差三频P观测值的组合(m), 0:false
* @note
*/
extern double TFLC_SD_P(const double frq[NFREQ], const int i, const int j, const int k, const double Pi[3], const double Pj[3]);


/**
* @brief       获取伪距三频无电离层组合IFLC
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    伪距观测值(m)
* @return      double      IFLC组合值(m),0:计算不成功
* @note
*/
extern double TFLC_IF_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ]);


/**
* @brief       获取相位三频无电离层组合IFLC
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   L           double    相位观测值(cycle)
* @param[in]   bLlam       bool      输入的相位单位是否为m, true:m, false:cycle
* @return      double      IFLC组合值(m),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern double TFLC_IF_L(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ], bool bLlam);


/**
* @brief       获取相位三频GF组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   L           double    相位观测值(cycle)
* @return      double      GF组合值(m),0:计算不成功
* @note
*/
extern double TFLC_GF_L(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ]);


/**
* @brief       获取相位三频伪距宽巷组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    相位观测值(m)
* @return      double      WL组合值(m),0:计算不成功
* @note
*/
extern double TFLC_WL_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ]);


/**
* @brief       获取相位三频伪距窄巷组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    相位观测值(m)
* @return      double      NL组合值(m),0:计算不成功
* @note
*/
extern double TFLC_NL_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ]);


/**
* @brief       获取三频MW组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   L           double    相位观测值(cycle)
* @param[in]   P           double    伪距观测值(m)
* @return      double      MW组合值(cycle),0:计算不成功
* @note
*/
extern double TFLC_MW_LP(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ], const double P[NFREQ]);


/**
* @brief       得到双频组合的索引号
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      int         IF索引号
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
*/
extern int TFLC_IFIndex1(const TFLCTYPE iFrq[2]);


/**
* @brief       得到双频组合的索引号
* @param[in]   id          int       IF索引号
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      bool        true or false
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
*/
extern bool TFLC_IFIndex2(const int id, TFLCTYPE iFrq[2]);


/**
* @brief       由某个频点得到与其联系的双频组合的索引号
* @param[in]   iFrq        哪个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   id          IF索引号
* @return      bool        true or false
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
*/
extern bool TFLC_IFIndex3(const TFLCTYPE iFrq, int id[2]);


/**
* @brief       选择IF组合的两个频率
* @param[in]   bFrq        bool      哪些频率可用
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      bool        true or false
* @note
*
*/
extern bool TFLC_SelectIFIndex(const bool bFrq[NFREQ], TFLCTYPE iFrq[2]);


/**
* @brief       计算三频伪距多路径值
* @param[in]   P[3]         double   三频伪距观测值(m)
* @param[in]   L[3]         double   三频相位观测值(cycle)
* @param[in]   lam[3]       double   三频相位观测值波长(m)
* @param[in]   CS_Type[3]   char     三频相位观测值周跳信息
* @param[in]   nMP[3]       int      MP累积个数
* @param[in]   MPMean[3]    double   MP均值,包含模糊度
* @param[out]  MP[3]        double   当前MP值,去掉模糊度
* @return      bool
* @note        前后多路径差超过2.0m或者发生周跳,重新初始化
*/
extern bool ComputeMultipath(const double P[3], const double L[3], const double lam[3], const char CS_Type[3], int nMP[3], double MPMean[3], double MP[3]);


/**
* @brief       双天线基线向量得到航向角和俯仰角
* @param[in]   BL           double   双天线基线向量
* @param[in]   Azimuth      double   北向起算的航向角[0,2*PI]和俯仰角[-PI/2,PI/2]
* @return
* @note
*/
//extern bool BL2Azimuth(const double BL[3], double Azimuth[3], const double* XYZ = NULL, const double* XYZP = NULL, double* AttP = NULL);
extern bool BL2Azimuth(const double BL[3], double Azimuth[3], const double* XYZ, const double* XYZP, double* AttP);


/**
* @brief       计算数据序列datas最佳的Std值
* @param[in]   datas      double   数据序列,<prn,val>
* @param[in]   norm_datas double   标准化以后的数据序列,norm_datas = datas/Std, Std为datas计算的Std
* @param[in]   nbad       int      假定的粗差个数
* @param[in]   Std_Thres  double   Std阈值,计算的Std小于此阈值就计算完成
* @param[out]  Mean       double   计算的最佳均值(除去粗差)
* @param[out]  Std        double   计算的最佳标准差(除去粗差)
* return       bool       true:计算成功,false:计算失败
* @note
*/
//extern bool getResiBestStd(map<int, double>& datas, map<double, int>& norm_datas, const int nbad, const double Std_Thres, double& Mean, double& Std);


/**
* @brief       计算数据序列datas最佳的Std值
* @param[in]   datas      double   数据序列,<prn,val>
* @param[in]   nbad_max   int      假定的粗差最大个数
* @param[in]   Val_Thres  double   判定为粗差的最小赋值(以均值为基准)
* @param[in]   Std_Thres  double   判定为粗差的Std
* @param[in]   Ratio_Thres double  判定为粗差时Std的放大因子
* @param[out]  MeanMin    double   计算的最佳均值(除去粗差)
* @param[out]  StdMin     double   计算的最佳标准差(除去粗差)
* @param[out]  iBadSat    int      判定为粗差的卫星号
* return       int        粗差个数(0:表示没有粗差)
* @note
*/
//extern int FindOutliers(map<int, double>& datas, const int nbad_max, const double Val_Thres, const double Std_Thres, const double Ratio_Thres, double& MeanMin, double& StdMin, map<int, int>& iBadSat);


/**
* @brief       初始化GNSS状态的索引
* @param[in]   pGNSSOpt    CGNSSOption   GNSS配置
* @param[in]   EstTrp      int           0:不估计,1:只估计流动站,2:估计流动站+基准站
* @param[in]   EstIon      int           0:不估计(IFLC),1:估计
* @param[in]   bReverse    bool          是否逆向滤波
* @param[out]  pGLSInfo    GNSS_GLSINFO  GNSS解算信息
* return       void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void InitGNSS_StateIndex(GNSS_GLSINFO* pGLSInfo, int EstTrp, int EstIon, bool bReverse);


/**
* @brief       GNSS方程部分信息清零
* @param[in]   n           int      卫星数
* @param[in]   pGNSSMEQ    GNSS_MEQ GNSS_MEQ
* return
* @note
*/
extern void zeroGNSSMEQ(const int n, GNSS_MEQ* pGNSSMEQ);


/**
* @brief       调整时间,和参考时间差不超过半周
* @param[in]   t            GPSTIME  待调整时间
* @param[in]   t0           GPSTIME  参考时间
* @return      GPSTIME      调整后的时间
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern GPSTIME adjweek(GPSTIME t, GPSTIME t0);


/**
* @brief       URA值转换为URA指数
* @param[in]   value       double   URA值
* @return      int         URA指数(0-14)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern int URA2Index(const double value);


/**
* @brief       URA指数转换为URA值
* @param[in]   index       int       URA指数(0-14)
* @return      double      URA值
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern double Index2URA(const int index);


/**
* @brief       获取动态阈值
* @param[in]   src[]		待确定动态阈值的源数组
* @param[in]   num			源数据的个数
* @param[in]   proportion	保留比例
* @param[in]   minThres		最小阈值
* @param[in]   maxThres		最大阈值
* @param[in]   bMedian		是否以中位数为基准
* @return      double		动态阈值
* @note
* @internals
*/
extern double GetDynamicThres(double* src, int nsrc, double proportion, double minThres, double maxThres, bool bMedian);


extern bool CheckDogPermission(GPSTIME *dataTime);

#endif
