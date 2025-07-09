/**
* @file
* @brief    Struct, Define and Const (SDC)
* @details  GNSS相关的结构体,定义和常量 \n
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/25,Ying Liu, new \n
*/


#ifndef GNSSTK_GNSSSDC_H
#define GNSSTK_GNSSSDC_H

#include "BaseSDC.h"
#include "BaseMatrix.h"
#include "BaseTime.h"

/*============================ start define ======================================*/

#define POSTPROIPS     // When post-processing GNSS data, this needs to be enabled, edit by Lying

#define MAXANT      64                  ///< max length of station name/antenna type

#define ERR_CBIAS   0.3                 ///< code bias error std (m)
#define ERR_ION     5.0                 ///< ionospheric delay std (m)
#define ERR_TROP    3.0                 ///< tropspheric delay std (m)
#define ERR_SAAS    0.3                 ///< saastamoinen model error std (m)
#define ERR_SAAG    0.25                ///< GPS + saastamoinen model error std (m)
#define ERR_UNBM    0.1                 ///< UNB3m model error std (m)
#define ERR_BRDCI   0.5                 ///< broadcast iono model error factor
#define ERR_BRDCI2  0.25                ///< the square of broadcast iono model error factor
#define ERR_CBIAS   0.3                 ///< code bias error std (m)
#define ERREPH_GLO  5.0                 ///< error of glonass ephemeris (m)

//#define NFREQ       2                  ///< number of carrier frequencies

#define FREQ1       1.575420E9          ///< L1/E1  frequency (Hz), 154*10.23MHZ
#define FREQ2       1.227600E9          ///< L2     frequency (Hz), 120*10.23MHZ
#define FREQ5       1.176450E9          ///< L5/E5a frequency (Hz), 115*10.23MHZ
#define FREQ6       1.278750E9          ///< E6/LEX frequency (Hz), 125*10.23MHZ
#define FREQ7       1.207140E9          ///< E5b    frequency (Hz), 118*10.23MHZ
#define FREQ8       1.191795E9          ///< E5a+b  frequency (Hz), 116.5*10.23MHZ

#define FREQ1_GPS   FREQ1               ///< L1 frequency (Hz), 154*10.23MHZ
#define FREQ2_GPS   FREQ2               ///< L2 frequency (Hz), 120*10.23MHZ
#define FREQ3_GPS   FREQ5               ///< L5 frequency (Hz), 115*10.23MHZ

#define FREQ1_GLO   1.602000E9          ///< GLONASS G1 base frequency (Hz)
#define DFRQ1_GLO   0.562500E6          ///< GLONASS G1 bias frequency (Hz/n)
#define FREQ2_GLO   1.246000E9          ///< GLONASS G2 base frequency (Hz)
#define DFRQ2_GLO   0.437500E6          ///< GLONASS G2 bias frequency (Hz/n)
#define FREQ3_GLO   1.202025E9          ///< GLONASS G3 frequency (Hz)

///< 新一代GLONASS三频点
#define FREQ1_GLN   1.600995E9          ///< GLONASS G1 frequency (Hz), 156.5*10.23MHZ
#define FREQ2_GLN   1.248060E9          ///< GLONASS G2 frequency (Hz), 122*10.23MHZ
#define FREQ3_GLN   1.202025E9          ///< GLONASS G3 frequency (Hz), 117.5*10.23MHZ

#define FREQ1_BD2   1.561098E9          ///< BeiDou2 B1     frequency (Hz) B1I    , 152.6*10.23MHZ
#define FREQ2_BD2   FREQ7               ///< BeiDou2 B2(B7) frequency (Hz) B2I/B2b, 118*10.23MHZ
#define FREQ3_BD2   1.268520E9          ///< BeiDou2 B3(B6) frequency (Hz) B3I    , 124*10.23MHZ

///< BD3的IQX通道信号频率和BD2是一样的
///< BD3的DPXAZ通道信号频率是新的
#define FREQ1_BD3   FREQ1_BD2           ///< BeiDou3 B1 frequency (Hz) B1I    , 152.6*10.23MHZ
#define FREQ2_BD3   FREQ2_BD2           ///< BeiDou3 B2 frequency (Hz) B2I/B2b, 118*10.23MHZ
#define FREQ3_BD3   FREQ3_BD2           ///< BeiDou3 B3 frequency (Hz) B3I    , 124*10.23MHZ
#define FREQ4_BD3   FREQ1               ///< BeiDou3 B4 frequency (Hz) B1C    , 154*10.23MHZ
#define FREQ5_BD3   FREQ5               ///< BeiDou3 B5 frequency (Hz) B2a    , 115*10.23MHZ

#define FREQ1_GAL   FREQ1               ///< Galileo E1    frequency (Hz), 154*10.23MHZ
#define FREQ2_GAL   FREQ5               ///< Galileo E5a   frequency (Hz), 115*10.23MHZ
#define FREQ3_GAL   FREQ7               ///< Galileo E5b   frequency (Hz), 118*10.23MHZ
#define FREQ4_GAL   FREQ8               ///< Galileo E5a+b frequency (Hz), 116.5*10.23MHZ
#define FREQ5_GAL   FREQ6               ///< Galileo E6    frequency (Hz), 125*10.23MHZ

#define FREQ1_QZS   FREQ1               ///< QZSS L1     frequency (Hz), 154*10.23MHZ
#define FREQ2_QZS   FREQ2               ///< QZSS L2     frequency (Hz), 120*10.23MHZ
#define FREQ3_QZS   FREQ5               ///< QZSS L5     frequency (Hz), 115*10.23MHZ
#define FREQ4_QZS   FREQ6               ///< QZSS L6/LEX frequency (Hz), 125*10.23MHZ

#define FREQTYPE_L1  0x01               ///< frequency type: L1/E1/B1
#define FREQTYPE_L2  0x02               ///< frequency type: L2/B2
#define FREQTYPE_L5  0x04               ///< frequency type: L5/E5a/B3
#define FREQTYPE_L6  0x08               ///< frequency type: E6
#define FREQTYPE_L7  0x10               ///< frequency type: E5b
#define FREQTYPE_L8  0x20               ///< frequency type: E5(a+b)
#define FREQTYPE_ALL 0xFF               ///< frequency type: all

#define SYSNON      0x00
#define SYSGPS      0x01
#define SYSGLO      0x02
#define SYSBD2      0x04
#define SYSBD3      0x08
#define SYSGAL      0x10
#define SYSQZS      0x20
#define SYSIRN      0x40
#define SYSLEO      0x80
#define SYSALL      (SYSGPS|SYSGLO|SYSBD2|SYSBD3|SYSGAL|SYSQZS)

#define ISYSNON     -1
#define ISYSGPS     0
#define ISYSGLO     1
#define ISYSBD2     2
#define ISYSBD3     3
#define ISYSGAL     4
#define ISYSQZS     5
#define ISYSIRN     6
#define ISYSLEO     7

///< 系统及卫星数
//#define NSYS        6

#define PRNGPS      0
#define NSATGPS     32

#define PRNGLO      (PRNGPS + NSATGPS)
#define NSATGLO     1  //27

#define PRNBD2      (PRNGLO + NSATGLO)
#define NSATBD2     18 //18

#define PRNBD3      (PRNBD2 + NSATBD2)
#define NSATBD3     45				///< 3GEO+3IGSO+24MEO

#define PRNGAL      (PRNBD3 + NSATBD3)
#define NSATGAL     36  //36

#define PRNQZS      (PRNGAL + NSATGAL)
#define NSATQZS     1  //8

#define NSATMAX     (NSATGPS + NSATGLO + NSATBD2 + NSATBD3 + NSATGAL + NSATQZS)

#define NRCV		3					///< 流动站GNSS接收机数量

#define OBS_PI      0
#define OBS_LI      1
#define OBS_DI      2

#define GNSSMODE_NONE       0
#define GNSSMODE_SPP        1			///< positioning mode: SPP
#define GNSSMODE_PPP_KINEMA 2			///< positioning mode: PPP-kinematic
#define GNSSMODE_PPP_STATIC 3			///< positioning mode: PPP-static
#define GNSSMODE_PPP_LEO    4			///< positioning mode: PPP-leo
#define GNSSMODE_SINGLE_MAX 4			///< single mode max

#define GNSSMODE_RTD        5			///< positioning mode: DGPS/DGNSS
#define GNSSMODE_RTK_KINEMA 6			///< positioning mode: RTK-kinematic
#define GNSSMODE_RTK_STATIC 7			///< positioning mode: RTK-static
#define GNSSMODE_RTK_MOVEB  8			///< positioning mode: moving-base

#define GNSSMODE_APOS_SPP   9			///< positioning mode: SPP for Receiver Array
#define GNSSMODE_APOS_RTD   10			///< positioning mode: RTD for Receiver Array
#define GNSSMODE_APOS_RTK   11			///< positioning mode: RTK for Receiver Array

#define GNSSMODE_ALL_MAX    11			///< all PMODE

#define ARSTATE_NONE     0				///< positioning mode: none
#define ARSTATE_FLOAT    1				///< positioning mode: float
#define ARSTATE_FLOFIX   2				///< positioning mode: float->fixed(固定的卫星数小于3颗)
#define ARSTATE_FIXED    3				///< positioning mode: fixed
#define ARSTATE_HOLD     4				///< positioning mode: fixed
#define ARSTATE_DFIX     5				///< positioning mode: double fixed

#define ARMODE_OFF       0				///< AR mode: off
#define ARMODE_CONT      1				///< AR mode: continuous
#define ARMODE_INST      2				///< AR mode: instantaneous
#define ARMODE_FIXHOLD   3				///< AR mode: fix and hold
#define ARMODE_PPPAR_FCB 4				///< AR mode: PPP-AR
#define ARMODE_PPPAR_IRC 5				///< AR mode: PPP-AR ILS
#define ARMODE_WLNL      6				///< AR mode: wide lane/narrow lane
#define ARMODE_TCAR      7				///< AR mode: triple carrier ar
#define PMODE_PPP_MAX    10

#define SOLTYPE_NONE        0           ///< positioning solution type: none
#define SOLTYPE_SPP         1  
#define SOLTYPE_SPP_EKF     2   
#define SOLTYPE_RTK_PRE     3 
#define SOLTYPE_RTD         4           ///< using pseudorange updates only
#define SOLTYPE_RTK_FLO     5           
#define SOLTYPE_RTK_FLOFIX  6  
#define SOLTYPE_RTK_FIXED   7
#define SOLTYPE_RTK_HOLD    8

#ifdef POSTPROIPS
#define MAXEPHNUM        120             ///< max number of stored ephemeris per satellite, for real-time, it is set to 1
#else
#define MAXEPHNUM        1             ///< max number of stored ephemeris per satellite, for real-time, it is set to 1
#endif

#define MAXCODE          46				///< max number of obs code
#define MAXBASESITE      3				///< max number of base site
#define IROVE			 0
#define IBASE			 1

///< RTK member parameter
#define DTTOL_M     (60.005)			///< tolerance of time difference for matching (s)
#define DTTOL       (0.005)				///< tolerance of time difference (s)
#define TTOL_MOVEB  (1.0+2*DTTOL)		///< time sync tolerance for moving-baseline (s)
#define INIT_ZWD    (0.15)				///< initial zwd (m)
#define VAR_HOLDAMB (0.001)				///< constraint to hold ambiguity (cycle^2)(这个值影响很大)

///< 整体平差(Common LS)中的事件标识
#define CLS_EVENT_P_VALIDATE     0x01	///< [P码正常,在CLS中可以使用该P观测值]
#define CLS_EVENT_L_VALIDATE     0x02	///< [L码正常,在CLS中可以使用该L观测值]
#define CLS_EVENT_GF             0x04	///< [GF周跳]
#define CLS_EVENT_MW             0x08	///< [MW周跳]
#define CLS_EVENT_LLI            0x10	///< [LLI周跳]
#define CLS_EVENT_P_OUTLIER      0x20	///< [P码粗差]
#define CLS_EVENT_L_OUTLIER      0x40	///< [L码粗差]
#define CLS_EVENT_P_OUTAGE       0x80	///< [P码连续跟踪时间太小或者观测值为0]
#define CLS_EVENT_L_OUTAGE       0x100	///< [L码连续跟踪时间太小或者观测值为0]

///< 观测值解算标识
///< 注意:降权或模糊度操作的观测值是可用的
///<     而大于OBSEVT_Exclude标志的观测值不可用
#define OBSEVT_NONE              0x00		///< 未知
#define OBSEVT_DW				 0x01		///< 观测值降权
#define OBSEVT_REAMB			 0x02		///< 初始化模糊度
#define OBSEVT_IAAMB			 0x04		///< 放大模糊度方差
#define OBSEVT_IBAMB             0x08		///< 放大模糊度方差
#define OBSEVT_Exclude           0x10		///< 下述情况的观测值将直接排除
#define OBSEVT_MISS              0x20		///< 数据缺失
#define OBSEVT_SSVH              0x40		///< 卫星不健康
#define OBSEVT_ELEV              0x80		///< 高度角或SNR不通过
#define OBSEVT_SNR               0x100		///< 高度角或SNR不通过
#define OBSEVT_DELE              0x200		///< 外部删去
#define OBSEVT_XMEQ              0x400		///< 列方程出现错误,例如卫地距无法计算,对流层/电离层出现问题
#define OBSEVT_Outlier			 0x800		///< 当前观测值为粗差,由下列方法探测得到
#define OBSEVT_XDOP              0x1000		///< 与多普勒的一致性不好
#define OBSEVT_DFRQ              0x2000		///< 不同频率间观测值做差,适用于多普勒和伪距
#define OBSEVT_Inno_v            0x4000		///< 验前残差探测为粗差
#define OBSEVT_Inno_nv           0x8000		///< 验前标准化残差探测为粗差
#define OBSEVT_Resi_v            0x10000	///< 验后残差探测为粗差
#define OBSEVT_Resi_nv           0x20000	///< 验后标准化残差探测为粗差

///< 周跳标识
#define CST_LLI  0x01					///< LLI周跳&Doppler
#define CST_GF   0x02					///< GF周跳
#define CST_MW   0x04					///< MW周跳
#define CST_Resi 0x08					///< PPP/RTK验后残差探测得到的周跳
#define CST_OUTC 0x10					///< 失锁引起的周跳
#define CST_ALL  0x20					///< 因周跳过多而判断所有卫星都发生周跳
#define CST_TDCP 0x40					///< TDCP检测出来的周跳

#define CSR_FIT_SIZE 120				///< 120个用来建模
#define CSR_PRE_SIZE 20					///< 20个用来预报
#define CSR_SIZE 140					///< 120个用来建模,20个用来预报
#define CSR_IdPos(I) (I*4)				///< 位置变化量的索引
#define CSR_IdClk(I) (I*4+3)			///< 钟差变化量的索引


///< 星历类型
typedef enum
{
    EPH_BRDC,
    EPH_PRE,
    EPH_IGS,
    EPH_IGR,
    EPH_IGU
} EPH_TYPE;


///< 对流层类型
typedef enum
{
    TROP_NONE,
    TROP_SS,
    TROP_GS,
    TROP_UM,
    TROP_SMF,
    TROP_NMF,
    TROP_GMF,
    TROP_VMF1,
    TROP_MIT,
    TROP_JPL,
    TROP_BER
} TROP_TYPE;


///< 电离层类型
typedef enum
{
    IONO_NONE,
    IONO_KLOB,
    IONO_IFLC
} IONO_TYPE;


///< 解算类型
typedef enum
{
    SOLVE_RTCM_SERIAL,  // 通过读写串口方式获取 流动站/基站 RTCM 数据
    SOLVE_FILE,         // 通过读写文件方式获取 流动站/基站 数据
} SOLVE_TYPE;


///< 输入数据类型
typedef enum
{
    RINEX,
    NMEA,
    RTCM,
} DATA_TYPE;


///< 三频观测组合类型
typedef enum 
{
    TFLC_L1 = 0,
    TFLC_L2 = 1,
    TFLC_L3 = 2,
    TFLC_L12_IF,
    TFLC_L13_IF,
    TFLC_L23_IF,
    TFLC_L123_IF,
    TFLC_L12_GF,
    TFLC_L13_GF,
    TFLC_L23_GF,
    TFLC_L12_MW,
    TFLC_L13_MW,
    TFLC_L23_MW,
    TFLC_L12_WL,
    TFLC_L13_WL,
    TFLC_L23_WL,
    TFLC_L12_NL,
    TFLC_L13_NL,
    TFLC_L23_NL
} TFLCTYPE;


///< 采用如下表述进行类型识别
typedef enum
{
    GDT_GeodeticReceiver = 0x10000 + 0,		///< 大地测量型设备(Trimble,Novatel,Javad,Septentrio,Topcon...)
    GDT_Geodetic_Trimble = 0x10000 + 1,
    GDT_Geodetic_Novatel = 0x10000 + 2,

    GDT_LowCostReceiver  = 0x20000 + 0,		///< 低成本模组设备(uBlox,华大北斗...)
    GDT_LowCost_uBloxM8T = 0x20000 + 1,
    GDT_LowCost_uBloxF9P = 0x20000 + 2,
    GDT_LowCost_uBloxF9PN = 0x20000 + 3,    ///< L1/L5频段，edit by Lying
    GDT_LowCost_TH1030   = 0x20000 + 4,     ///< L1/L5频段，edit by Lying

    GDT_SmartPhone       = 0x40000 + 0,		///< 手机芯片级设备(HUAWEI P30,MI8,Samsung...)
    GDT_SmartPhone_P30   = 0x40000 + 1,
    GDT_SmartPhone_MI8   = 0x40000 + 2,
} GNSS_Device_TYPE;

static const GNSS_Device_TYPE gs_GNSSDeviceType = GDT_LowCostReceiver;

///*============================ end define ========================================*/






/*============================ start const =======================================*/
///< global static = gs_ : 全局静态变量

static const double gs_GLO_Freq[] = { FREQ1_GLO,FREQ2_GLO,FREQ3_GLO };
static const double gs_GLO_dFreq[] = { DFRQ1_GLO,DFRQ2_GLO,0.0 };
static const int    gs_GLO_iFreq[] = { -9999, 1,-4,5,6,1,-4,5,6,-2,-7,0,-1,-2,-7,0,-1,4,-3,3,2,4,-3,3,2,-6,-6,-6 }; ///< 第一个-9999无效,为了直接通过sat号来索取
static const double gs_GPSL1Lam = (CLIGHT / FREQ1_GPS);

// It's used when manually switching frequencies, edited by Lying
static const bool gs_bSwitchGNSSFrq = true;
extern char gs_strGPSFrq[NFREQ][5];
extern char gs_strGLOFrq[NFREQ][5];
extern char gs_strBD2Frq[NFREQ][5];
extern char gs_strBD3Frq[NFREQ][5];
extern char gs_strGALFrq[NFREQ][5];
extern char gs_strQZSFrq[NFREQ][5];

static const int    gs_Index2Sys[NSYS] = { SYSGPS, SYSGLO, SYSBD2, SYSBD3, SYSGAL, SYSQZS };

static const double gs_ura_eph[15] = {
    2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
    3072.0,6144.0
};

///*============================ end const =========================================*/






///*============================ start struct ======================================*/

///< It's used for intermediate result output during debugging, edited by Lying
typedef struct tagLOGDATA
{
    // 观测数据
    GPSTIME gt;                 ///< gps time
    int nSat;                   ///< 卫星总数

    // SPP
    int SPPnMeas;               ///< SPP中伪距观测值数量
    int SPVnMeas[2];            ///< SPV_EKF中多普勒、相位观测值数量
    int SPPSolType;             ///< SPP
    int SPVSolType;             ///< SPV_EKF
    double SPVXYZ[3];           ///< SPV_XYZ

    // RTK
    int RTKnMeas[2];            ///< RTK中双差伪距、双差相位数量
    int RTKSolType[2];          ///< RTK中伪距和相位是否更新成功
    bool bReinitAmb[NFREQ];     ///< 所有模糊度是否被重置

    // AR
    int valAmbs;                ///< 进入模糊度固定的候选模糊度个数 m_nPara[2]
    int fixAmbs;                ///< 部分法固定的模糊度个数
    double fixRatio;            ///< 部分法固定的RATIO值
    double fixPDOP;			    ///< runPAR中计算的位置精度因子
    double fixPosDif;           ///< 固定解与浮点解差异
    double fixAmbRms;           ///< 固定解与浮点解差异

}LOG_DATA;
void InitLOGDATA(LOG_DATA* pStruct);


///< It's used for backing up data to save memory, edited by Lying
typedef struct tagOBSDATAT_PL
{
    unsigned char prn;
    double L[NFREQ];
    double P[NFREQ];

}OBS_DATA_t_PL;
void InitOBSDATAT_PL(OBS_DATA_t_PL* pStruct);


///< Observations
typedef struct tagOBSDATAT      ///< Obs 观测值主体
{
    unsigned char   prn;
    double  L[NFREQ];           ///< observation data carrier-phase (cycle)
    double  P[NFREQ];           ///< observation data pseudorange (m)
    double  D[NFREQ];           ///< observation data doppler frequency (Hz)
    float   S[NFREQ];           ///< signal strength
    char    code[NFREQ][3];     ///< P码类型(通道)

    unsigned char SNR[NFREQ];   ///< signal strength (0.25 dBHz)
    unsigned char LLI[NFREQ];   ///< loss of lock indicator

}OBS_DATA_t;
void InitOBSDATAT(OBS_DATA_t* pStruct);


///< 
typedef struct tagOBSDATA
{
    GPSTIME gt;                 ///< gps time
    char flag;                  ///< 标识位,0:可用
    int nsat;                   ///< 卫星总数
    int ngnss[NSYS];            ///< 各卫星数
    OBS_DATA_t obs[MAXOBS];     ///< 观测数据体

}OBS_DATA;
void InitOBSDATA(OBS_DATA* pStruct);


typedef struct tagGPSEPH        ///< GPS broadcast ephemeris type
{
    unsigned char prn;          ///< satellite number
    int iode, iodc;             ///< IODE,IODC
    double sva;                 ///< SV accuracy (m)
    char svh;                   ///< SV health (0:ok)
    short int week;             ///< GPS/QZS: gps week, GAL: galileo week
    int code;                   ///< GPS/QZS: code on L2, GAL: data sources
    GPSTIME toe, toc/*,ttr*/;   ///< Toe,Toc,T_trans
                                ///< SV orbit parameters
    double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
    double crc, crs, cuc, cus, cic, cis;
    double toes;                ///< Toe (s) in week
    double f0, f1, f2;          ///< SV clock parameters (af0,af1,af2)
    double tgd[4];              ///< group delay parameters
                                ///< tgd[0]: L1/L2, B1/B3, tgd[1]: B2/B3

} GPSEPH, BDSEPH, GALEPH, QZSEPH;
void InitGPSEPH(GPSEPH* pStruct);


typedef struct tagGLOEPH        ///< GLONASS broadcast ephemeris type
{
    unsigned char prn;          ///< satellite number
    int iode;                   ///< IODE (0-6 bit of tb field)
    int frq;                    ///< satellite frequency number
    int svh, sva, age;          ///< satellite health, accuracy, age of operation
    GPSTIME toe;                ///< epoch of epherides (gpst)
    GPSTIME tof;                ///< message frame time (gpst)
    double pos[3];              ///< satellite position (ecef) (m)
    double vel[3];              ///< satellite velocity (ecef) (m/s)
    double acc[3];              ///< satellite acceleration (ecef) (m/s^2)
    double taun, gamn;          ///< SV clock bias (s)/relative freq bias

} GLOEPH;
void InitGLOEPH(GLOEPH* pStruct);


typedef struct tagAMBC {        ///< ambiguity control type

    ///< 原始观测值固定
    double  vL[NFREQ];          ///< L的标准差
    double  NL[NFREQ];          ///< L小数部分/L的标准差
    bool    bBad[NFREQ];        ///< 采用方差膨胀法探测NL中异常值，辅助子集排序

    ///< 求宽巷/窄巷所用
    GPSTIME gt[NFREQ];          ///< last epoch
    int     n[NFREQ];           ///< number of epochs
    double  LC[NFREQ];          ///< linear combination average(cycle)
    double  LCv[NFREQ];         ///< linear combination variance(cycle)
    bool    bFixWL;             ///< 宽巷是否固定成功
    int     iWL;                ///< 宽巷/窄巷整数部分
    double  vN1;                ///< 宽巷/窄巷标准差

} AMBC;
void InitAMBC(AMBC* pStruct);


typedef struct tagSATSTATUS            ///< satellite status type
{
    ///< 基本信息
    char          bExclude;			   ///< 该卫星被排除. 0:正常; 1:非选中系统; 2:opt中排除该卫星; 4:卫星不健康;
    int           sys;                 ///< 卫星系统
    char          sys_id;              ///< 卫星系统序号
    char          sat;                 ///< 系统内序号

    double        lam[NFREQ];          ///< 波长(m)
    double        frq[NFREQ];          ///< 频率(HZ), frq = C/lam
    double        azel[2];             ///< azimuth/elevation angles {az,el} (rad)
    int           lock[NFREQ];         ///< lock counter of phase
    int           outc[NFREQ];         ///< obs outage counter of phase
    float         S[NFREQ];			   ///< signal strength, OBS_DATA_t中的S观测值, 不是SNR值

    // 观测值事件标志:-1:正常, 0:未知, >0:有问题[OBSEVT_]
    bool          OBSUsed[NFREQ];      ///< 选星算法后实际参与解算的卫星标记 wyz20211213
    bool          OBSValid[3][NFREQ];  ///< P/L/D Valid flag for SPP/PPP/RTK
    int           OBSEvent[3][NFREQ];  ///< P/L/D Event flag for SPP/PPP/RTK

    ///< 模糊度信息
    double        Amb0[NFREQ];         ///< PPP/RTK每个时刻的模糊度初值,伪距减去相位
    bool          bResetAmb[NFREQ];    ///< 模糊度重新初始化

    ///< 周跳信息
    char          CS_Type[NFREQ];      ///< 周跳类型,bit0:LLI,bit1:GF,bit2:MW
    GPSTIME       CS_GFTime[NFREQ];    ///< 当前历元的GF时间
    double        CS_GF[NFREQ];        ///< 当前时刻GF值,L1-L2,L1-L5,L2-L5(m)
    double        CS_dGF[NFREQ];       ///< 当前时刻dGF值
    double        CS_MW[NFREQ];        ///< Melbourne-Wubbena (cycle)
    unsigned int  CS_nMW[NFREQ];       ///< number of wilde-lane bias
    unsigned int  CS_GFNum[NFREQ];     ///< number of cycle slipcycle slip being detected by GF for every frequency
    bool          CS_bGFMulti;         ///< more than 2 times cycle slip has been detected by GF for a certain frequency

    ///< 周跳修复信息:

    ///< AR信息
    char          AR_Status[NFREQ];    ///< AR状态 (0:none,1:float,2:fix,3:hold)
    char          AR_StatusPre[NFREQ]; ///< 前一历元的AR状态
    int           AR_AmbPre[NFREQ];    ///< 前个历元固定的模糊度数值
    int           AR_Amb[NFREQ];       ///< 当前历元固定的模糊度数值
    int           AR_nFixed[NFREQ];    ///< 模糊度连续固定为某个数
    int           AR_Lock[NFREQ];      ///< 模糊度锁定的个数
    AMBC          AR_AmbCtrl;          ///< 宽巷/窄巷模糊度固定的结构体

    ///< 卫星信息
    char          Satsvh;              ///< 卫星健康,0:健康
    double        SatPos[3];           ///< 卫星位置
    double        SatVel[3];           ///< 卫星速度
    double        SatClk;              ///< 卫星钟差
    double        SatClkVel;           ///< 卫星钟速

    // 质量控制信息
    int           OBSIndex;			   ///< 在观测值列表中的位置
    double		  OBSConsist[3][NFREQ];///< 0:伪距历元差减去多普勒,1:相位历元差减去多普勒,2:多普勒历元差

    double        SD_L[NFREQ];         ///< 站间单差相位（流动站-基准站）

    double        DD_L[NFREQ];         ///< 相位双差残差，以m为单位
} SATSTATUS;
void InitSATSTATUS(SATSTATUS* pStruct);


typedef struct tagGNSSMEQ
{
    bool   bMEQ;					    ///< 经过setMEQBase()后判断该观测值是否可用
    int    GLSPosP[NFREQ];			    ///< 伪距观测值在GLS中的位置
    int    GLSPosL[NFREQ];			    ///< 相位观测值在GLS中的位置
    double H_XYZ[3];				    ///< XYZ 的观测系数
    double v_P[NFREQ];				    ///< 伪距新息(残差)
    double v_L[NFREQ];				    ///< 相位新息(残差),未做模糊度改正
    double v_D[NFREQ];				    ///< 多普勒新息(残差)
    double tdv_L[NFREQ];				///< 相位三差，以m为单位
    double R_P[NFREQ];				    ///< 伪距方差
    double R_L[NFREQ];				    ///< 相位方差
    double R_D[NFREQ];				    ///< 多普勒方差
    double Trp_base;				    ///< 基站的总对流层,一次迭代中是不变的

    double RFactL[NFREQ];			    ///< 相位方差的总膨胀因子
    double RFactP[NFREQ];			    ///< 伪距方差的总膨胀因子
    double RFactD[NFREQ];			    ///< 多普勒方差的总膨胀因子
    double RFactLCur[NFREQ];		    ///< 相位方差的当前膨胀因子,(9999.0: 模糊度重新初始化,等价权赋为1.0)
    double RFactPCur[NFREQ];		    ///< 伪距方差的当前膨胀因子
    double RFactDCur[NFREQ];		    ///< 多普勒方差的当前膨胀因子
    int    AmbFlag[NFREQ];			    ///< 模糊度方差膨胀指标,1-5:方差膨胀程度,9:模糊度重新初始化

    int    OBSIndex;				    ///< RTK中基站数据的序号索引

    // 质量控制信息
    int			  QC_islip[NFREQ];      ///< cycle slip flag(0:ok,1:susp,2:exact) ？是否需要
    int			  QC_iresi[NFREQ * 3];  ///< residual index(0:ok)
    int			  QC_isres[NFREQ * 3];  ///< standard residual index(0:ok)
    int			  QC_isresbad[NFREQ];   ///< standard resi bad flag
    int			  QC_badcnt[NFREQ];     ///< standard resi bad count ？是否需要
    bool		  QC_CycleSlip[NFREQ];  ///< TDCP探测出的周跳信息
    bool		  QC_bGrossPRtk[NFREQ]; ///< Rtk部分探测出的伪距粗差 ？是否需要

} GNSS_MEQ;
void InitGNSS_MEQ(GNSS_MEQ* pStruct);


typedef struct tagGNSS_GLSINFO
{
    int IPos;
    int LPos;
    int IVel;
    int LVel;
    int IAcc;
    int LAcc;
    int IClk;
    int LClk;
    int IClkVel;
    int LClkVel;
    int ITrp;
    int LTrp;
    int IAmb;
    int LAmb;
    int EAmb;
    int IIon;
    int LIon;
    int EIon;
    int	LStateConst;						    ///< 所有参数个数
    int	LStateFixed;						    ///< 除了模糊度以外的参数个数

    int         StateIndex[MAXRTKNX];		    ///< 当前滤波可估参数在StateX中的索引
    int         nStateIndex;                    ///< StateIndex 数量
    double	    StateX[MAXRTKNX];			    ///< 全局存放所有状态值(当观测更新失败时,一直存预报值)
    double	    StateP[MAXRTKNX * MAXRTKNX];	///< 全局存放所有状态方差
    double	    StateXa[MAXRTKNX];			    ///< 全局存放所有状态值,固定解
    double	    StatePa[MAXRTKNX * MAXRTKNX];   ///< 全局存放所有状态方差,固定解
    double	    MKFStateX[MAXRTKNX];            ///< MKF中需要用到的StateX
    double	    MKFStateP[MAXRTKNX * MAXRTKNX]; ///< MKF中需要用到的StateP
    int         m_StateSolType;                 ///< 状态向量StateX的解类型
    double      m_Pos0[3];                      ///< RTK先验位置
    double      m_PosVar0[9];                   ///< RTK先验位置协方差
    int         m_Pos0Type;                     ///< RTK先验位置来源，SPP or SPP EKF
    int         m_VelMeasType;                  ///< dXYZ来源, 0:NULL,1:TDCP,2:PVA观测,3:MEMS,4:PVA预报,5:常速度
    double		m_dXYZ[3];				        ///< 位移变化量
    double		m_dXYZVar[9];		            ///< 位移变化量的方差 给到RTK
    double		m_tEKFPredict;                  ///< EKF预报的时间间隔（s）

    int         m_AmbIndex[NSATMAX * NFREQ];    ///< 建立StateX中Prn、Frq与模糊度参数索引的映射关系
    int			nMeas;                          ///< 当前观测值个数
    int			nMeasL;                         ///< 相位观测个数
    int			nMeasP;                         ///< 伪距观测个数
    int			nMeasD;                         ///< 多普勒观测个数
    int			nMeasI;                         ///< 电离层虚拟观测个数

    int			nOutEpoch;                      ///< 丢失的历元数
    bool		bReinitAllAmb[NFREQ];           ///< 重新初始化所有周跳
    double      DOPs[4];

    int         nMEQ;
    GNSS_MEQ	MEQ[MAXOBS];

}GNSS_GLSINFO;
void InitGNSS_GLSINFO(GNSS_GLSINFO* pStruct);


///*============================ end struct ========================================*/


#endif
