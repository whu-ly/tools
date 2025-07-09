/**
* @file
* @brief    Signal Point Positioning(SPP).
* @details  单点定位.
* @author   Ying Liu et al from PLANET. Email: liuying1@whu.edu.cn
* @date     2024/10/06
* @version  2.2
* @par      Copyright(c) 2021-2024 School of Geodesy and Geomatics, Wuhan University. All Rights Reserved.
* @par      History:
*           2024/10/06, Ying Liu, modify the code to comply with the C language specification.
*/


#ifndef GNSSTK_SPPOINT_H
#define GNSSTK_SPPOINT_H


#include "CLSEstimator.h"
#include "CILSEstimator.h"
#include "CGNSSMember.h"
#include "CEphemerisComputer.h"
#include "CIonosphere.h"
#include "CTroposphere.h"


/**
* @brief    观测值使用情况
*
* @note     每次观测方程中使用的卫星号及频率
*/
typedef struct tagOBSUSED
{
    unsigned char f;		///< 第i个观测值的频率
    unsigned char prn;	    ///< 第i个观测值的prn

} OBS_USED;


/**
* @brief    SPP
*
* @note     实际包括单点定位、测速等功能
*/
typedef struct tagCSPPoint
{
    ///< 外部初始化信息
    OBS_DATA* m_pOBSData;							///< 数据体
    CEphemerisComputer* m_pEphComputer;						///< 星历计算器
    CGLSEstimator* m_GLS;								///< EKF估计器
    SATSTATUS* m_pSatStatus;                       ///< PVD-解算中间参数与状态
    CGNSSOption* m_pGNSSOpt;							///< GNSS Option
    CGNSSSolution* m_pGNSSSol;							///< GNSS Solution
    GNSS_GLSINFO* m_pGLSInfo;                         ///< 储存共视卫星信息    

    ///< 解算方法开关
    bool				m_bEstDis;							///< 是否使用TDCP估计位移
    bool                m_bUseCons;                         ///< 测速中是否使用虚拟约束

    ///< 估计器
    CLSEstimator		m_LSQ;                              ///< LSQ估计器
    CILSEstimator       m_ILS;                              ///< 整数估计器
    int					m_nState;							///< LS 或 GLS 中实际状态量个数
    int					m_nMeas;							///< LS 或 GLS 中观测值个数
    double				m_B[MAXOBSLIM * MAXLSQNX];			///< 观测矩阵，给 LS 或 GLS 估计器赋值，使用前需清零，实际维数 MAXOBSLIM * MAXLSQNX
    double				m_V[MAXOBSLIM];						///< 观测残差，同 m_B
    double				m_R[MAXOBSLIM * MAXOBSLIM];			///< 观测权阵，同 m_B

    ///< SPP
    unsigned char		m_SPPSolType;						///< 位置解算标识
    int					m_SPPnMeas;							///< SPP有效观测值数
    double				m_SPPXYZ[3];						///< 位置XYZ
    double				m_SPPXYZP[9];						///< 位置方差
    double				m_SPPLLH[3];						///< 位置LLH
    bool				m_bSPPClk[NSYS * NFREQ];			///< 是否存在钟差
    double				m_SPPClk[NSYS * NFREQ];				///< 钟差(m)
    double				m_SPPDOPs[4];						///< 位置DOP,0:GDOP,1:PDOP,2:HDOP,3:VDOP
    double              m_SPPLSQRMS;                        ///< 验后残差RMS

    ///< SPV
    unsigned char		m_SPVSolType;						///< 速度解算标识
    double				m_SPVXYZ[3];						///< 速度XYZ (正反向都是前向速度，即多普勒瞬时测速结果)
    double				m_SPVXYZP[9];						///< 速度方差
    double				m_SPVClkVel[NSYS];					///< 钟速(m/s)
    double				m_SPVDOPs[4];						///< 速度DOP
    double              m_SPVLSQRMS;                        ///< 验后残差RMS

    ///< TDCP, 3Dis+1Clk+nCs
    unsigned char		m_DisSolType;						///< 位移解算标识
    int					m_DisnMeas;					        ///< 有效观测值数
    double				m_DisXYZ[3];						///< 位移XYZ (正反向都是前向速度，与多普勒测速结果一致)
    double				m_DisXYZP[9];					    ///< 位移方差
    double				m_DisClkVel;				        ///< 钟差变化(m)
    double				m_DisDOPs[4];						///< DOP
    double              m_DisLSQRMS;                        ///< 验后残差RMS
    bool                m_bDisUsedPrn[NSATMAX + 1];         ///< 星历切换的卫星不参与TDCP解算

    int                 m_DisICS;                           ///< 周跳状态起始位置, for RCS
    int                 m_nEstCS;                           ///< 待估计周跳个数
    int                 m_iCS_PrnFrq[MAXOBSLIM];            ///< 待估计周跳索引对应prn和frq
    int                 m_iPrnFrq_CS[NSATMAX * NFREQ];      ///< prn和frq对应待估计周跳索引
    double              m_DisCSF[MAXOBSLIM];                ///< 周跳浮点解估值, cycle

    ///< SPP EKF
    int			        m_SPPEKFSolType;					///< SPP EKF解算标识，-1未初始化，0初始化，1观测更新，2时间更新
    int					m_nSPPEKFPredict;				    ///< SPP EKF预报次数
    double				m_SPPEKFXYZ[3];						///< SPP EKF位置解
    double				m_SPPEKFXYZP[9];					///< SPP EKF位置解方差

    ///< MDOP+TDCP EKF, 3Vel+3Acc+2ClkVel
    bool				m_bVAUseL;							///< VA滤波中是否使用相位进行更新
    bool				m_bVAUseD;							///< VA滤波中是否使用多普勒进行更新
    bool				m_bVAEstAcc;						///< VA滤波中是否估计加速度
    bool				m_bVAISBRW;							///< VA滤波中钟速是否随机游走估计（或者白噪声）
    int					m_VAIVel;						    ///< 速度状态起始位置
    int					m_VALVel;						    ///< 速度状态长度
    int					m_VAIAcc;							///< 加速度状态起始位置
    int					m_VALAcc;							///< 加速度状态长度
    int					m_VAIClkVel;						///< 钟速状态起始位置
    int					m_VALClkVel;						///< 钟速状态长度
    int                 m_nVAState;                         ///< VA滤波中待估计参数个数，与m_VAStateX及m_VAStateP对应
    double				m_VAStateX[MAXSPVNX];			    ///< VA滤波中状态向量，实际维数 m_nVAState * 1
    double				m_VAStateP[MAXSPVNX * MAXSPVNX];	///< VA滤波中状态方差阵，实际维数 m_nVAState * m_nVAState
    double				m_VAStateXp[MAXSPVNX];				///< VA滤波中状态预报值，实际维数 m_nVAState * 1
    double				m_VAStatePp[MAXSPVNX * MAXSPVNX];	///< VA滤波中状态协方差阵预报值，实际维数 m_nVAState * m_nVAState
    int		            m_VAStateIndex[MAXSPVNX];			///< VA滤波中可估参数在列表中的索引，实际维数 m_iVAStateIndex * 1
    int                 m_iVAStateIndex;
    double              m_VAStatePhi[36];					///< VA滤波中Vel,Acc的离散状态转移矩阵
    double              m_VAStateQ[36];						///< VA滤波中Vel,Acc的离散过程噪声
    double				m_VAResiThres;						///< 残差阈值
    double				m_VAResiThresStaL;					///< 相位标准化残差阈值
    double				m_VAResiThresStaD;					///< 多普勒标准化残差阈值
    double				m_VAVel[3];							///< 滤波结果得到的速度
    double				m_VAVelP[9];						///< 滤波结果得到的速度协方差矩阵
    double				m_VADOPs[4];						///< VA滤波DOP
    int                 m_VAEKFSolType;						///< 滤波解算标识，-1未初始化，0初始化，1相位观测更新，2多普勒观测更新，3时间更新
    int                 m_nVAEKFPredict;				    ///< VA滤波中预报的次数
    int					m_nVAMeasL;							///< VA滤波中相位观测值数
    int					m_nVAMeasD;							///< VA滤波中多普勒观测值数
    bool                m_bVADisBadL;                       ///< VA滤波中当前历元是否发生抗差迭代

    ///< 解算中间参数
    OBS_USED			m_OBSUsed[MAXOBSLIM];				///< 每次观测方程中使用的卫星号及频率
    CIonosphere			m_IonoModel;						///< 电离层模型
    CTroposphere		m_TropModel;						///< 对流层模型
    GPSTIME				m_OBSTime;							///< 观测时间
    GPSTIME				m_SolTime;							///< 解算时间，只有UpdateSolution成功后才赋值
    GPSTIME             m_PriorPosRTKTime;                  ///< RTK滤波传递过来的先验位置对应时间，与m_FilterTime对应
    double				m_PriorPosClk[MAXSPPNX];		    ///< 先验位置钟差(XYZ,CLK)
    double				m_PriorPosSigma;					///< 先验位置的方差，< 999 说明 m_PriorPosClk 可用
    double              m_PriorPosRTK[3];                   ///< RTK滤波传递过来的先验位置
    double				m_azel[2][NSATMAX + 1];				///< azimuth / elevation angles{ az,el } (rad)
    double				m_GapTime;							///< 实际时间间隔，取了绝对值的，用是否逆向来取负
    double				m_Resi[MAXOBSLIM];                  ///< 验后残差
    double				m_ResiSig[MAXOBSLIM];				///< 验后方差
    double              m_ResiSta[MAXOBSLIM];               ///< 验后标准化残差
    bool				m_bGoodEnvState;					///< 通过周跳比例等标识判断当前环境优劣

    ///< 观测值与星历备份
    int					m_jOBSIndex[MAXOBS];                ///< 前后历元观测值索引
    OBS_DATA			m_jObsData;							///< 前一历元观测值
    int					m_jPosSolType;						///< 前一历元位置解状态
    double				m_jSPPXYZ[3];						///< 前一历元位置XYZ
    double				m_jSPPClk[NSYS * NFREQ];			///< 前一时刻钟差(m)
    double              m_jSatStatus_SatPos[NSATMAX + 1][3];
    double              m_jSatStatus_SatClk[NSATMAX + 1];
    signed char         m_jSatStatus_Satsvh[NSATMAX + 1];   ///< 前一历元卫星健康状态，0:健康
    GPSTIME             m_jEphtoe[NSATMAX + 1];

    FILE*               m_CSEstFp;                          ///< 输出静态数据周跳估计情况

} CSPPoint;


/** @brief  The constructor, which needs to be manually called when defining the struct */
extern void InitCSPPoint(CSPPoint* pStruct);


/** @brief  利用外部信息初始化SPP */
extern bool InitSPP(CSPPoint* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, CEphemerisComputer* pEphComputer, SATSTATUS* pSatStatus, CGLSEstimator* pGLSEst);


/** @brief  SPP解算总入口 */
extern bool runOneSPP(CSPPoint* pStruct, OBS_DATA* pOBSData);


#endif