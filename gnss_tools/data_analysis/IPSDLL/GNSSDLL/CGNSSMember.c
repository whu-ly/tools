#include "CGNSSMember.h"
#include "BaseCmnFunc.h"


///< Constructor function, to be called when defining
void InitCGNSSOption(CGNSSOption* pStruct)
{	
    if (!pStruct)return;

    // GNSS输入文件
    pStruct->m_BaseSiteID       = 0;

    // GNSS处理方式
    pStruct->m_SampleTime[IROVE] = 1.0;
    pStruct->m_SampleTime[IBASE] = 1.0;
    //pStruct->m_ProcTime = /*pStruct->m_SampleTime[IROVE]*/1.0;
    //pStruct->m_bDownProcTime = (pStruct->m_SampleTime[IROVE] == pStruct->m_ProcTime) ? false : true;
    pStruct->m_PMode            = GNSSMODE_RTK_KINEMA;
    pStruct->m_BaselineType     = 0;
    for (int i = 0; i < MAXBASESITE; i++)
        pStruct->m_BaseXYZ[i][0] = pStruct->m_BaseXYZ[i][1] = pStruct->m_BaseXYZ[i][2] = 0;
    pStruct->m_SatSYS           = SYSGPS | SYSBD2 | SYSBD3 | SYSGAL;
    for (int f = 0; f < NFREQ; f++) pStruct->m_bFreqUsed[f] = true;
    pStruct->m_RTKPredictMAXEpoch = 3;

    // GNSS预处理参数
    pStruct->m_ElevMask          = 1.0 * D2R;      // 截止高度角
    pStruct->m_ErrorPLR[ISYSGPS] = 300.0;          // code/phase error ratio
    pStruct->m_ErrorPLR[ISYSGLO] = 1000.0;          // code/phase error ratio
    pStruct->m_ErrorPLR[ISYSBD2] = 1000.0;          // code/phase error ratio
    pStruct->m_ErrorPLR[ISYSBD3] = 1000.0;          // code/phase error ratio
    pStruct->m_ErrorPLR[ISYSGAL] = 1000.0;          // code/phase error ratio
    pStruct->m_ErrorPLR[ISYSQZS] = 1000.0;          // code/phase error ratio
    pStruct->m_Error_a           = 0.003;
    pStruct->m_Error_b           = 0.003;
    pStruct->m_Error_EF[ISYSGPS] = 1.0;             // error factor: GPS
    pStruct->m_Error_EF[ISYSGLO] = 1.5;             // error factor: GLONASS
    pStruct->m_Error_EF[ISYSBD2] = 1.0;             // error factor: Beidou2
    pStruct->m_Error_EF[ISYSBD3] = 1.0;             // error factor: Beidou3
    pStruct->m_Error_EF[ISYSGAL] = 1.0;             // error factor: Galileo
    pStruct->m_Error_EF[ISYSQZS] = 1.0;             // error factor: QZSS
    for (int f = 0; f < NFREQ; f++)pStruct->m_SNRThres[f] = 1.0;
    pStruct->m_bCJ = true;
    pStruct->m_bCS_LLI = true;
    pStruct->m_bCS_GF = true;
    pStruct->m_bCS_MW = true;

    // GNSS误差改正

    // GNSS 模糊度固定
    pStruct->m_ARMode           = ARMODE_CONT;
    pStruct->m_ARType           = 0;
    pStruct->m_ARMinLock        = 3;
    pStruct->m_ARAmbLock[0]     = 5;
    pStruct->m_ARAmbLock[1]     = 3;
    pStruct->m_AR_PDOP          = 5.0;
    pStruct->m_AR_nSat          = 6;
    pStruct->m_AR_ADOP          = 0.1;
    pStruct->m_FixConsistCheck  = 5.0;
    pStruct->m_ARFixAmbRMS      = 5.0;
    pStruct->m_ARFix_nFixEpoch  = 0;
    pStruct->m_ARHold_nSat      = 6;
    pStruct->m_ARHold_ADOP      = 0.05;
    pStruct->m_ARHold_SRBS      = 0.99;
    pStruct->m_ARHold_Ratio     = 3.0;
    pStruct->m_ARHold_nFixEpoch = 3;
    pStruct->m_ARHold_nEpoch    = 3;

    // GNSS大气误差修正

    // GNSS数据质量控制
    pStruct->m_DataGapTol       = 300;
    pStruct->m_MaxOutage        = 3;
    pStruct->m_DDOPThres        = 20.0;
    pStruct->m_DCodeThres       = 8.0;
    pStruct->m_DPhaseThres      = 0.5;

    // GNSS参数估计策略
    pStruct->m_PosEstMode       = 0;///< 0:白噪声估计,1:常量随机游走,2:Dis/Vel驱动模型,3:加速度/速度模型(VA-EKF)

    // GNSS误差模型参数
    pStruct->BASE_ISDV_Vel2      = 100.0;
    pStruct->BASE_ISDV_Acc2      = 100.0;//1
    pStruct->BASE_PNSD_Pos2      = 0.0001;
    pStruct->BASE_PNSD_Vel2      = 0.25;//5
    pStruct->BASE_PNSD_Acc2      = 0.0025;//0.01
    pStruct->BASE_SPPPosSigma2   = 10000.0;
    pStruct->BASE_ISDV_ClkVel2   = 2500.0;
    pStruct->BASE_PNSD_ClkVel2   = 25.0;
    pStruct->BASE_SPPClkSigma2   = 10000.0;
    pStruct->BASE_SPPClkVelSigma2 = 100.0;
    pStruct->RTK_ISDV_Amb2       = 10000.0;

    pStruct->m_GNSSDeviceType = GDT_LowCostReceiver;
}


///< Constructor function, to be called when defining
void InitCGNSSSolution(CGNSSSolution* pStruct)
{
    if (!pStruct)return;

    pStruct->m_pGNSSOpt   = NULL;
    pStruct->m_pSatStatus = NULL;
    pStruct->m_pOBSData = NULL;
    pStruct->m_bUseLOG = true;

    InitGPSTIME(&pStruct->m_time);
    InitGPSTIME(&pStruct->m_FilterTime);
    InitLOGDATA(&pStruct->m_LOG);

    for (int i = 0; i < 9; i++)
    {
        if (i < 3)
        {
            pStruct->m_XYZPos[i] = pStruct->m_LLHPos[i] = pStruct->m_XYZVel[i] = pStruct->m_ENUVel[i] = pStruct->m_Att[i] = pStruct->m_BLENU[i] = 0;
        }

        pStruct->m_XYZPosP[i] = pStruct->m_ENUPosP[i] = pStruct->m_XYZVelP[i] = pStruct->m_ENUVelP[i] = pStruct->m_AttP[i] = pStruct->m_Rel[i] = 0;
    }

    for (int i = 0; i < NSYS; i++)
    {
        pStruct->m_ngnss[i] = pStruct->m_vgnss[i] = pStruct->m_fgnss[i] = 0;
    }

    pStruct->m_nsat = 0;
    pStruct->m_DDOP = 0;
    pStruct->m_PDOP = 0;
    pStruct->m_PDOP_IAR = 0;
    pStruct->m_HDOP = 0;
    pStruct->m_VDOP = 0;
    pStruct->m_ARRatio = 0.0;
    pStruct->m_AmbState = ARSTATE_NONE;
    pStruct->m_VelType = 0;
    pStruct->m_SolType  = SOLTYPE_NONE;
    pStruct->m_RMSL = 0.0;
    pStruct->m_RMSP = 0.0;
}


// CRC校验
static int NMEACRC(char* buff, int len)
{
    int checknum = 0;
    char* pdata = NULL;
    short i;

    if (!buff)return 0;

    pdata = buff;
    for (i = 0; i < len; i++)
    {
        checknum ^= *(pdata + i);
    }
    return checknum;
}


static bool WriteNEMA(CGNSSSolution* pStruct)
{
    if (!pStruct)return false;

    // 命令开关
    const bool bGGA = true;
    const bool bRMC = false;
    const bool bLOG = false/*pStruct->m_bUseLOG*/;

    double PDOP = (pStruct->m_PDOP < 99.9) ? pStruct->m_PDOP : 99.9;
    double HDOP = (pStruct->m_HDOP < 99.9) ? pStruct->m_HDOP : 99.9;
    double min = 0.0, Rel[9], ENUPosP[9];
    int    checkNum, checkStart, checkEnd;
    int    deg = 0, solLen = 0, QNMEA = 5;
    char   sol[MAXSIZE] = "";


    YMDHMS ymd = GPST2YMDHMS(GPST2UTC(pStruct->m_time));
    XYZ2LLH(pStruct->m_XYZPos, pStruct->m_LLHPos, 0);
    Rxyz2enu(pStruct->m_LLHPos, Rel);
    MatrixMultiply_HPHT(3, 3, Rel, pStruct->m_XYZPosP, false, ENUPosP);

    if (pStruct->m_SolType < SOLTYPE_RTK_PRE)          QNMEA = 3;
    else if (pStruct->m_SolType >= SOLTYPE_RTK_FLOFIX) QNMEA = 4;

    if (bGGA)
    {
        solLen = 0;
        memset(sol, 0, MAXSIZE);

        static bool bHead1 = true;
        if (bHead1)
        {
            solLen += sprintf(sol + solLen, "$ACCUR,1,12,0,%04d%02d%02d,,123206.00,,,,,,,,,,*", ymd.year, ymd.month, ymd.day);
            checkNum = NMEACRC(sol+1,solLen-2); // 校验和
            solLen += sprintf(sol + solLen, "%X\n",checkNum);
            bHead1 = false;
        }

        checkStart = solLen + 1; //起始下标
        checkEnd = solLen + 1; //结束下标

        solLen += sprintf(sol + solLen, "$GNGGA,");

        // 时间: 时分秒
        solLen += sprintf(sol + solLen, "%02d%02d%05.2f,", ymd.hour, ymd.min, ymd.sec);

        // 纬度
        deg = (int)(pStruct->m_LLHPos[0] * R2D);
        min = (pStruct->m_LLHPos[0] * R2D - (int)(pStruct->m_LLHPos[0] * R2D)) * 60;
        solLen += sprintf(sol + solLen, "%03d%09.6f,N,", deg, min);

        // 经度
        deg = (int)(pStruct->m_LLHPos[1] * R2D);
        min = (pStruct->m_LLHPos[1] * R2D - (int)(pStruct->m_LLHPos[1] * R2D)) * 60;
        solLen += sprintf(sol + solLen, "%03d%09.6f,E,", deg, min);

        // 解状态
        solLen += sprintf(sol + solLen, "%d,", QNMEA);

        // 卫星数
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_nsat);

        // HDOP
        solLen += sprintf(sol + solLen, "%3.1f,", HDOP);

        // 高程
        solLen += sprintf(sol + solLen, "%5.3f,M,", pStruct->m_LLHPos[2]);

        // 高程异常
        solLen += sprintf(sol + solLen, "%5.3f,M,", 0.0);

        solLen += sprintf(sol + solLen, ",,*");

        // 校验和：$与*之间所有字符ASCII码的校验和(各字节做异或运算，得到校验和后，再转换16进制格式的ASCII字符。)
        checkEnd = solLen - 2; //-2:*与下标
        checkNum = NMEACRC(sol+checkStart,checkEnd-checkStart+1);
        solLen += sprintf(sol + solLen, "%X\n",checkNum);

        memset(pStruct->m_SolGGA, 0, MAXSIZE);
        strcpy(pStruct->m_SolGGA, sol);
    }

    if (bRMC) // RMC推荐定位信息
    {
        solLen = 0;
        memset(sol, 0, MAXSIZE);

        checkStart = solLen + 1; //起始下标
        checkEnd = solLen + 1; //结束下标

        // UTC时间 hhmmss(时分秒)
        solLen += sprintf(sol + solLen, "$GNRMC,%02d%02d%05.2f,", ymd.hour, ymd.min, ymd.sec);

        // 定位状态, A:有效定位, V:无效定位
        if (PDOP > 15.0 || MatrixTrace(3, 3, pStruct->m_XYZPosP) > 900.0) solLen += sprintf(sol + solLen, "V,");
        else solLen += sprintf(sol + solLen, "A,");

        // 纬度 ddmm.mmmmmm 度分 
        deg = (int)(pStruct->m_LLHPos[0] * R2D);
        min = (pStruct->m_LLHPos[0] * R2D - (int)(pStruct->m_LLHPos[0] * R2D)) * 60;
        solLen += sprintf(sol + solLen, "%d%09.6f,N,", deg, min);

        // 经度 dddmm.mmmmmm 度分
        deg = (int)(pStruct->m_LLHPos[1] * R2D);
        min = (pStruct->m_LLHPos[1] * R2D - (int)(pStruct->m_LLHPos[1] * R2D)) * 60;
        solLen += sprintf(sol + solLen, "%d%09.6f,E,", deg, min);

        // 地面速率(000.0~999.9节，Knot，前导位数不足则补0)
        double Venu[3] = { 0.0 };
        Vxyz2enu(pStruct->m_LLHPos, pStruct->m_XYZVel, Venu);
        double V_Knot = sqrt(Venu[0] * Venu[0] + Venu[1] * Venu[1]) * 3.6 / 1.8518;
        solLen += sprintf(sol + solLen, "%05.1f,", V_Knot);

        // 地面航向(000.0~359.9度，以真北为参考基准，前导位数不足则补0)
        double Heading = R2D * atan2(Venu[0], Venu[1]);
        if (Heading < 0) Heading += 360.0;
        solLen += sprintf(sol + solLen, "%05.1f,", Heading);

        // UTC日期，ddmmyy(日月年)
        solLen += sprintf(sol + solLen, "%02d%02d%02d,", ymd.day, ymd.month, ymd.year % 100);

        // Magnetic Variation, 磁偏角(000.0~180.0度, 前导位数不足则补0) +方向E(东)或W(西)
        // 先默认为000.0
        solLen += sprintf(sol + solLen, "%05.1f,E,", 000.0);

        // Mode Indicator，模式指示(A:自主定位, D:差分, E:估算, N:数据无效)
        // 先默认为差分
        solLen += sprintf(sol + solLen, "D*");

        // 校验和
        checkEnd = solLen - 2; //-2:*与下标
        checkNum = NMEACRC(sol + checkStart, checkEnd - checkStart + 1);
        solLen += sprintf(sol + solLen, "%X\n", checkNum);

        memset(pStruct->m_SolRMC, 0, MAXSIZE);
        strcpy(pStruct->m_SolRMC, sol);
    }

    if(bLOG && pStruct->m_LOG.gt.GPSWeek > 100)
    {
        // $GNLOG,时间1,时间2,解状态,pStruct->m_nMeasSPP,pStruct->m_PosSolType,pStruct->m_VelSolType,pStruct->m_DisSolType,pStruct->m_SPVEKFSolType,pStruct->m_EFKSolType,dXZYType,nMeasP.nMeasL,
        solLen = 0;
        memset(sol, 0, MAXSIZE);

		static bool bHead2 = true;
		if (bHead2)
		{
			solLen += sprintf(sol + solLen, 
				"$GNLOG,Week,GPST(sec),HMS,nsat,QNMEA,Q,SPPnMeas,SPPSolType,SPVnMeas[0],SPVnMeas[1],SPVSolType,bReinitAmb[0],bReinitAmb[1],RTKnMeas[0],RTKnMeas[1],RTKSolType[0],RTKSolType[1],valAmbs,fixAmbs,fixRatio,fixPDOP,fixPosDis,fixAmbRms");
            checkNum = NMEACRC(sol + 1, solLen - 2); // 校验和
			solLen += sprintf(sol + solLen, "*%X\n", checkNum);
			bHead2 = false;
		}

        solLen += sprintf(sol + solLen, "$GNLOG,");

        // 01.时间: 时分秒、周内秒
        solLen += sprintf(sol + solLen, "%4d,", pStruct->m_LOG.gt.GPSWeek);
        solLen += sprintf(sol + solLen, "%8.2lf,", GetGPSTIMESow(pStruct->m_LOG.gt));
        solLen += sprintf(sol + solLen, "%02d%02d%05.2f,", ymd.hour, ymd.min, ymd.sec);

		// 04.卫星数
		solLen += sprintf(sol + solLen, "%d,", pStruct->m_LOG.nSat);

        // 02.解状态
        solLen += sprintf(sol + solLen, "%d,", QNMEA);
		solLen += sprintf(sol + solLen, "%d,", pStruct->m_SolType);

        // 05.SPP和SPV相关
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.SPPnMeas);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.SPPSolType);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.SPVnMeas[0]);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.SPVnMeas[1]);
        solLen += sprintf(sol + solLen, "%8.3f,", pStruct->m_LOG.SPVXYZ[0]);
        solLen += sprintf(sol + solLen, "%8.3f,", pStruct->m_LOG.SPVXYZ[1]);
        solLen += sprintf(sol + solLen, "%8.3f,", pStruct->m_LOG.SPVXYZ[2]);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.SPVSolType);

        for (int i = 0; i < NFREQ; i++)solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.bReinitAmb[i]);

        // 06.浮点解相关
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.RTKnMeas[0]);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.RTKnMeas[1]);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.RTKSolType[0]);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.RTKSolType[1]);

        // 07.模糊度固定相关
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.valAmbs);
        solLen += sprintf(sol + solLen, "%2d,", pStruct->m_LOG.fixAmbs);
        solLen += sprintf(sol + solLen, "%6.2f,", pStruct->m_LOG.fixRatio);
        solLen += sprintf(sol + solLen, "%6.2f,", pStruct->m_LOG.fixPDOP);
        solLen += sprintf(sol + solLen, "%6.2f,", pStruct->m_LOG.fixPosDif);
        solLen += sprintf(sol + solLen, "%6.2f,", pStruct->m_LOG.fixAmbRms);

        // PDOP
        solLen += sprintf(sol + solLen, "%6.2f,", pStruct->m_PDOP);

        // 08.校验和
        checkNum = NMEACRC(sol+1,solLen-2);
        solLen += sprintf(sol + solLen, "*%X\n",checkNum);

        memset(pStruct->m_SolLOG, 0, MAXSIZE);
        strcpy(pStruct->m_SolLOG, sol);
    }

    return true;
}













bool WriteSolution(CGNSSSolution* pStruct)
{
    if (!pStruct)return false;

    if (pStruct->m_PDOP > 999.0) pStruct->m_PDOP = 999.0;
    if (pStruct->m_DDOP > 999.0) pStruct->m_DDOP = 999.0;

    XYZ2LLH(pStruct->m_XYZPos, pStruct->m_LLHPos, 0);
    Rxyz2enu(pStruct->m_LLHPos, pStruct->m_Rel);
    M33XM31(pStruct->m_Rel, pStruct->m_XYZVel, pStruct->m_ENUVel);
    MatrixMultiply_HPHT(3, 3, pStruct->m_Rel, pStruct->m_XYZPosP, false, pStruct->m_ENUPosP);
    MatrixMultiply_HPHT(3, 3, pStruct->m_Rel, pStruct->m_XYZVelP, false, pStruct->m_ENUVelP);

    // 对结果进行检核,结果太差就不输出
    if (pStruct->m_SolType == SOLTYPE_NONE)return false;
    if (fabs(pStruct->m_XYZPos[0] * pStruct->m_XYZPos[1] * pStruct->m_XYZPos[2]) < 1.0)return false;
    if (pStruct->m_nsat < 5 || MatrixTrace(3, 3, pStruct->m_XYZPosP) > 900.0) return false;

    WriteNEMA(pStruct);

    return true;
}