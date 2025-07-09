#include "BaseMatrix.h"
#include "BaseMath.h"
#include "BaseCmnFunc.h"
#include "CSPPoint.h"
#include "GNSSCmnFunc.h"


///< Constructor function, to be called when defining
void InitCSPPoint(CSPPoint* pStruct)
{
    if (!pStruct)return;

    // 外部初始化信息
    pStruct->m_pOBSData = NULL;
    pStruct->m_pEphComputer = NULL;
    pStruct->m_GLS = NULL;
    pStruct->m_pSatStatus = NULL;
    pStruct->m_pGNSSOpt = NULL;
    pStruct->m_pGNSSSol = NULL;
    pStruct->m_pGLSInfo = NULL;

    // 解算方法开关
    pStruct->m_bEstDis = true;
    pStruct->m_bUseCons = false;

    // 估计器
    InitCILSEstimator(&pStruct->m_ILS);
    InitCLSEstimator(&pStruct->m_LSQ);
    pStruct->m_nState = 0;
    pStruct->m_nMeas = 0;

    // SPP
    pStruct->m_SPPSolType = GNSSMODE_NONE;
    M31Zero(pStruct->m_SPPXYZ);
    M33Zero(pStruct->m_SPPXYZP);
    M31Zero(pStruct->m_SPPLLH);
    SetValsB(pStruct->m_bSPPClk, false, NSYS * NFREQ);
    SetValsD(pStruct->m_SPPClk, 0.0, NSYS * NFREQ);

    // SPV
    pStruct->m_SPVSolType = GNSSMODE_NONE;
    M31Zero(pStruct->m_SPVXYZ);
    M33Zero(pStruct->m_SPVXYZP);
    SetValsD(pStruct->m_SPVClkVel, 0.0, NSYS);

    // TDCP
    pStruct->m_DisSolType = GNSSMODE_NONE;
    M31Zero(pStruct->m_DisXYZ);
    M33Zero(pStruct->m_DisXYZP);
    pStruct->m_DisClkVel = 0.0;
    pStruct->m_DisICS = 4;

    // SPP EKF
    pStruct->m_SPPEKFSolType = -1;
    pStruct->m_nSPPEKFPredict = 0;

    // VA EKF
    pStruct->m_VAEKFSolType = -1;
    pStruct->m_nVAEKFPredict = 0;
    pStruct->m_bVAISBRW = false;
    pStruct->m_bVAUseL = true;
    pStruct->m_bVAUseD = true;
    pStruct->m_bVAEstAcc = true;
    M31Zero(pStruct->m_VAVel);
    M33Zero(pStruct->m_VAVelP);
    pStruct->m_VAResiThres = 1.2;
    pStruct->m_VAResiThresStaL = 3;
    pStruct->m_VAResiThresStaD = 3;
    pStruct->m_nVAState = 0;

    // 解算中间参数
    InitCIonosphere(&pStruct->m_IonoModel);
    InitCTroposphere(&pStruct->m_TropModel);
    InitGPSTIME(&pStruct->m_OBSTime);
    InitGPSTIME(&pStruct->m_SolTime);
    InitGPSTIME(&pStruct->m_PriorPosRTKTime);
    pStruct->m_PriorPosSigma = 999.9;
    M31Zero(pStruct->m_PriorPosRTK);
    pStruct->m_GapTime = 1.0;
    pStruct->m_bGoodEnvState = true;

    // 观测值与星历备份
    pStruct->m_jPosSolType = GNSSMODE_NONE;
    SetValsD(pStruct->m_azel[0], 0.0, NSATMAX + 1);
    SetValsD(pStruct->m_azel[1], 0.0, NSATMAX + 1);
    SetValsD(pStruct->m_jSPPClk, 0.0, NSYS);
    SetValsI(pStruct->m_jOBSIndex, -1, MAXOBS);
    M31Zero(pStruct->m_jSPPXYZ);
    for (int i = 0; i < NSATMAX + 1; i++) InitGPSTIME(pStruct->m_jEphtoe + i);

    pStruct->m_CSEstFp = NULL;
}


static void ClearSPP(CSPPoint* pStruct)
{
    if (!pStruct)return;

    if (pStruct->m_pGNSSSol) pStruct->m_pGNSSSol->m_SolType = GNSSMODE_NONE;

    pStruct->m_SPPSolType = GNSSMODE_NONE;
    pStruct->m_SPVSolType = GNSSMODE_NONE;
    pStruct->m_DisSolType = GNSSMODE_NONE;
    pStruct->m_nState = 0;
    M31Zero(pStruct->m_SPPXYZ);
    M33Zero(pStruct->m_SPPXYZP);
    M31Zero(pStruct->m_SPPLLH);
    M31Zero(pStruct->m_SPVXYZ);
    M33Zero(pStruct->m_SPVXYZP);
    SetValsD(pStruct->m_SPPClk, 0.0, NSYS * NFREQ);
    SetValsD(pStruct->m_SPVClkVel, 0.0, NSYS);
    SetValsD(pStruct->m_SPPDOPs, 0.0, 4);
    SetValsD(pStruct->m_DisDOPs, 0.0, 4);
    SetValsD(pStruct->m_SPVDOPs, 0.0, 4);

    for (int prn = 0; prn <= NSATMAX; prn++)
    {
        pStruct->m_bDisUsedPrn[prn] = true;
        //pStruct->m_SatStatus[prn].azel[0] = pStruct->m_SatStatus[prn].azel[1] = 0.0;
        for (int f = 0; f < NFREQ; f++)
        {
            if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;
            pStruct->m_pSatStatus[prn].OBSUsed[f] = true;
            for (int i = 0; i < 3; i++)
            {
                pStruct->m_pSatStatus[prn].OBSValid[i][f] = false;
                pStruct->m_pSatStatus[prn].OBSEvent[i][f] = OBSEVT_NONE;
                pStruct->m_pSatStatus[prn].OBSConsist[i][f] = 0.0;
            }
        }
    }
}


bool InitSPP(CSPPoint* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, CEphemerisComputer* pEphComputer, SATSTATUS* pSatStatus, CGLSEstimator* pGLSEst)
{
    if (!pStruct)return false;

    pStruct->m_pSatStatus = pSatStatus;
    pStruct->m_pGNSSOpt = pGNSSOpt;
    pStruct->m_pGNSSSol = pGNSSSol;
    pStruct->m_pEphComputer = pEphComputer;
    pStruct->m_GLS = pGLSEst;

    if (initSatStatus(pStruct->m_pSatStatus, NULL) == false)return false;

    ClearSPP(pStruct);

    return true;
}


static void MatchOBSIndex(CSPPoint* pStruct)
{
    if (!pStruct)return;

    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        pStruct->m_jOBSIndex[i] = -1;

        for (int j = 0; j < pStruct->m_jObsData.nsat; j++)
        {
            if (pStruct->m_pOBSData->obs[i].prn == pStruct->m_jObsData.obs[j].prn)
            {
                pStruct->m_jOBSIndex[i] = j;
                break;
            }
        }
    }
}


static void getSatelliteOrbits(CSPPoint* pStruct)
{
    if (!pStruct)return;

    EPH_TYPE SPPEph = EPH_BRDC;
    int sys = 0, prn = 0, sat = 0, isys = 0;
    bool bEphChange = false;
    double Pr = 0.0;
    OBS_DATA_t* pObsDatai = NULL;
    GPSTIME satTime, toe1, toe2;

    // 计算卫星位置
    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        pObsDatai = &(pStruct->m_pOBSData->obs[i]);
        prn = pObsDatai->prn;
        sat = satprn2no(prn, &sys);
        isys = pStruct->m_pSatStatus[prn].sys_id;
        pStruct->m_pSatStatus[prn].Satsvh = -1;

        if (!(sys & pStruct->m_pGNSSOpt->m_SatSYS)) continue;
        if (pStruct->m_pEphComputer->m_bNavEph[isys] == false) continue;

        for (int f = 0; f < NFREQ; f++)
        {
            if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;
            if ((Pr = pObsDatai->P[f]) != 0.0) break;
        }

        if (IsEqual(Pr, 0.0)) continue;

        // 得到卫星信号发射时刻,计算卫星位置, 此处只要误差总和小于100m, 引起的卫星准确发射时刻小于1us, 卫星位置精度小于1mm
        satTime = MinusGPSTIMESec(pStruct->m_OBSTime, (Pr / CLIGHT));

        // PPP模式下,使用精密星历和精密钟差,保证SPP和PPP的基准一致,SPP预报的钟差就能用到PPP中去,否则可能出现不兼容情况
        if (pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_KINEMA || pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_STATIC || pStruct->m_pGNSSOpt->m_PMode == GNSSMODE_RTK_MOVEB)
        {
            SPPEph = EPH_BRDC;
        }

        if (SPPEph == EPH_PRE || pStruct->m_bEstDis == false)
        {
            if (EphComputer(pStruct->m_pEphComputer, satTime, prn, SPPEph) == false)continue;
        }
        else
        {
            if (EphComputer(pStruct->m_pEphComputer, satTime, prn, SPPEph) == false)continue;
            switch (sys) 
            {
            case SYSGPS: {toe1 = pStruct->m_pEphComputer->m_GPSEph.toe; break; }
            case SYSGLO: {toe1 = pStruct->m_pEphComputer->m_GLOEph.toe; break; }
            case SYSBD2: {toe1 = pStruct->m_pEphComputer->m_BD2Eph.toe; break; }
            case SYSBD3: {toe1 = pStruct->m_pEphComputer->m_BD3Eph.toe; break; }
            case SYSGAL: {toe1 = pStruct->m_pEphComputer->m_GALEph.toe; break; }
            case SYSQZS: {toe1 = pStruct->m_pEphComputer->m_QZSEph.toe; break; }
            default: continue;
            }
            toe2 = pStruct->m_jEphtoe[prn];
            pStruct->m_jEphtoe[prn] = toe1;
            bEphChange = (pStruct->m_jOBSIndex[i] >= 0 && !IsGPSTIMEEqual(toe1, toe2)); // 前一历元有数据，且星历参考时刻不同，说明星历切换
            if (bEphChange)pStruct->m_bDisUsedPrn[prn] = false;
        }

        M31EQU(pStruct->m_pEphComputer->m_SatPos, pStruct->m_pSatStatus[prn].SatPos);
        M31EQU(pStruct->m_pEphComputer->m_SatVel, pStruct->m_pSatStatus[prn].SatVel);
        pStruct->m_pSatStatus[prn].SatClk = pStruct->m_pEphComputer->m_SatClk;
        pStruct->m_pSatStatus[prn].SatClkVel = pStruct->m_pEphComputer->m_SatClkVel;
        pStruct->m_pSatStatus[prn].Satsvh = pStruct->m_pEphComputer->m_svh;
    }
}


static void CheckOBSConsist(CSPPoint* pStruct)
{
    if (!pStruct)return;

    if (pStruct->m_GapTime > 3.0) return;

    const double dt = pStruct->m_GapTime;
    int i = 0, j = 0, prn = 0, nP = 0, nL = 0;
    double lam = 0.0, dop = 0.0, OBSConsist = 0.0;
    double dClockJumpValueP[NSATMAX * NFREQ], dClockJumpValueL[NSATMAX * NFREQ], dClockJumpP, dClockJumpL; // 钟跳值序列
    double DCur, DPre, lamCur; // 用于检验的多普勒

    for (int f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;

        nP = nL = 0;
        dClockJumpP = dClockJumpL = 0.0;

         // 1.观测值历元差与多普勒比较
        for (i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            if ((j = pStruct->m_jOBSIndex[i]) < 0) continue;

            prn = pStruct->m_pOBSData->obs[i].prn;
            lam = pStruct->m_pSatStatus[prn].lam[f];

            // TH仅L1频段多普勒有效，M66Lite同一频点多普勒会出现相似偏差、造成判断错误
            DCur = pStruct->m_pOBSData->obs[i].D[0];
            DPre = pStruct->m_jObsData.obs[j].D[0];
            lamCur = pStruct->m_pSatStatus[prn].lam[0];
            if (DCur == 0.0 || DPre == 0.0 || lamCur == 0.0 || lam == 0.0)continue;

            OBSConsist = (DCur - DPre) * lamCur;
            pStruct->m_pSatStatus[prn].OBSConsist[OBS_DI][f] = OBSConsist;
            if (fabs(OBSConsist) > pStruct->m_pGNSSOpt->m_DDOPThres) continue; // 多普勒前后历元较差，阈值20m/s
            dop = (DCur + DPre) * lamCur / 2.0;

             //伪距历元间变化率与多普勒进行比较, 阈值5.0m/s
            if (pStruct->m_pOBSData->obs[i].P[f] != 0.0 && pStruct->m_jObsData.obs[j].P[f] != 0.0)
            {
                OBSConsist = dop * dt + (pStruct->m_pOBSData->obs[i].P[f] - pStruct->m_jObsData.obs[j].P[f]);
                dClockJumpValueP[nP++] = OBSConsist;
                pStruct->m_pSatStatus[prn].OBSConsist[OBS_PI][f] = OBSConsist;
            }

             //载波历元间变化率与多普勒进行比较，阈值0.5m/s
            if (pStruct->m_pOBSData->obs[i].L[f] != 0.0 && pStruct->m_jObsData.obs[j].L[f] != 0.0)
            {
                OBSConsist = dop * dt + (pStruct->m_pOBSData->obs[i].L[f] - pStruct->m_jObsData.obs[j].L[f]) * lam;
                dClockJumpValueL[nL++] = OBSConsist;
                pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f] = OBSConsist;
            }
        }

         // 2.钟跳调整，分别用历元差观测值的中位数标记伪距和相位的钟跳
        SortArr(dClockJumpValueP, nP, true);
        if (nP > 0)dClockJumpP = dClockJumpValueP[nP / 2];
        SortArr(dClockJumpValueL, nL, true);
        if (nL > 0)dClockJumpL = dClockJumpValueL[nL / 2];

        for (i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            prn = pStruct->m_pOBSData->obs[i].prn;

            if (pStruct->m_pSatStatus[prn].OBSConsist[OBS_PI][f] != 0.0)
            {
                if (fabs(dClockJumpP) > 1E-4 * CLIGHT) pStruct->m_pSatStatus[prn].OBSConsist[OBS_PI][f] -= dClockJumpP;
            }

            if (pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f] != 0.0)
            {
                if (fabs(dClockJumpL) > 1E-4 * CLIGHT) pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f] -= dClockJumpL;
            }
        }

         // 3.以多普勒为基准探测伪距大粗差
        int nsat[3] = { 0 }, nrei[3] = { 0 };
        for (i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            if ((j = pStruct->m_jOBSIndex[i]) < 0) continue;

            prn = pStruct->m_pOBSData->obs[i].prn;

            if (pStruct->m_pOBSData->obs[i].P[f] != 0.0 && pStruct->m_jObsData.obs[j].P[f] != 0.0)nsat[OBS_PI]++;
            if (fabs(pStruct->m_pSatStatus[prn].OBSConsist[OBS_PI][f]) > pStruct->m_pGNSSOpt->m_DCodeThres)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_XDOP;
                nrei[OBS_PI]++;
            }

            if (pStruct->m_pOBSData->obs[i].L[f] != 0.0 && pStruct->m_jObsData.obs[j].L[f] != 0.0)nsat[OBS_LI]++;
            if (fabs(pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f]) > pStruct->m_pGNSSOpt->m_DPhaseThres)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_XDOP;
                nrei[OBS_LI]++;

                // 多频采用同一多普勒进行检验，双频相位均异常且量级相同，大概率是多普勒出现问题（其他接收机也是）
                if (/*(pStruct->m_pGNSSOpt->m_GNSSDeviceType == GDT_LowCost_TH1030) &&*/ f != 0)
                {
                    double consL[NFREQ], mean = 0.0, dif = 0.0;
                    int iconsL = 0;
                    for (int f1 = 0; f1 <= f; f1++)
                    {
                        if (pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f1] != 0.0)
                            consL[iconsL++] = pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f1];
                    }
                    if (iconsL > 1)
                    {
                        Mean_RMS_Var(consL, iconsL, &mean, NULL, NULL, false);
                        for (int f1 = 0; f1 <= f; f1++)
                        {
                            if (pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f1] & OBSEVT_XDOP)
                            {
                                dif = pStruct->m_pSatStatus[prn].OBSConsist[OBS_LI][f1] - mean;
                                if (fabs(dif) < 0.10)
                                    pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f1] &= ~(OBSEVT_XDOP);
                            }
                        }
                    }
                }
            }

            if (pStruct->m_pOBSData->obs[i].D[f] != 0.0 && pStruct->m_jObsData.obs[j].D[f] != 0.0)nsat[OBS_DI]++;
            if (fabs(pStruct->m_pSatStatus[prn].OBSConsist[OBS_DI][f]) > pStruct->m_pGNSSOpt->m_DDOPThres)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_DI][f] |= OBSEVT_XDOP;
                nrei[OBS_DI]++;
            }
        }

         // 4.未通过检验数太多，则认为多普勒有问题
        for (i = 0; i < 3; i++)
        {
            if ((nrei[i] * 2 + 1 > nsat[i]) && nrei[i] >= 2)
            {
                for (j = 0; j < pStruct->m_pOBSData->nsat; j++)
                {
                    prn = pStruct->m_pOBSData->obs[j].prn;
                    pStruct->m_pSatStatus[prn].OBSEvent[i][f] &= ~(OBSEVT_XDOP);
                }
            }
        }
    }
}


static bool PrepareSPP(CSPPoint* pStruct)
{
    if (!pStruct)return false;

    // 1. 清空SPP
    ClearSPP(pStruct);

    // 补充1：调整多普勒符号
    static int dopType = 0; // 0 = 初始化, 1 = 正常负号, 2 = 取相反符号
    if (dopType == 0)
    {
        int i = 0, j = 0;
        if ((j = pStruct->m_jOBSIndex[i]) >= 0)
        {
            int prn = pStruct->m_pOBSData->obs[i].prn;
            double dCur = pStruct->m_pOBSData->obs[i].D[0];
            double dPre = pStruct->m_jObsData.obs[j].D[0];
            double lamCur = pStruct->m_pSatStatus[prn].lam[0];
            if (dCur != 0.0 && dPre != 0.0 && lamCur > 0.0)
            {
                double cons1 = (dCur + dPre) * lamCur / 2.0 * pStruct->m_GapTime + (pStruct->m_pOBSData->obs[i].P[0] - pStruct->m_jObsData.obs[j].P[0]);
                double cons2 = -(dCur + dPre) * lamCur / 2.0 * pStruct->m_GapTime + (pStruct->m_pOBSData->obs[i].P[0] - pStruct->m_jObsData.obs[j].P[0]);
                if (fabs(cons1) < pStruct->m_pGNSSOpt->m_DCodeThres) dopType = 1;
                if (fabs(cons2) < pStruct->m_pGNSSOpt->m_DCodeThres) dopType = 2;
            }
        }
    }
    if (dopType == 2)
    {
        for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            for (int f = 0; f < NFREQ; f++) pStruct->m_pOBSData->obs[i].D[f] = -pStruct->m_pOBSData->obs[i].D[f];
        }
    }

    // 2. 获取时间和先验坐标
    if (pStruct->m_jObsData.gt.GPSWeek > 10) pStruct->m_GapTime = fabs(MinusGPSTIME(pStruct->m_pOBSData->gt, pStruct->m_jObsData.gt));
    if (pStruct->m_PriorPosSigma > 999.0 && pStruct->m_PriorPosRTK[0] != 0.0
        && fabs(MinusGPSTIME(pStruct->m_PriorPosRTKTime, pStruct->m_OBSTime)) < 5.0)
    {
        pStruct->m_PriorPosSigma = 25;
        SetValsD(pStruct->m_PriorPosClk, 0.0, MAXSPPNX);
        M31EQU(pStruct->m_PriorPosRTK, pStruct->m_PriorPosClk);
    }

    // 4. 匹配前后历元观测值索引
    MatchOBSIndex(pStruct);

    // 5. 卫星轨道钟差计算
    getSatelliteOrbits(pStruct);

    // 6. 数据一致性检验
    CheckOBSConsist(pStruct);

    // 7. 依据高度角和信噪比选星
    //SelectSatellite(pStruct);

    return true;
}


static double Prange(OBS_DATA_t* pObsDatai, int prn, int freq, double *var)
{
    double tgd = 0.0;
    //double gamma = 0.0;
    //int sys = SYSNON;

    //var = 0.0;
    //satprn2no(prn, &sys);

    //if (pObsDatai->P[freq] == 0.0 || freq >= 2) return 0.0;

    //if (sys == SYSGPS || sys == SYSQZS)
    //{
    //    gamma = SQR(FREQ1_GPS) / SQR(FREQ2_GPS);
    //    tgd = m_EphComputer.getTGD(m_OBSTime, prn, 0); // L1-L2
    //    if (freq == 1)tgd = 0.0; // 该程序中，频点二处为L5频点，广播星历中没有相应的改正参数，因此不改
    //}
    //else if (sys == SYSGAL)
    //{
    //    gamma = SQR(FREQ1_GAL) / SQR(FREQ2_GAL);
    //    tgd = m_EphComputer.getTGD(m_OBSTime, prn, 0); // E1-E5a
    //    if (freq == 1)tgd = tgd * gamma;
    //}
    //else if (sys == SYSBD2 || sys == SYSBD3)
    //{
    //    tgd = m_EphComputer.getTGD(m_OBSTime, prn, 0);
    //    if (freq == 1)tgd = 0.0;
    //}

    return (pObsDatai->P[freq] - tgd);

    //// L1与L5频点组合，可以暂时先不改TGD
    //return pObsDatai->P[freq];
}


static double Variance(int prn, int frq, double el, float S, int OBSType)
{
    int sys = -1;
    satprn2no(prn, &sys);

    double fact = 1.0;
    switch (OBSType)
    {
    case OBS_PI: {fact = 1.0; break; }
    case OBS_LI: {fact = 0.01; break; }
    case OBS_DI: {fact = 0.1; break; }
    default:fact = 1.0;
    }

    if (S > 0.0)
    {
        // 以伪距为基准
        double a = 637.4471, b = 0.1583, c = 0.1403;

        switch (sys)
        {
        case SYSGPS: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
        case SYSGLO: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
        case SYSBD2: {a = 84.8071;  b = 0.1288; c = 0.3858; break; }
        case SYSBD3: {a = 84.8071;  b = 0.1288; c = 0.3858; break; }
        case SYSGAL: {a = 383.3415; b = 0.1532; c = 0.3231; break; }
        case SYSQZS: {a = 637.4471; b = 0.1583; c = 0.1403; break; }
        }

        //这个方差基本接近真实方差
        double sigma = fact * (a * exp(-b * S) + c);
        if (OBSType == OBS_PI && frq == 2) sigma = sigma / 1.5; // L5波段伪距更好
        return (sigma * sigma);
    }
    else
    {
        // 以相位为基准
        double PLR2 = 1.0 / (fact * fact);
        double Error_a2 = 9e-6;
        double Error_b2 = 9e-6;

        double varr = fact * fact * PLR2 * (Error_a2 + Error_b2 / (sin(el) * sin(el)));

        return varr;
    }
}


static bool setMEQ_Pos(CSPPoint* pStruct, const int iter)
{
    if (!pStruct)return false;

    SetValsD(pStruct->m_B, 0, MAXOBSLIM * MAXLSQNX);
    SetValsD(pStruct->m_V, 0, MAXOBSLIM);
    SetValsD(pStruct->m_R, 0, MAXOBSLIM * MAXOBSLIM);
    pStruct->m_nMeas = 0;
    pStruct->m_nState = 0;

    for (int i = 0; i < NSYS* NFREQ; i++)
    {
        pStruct->m_bSPPClk[i] = false;
    }

    int prn = 0, f = 0, isys = 0;
    double lam = 0.0, dist = 0.0, rsVec[3] = { 0.0 }, azel[2] = { 0.0 };
    double P = 0.0, v = 0.0, varP = 0.0, var = 0.0;

    XYZ2LLH(pStruct->m_SPPXYZ, pStruct->m_SPPLLH, 0);

    for (f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;

        for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            OBS_DATA_t* pObsDatai = &(pStruct->m_pOBSData->obs[i]);
            prn = pObsDatai->prn;
            lam = pStruct->m_pSatStatus[prn].lam[f];
            isys = pStruct->m_pSatStatus[prn].sys_id;

            // 选星剔除
            if (!pStruct->m_pSatStatus[prn].OBSUsed[f]) continue;

            // 必须放在选星后，否则m_nMeas可能越界
            if (iter == 0)
            {
                pStruct->m_Resi[pStruct->m_nMeas] = pStruct->m_ResiSig[pStruct->m_nMeas] = 0.0;
            }

            // 排除不健康的伪距观测值
            if (pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] >= OBSEVT_Exclude) continue;

            if (pStruct->m_pOBSData->obs[i].S[f] > 0.0 && pStruct->m_pOBSData->obs[i].S[f] < pStruct->m_pGNSSOpt->m_SNRThres[f])
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_SNR; continue;
            }

            // 观测值一致性检验未通过的不参与检验，已通过OBSEvent剔除

            if (pStruct->m_pSatStatus[prn].Satsvh != 0 || (lam == 0.0))
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_SSVH; continue;
            }

            if ((P = Prange(pObsDatai, prn, f, &varP)) == 0.0 || P < 1E7)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_XMEQ; continue;
            }

            if ((dist = geodist(pStruct->m_pSatStatus[prn].SatPos, pStruct->m_SPPXYZ, rsVec)) <= 0.0)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_XMEQ; continue;
            }

            if (satazel(pStruct->m_SPPLLH, rsVec, azel) < pStruct->m_pGNSSOpt->m_ElevMask)
            {
                // 高度角会随着位置迭代而变化,所以只在验后进行赋值
                if (iter == 0) pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_ELEV;
                continue;
            }

            if (IonoComputer(&pStruct->m_IonoModel, pStruct->m_OBSTime, pStruct->m_SPPLLH, azel, lam, IONO_KLOB, pStruct->m_pEphComputer->m_GPSION) == false)
            {
                if (iter == 0) pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_XMEQ;
                continue;
            }

            if (TropComputer(&pStruct->m_TropModel, pStruct->m_OBSTime, pStruct->m_SPPLLH, azel[1], true, TROP_UM, TROP_SMF, TROP_NONE) == false)
            {
                if (iter == 0) pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_XMEQ;
                continue;
            }

            v = P - (dist - CLIGHT * pStruct->m_pSatStatus[prn].SatClk + pStruct->m_IonoModel.m_STD + pStruct->m_TropModel.m_STD); // 未加入测站钟差

            // 钟差改正
            pStruct->m_B[pStruct->m_nMeas * MAXLSQNX + 3 + isys * NFREQ + f] = 1.0;
            v -= pStruct->m_SPPClk[isys * NFREQ + f];
            pStruct->m_bSPPClk[isys * NFREQ + f] = true;

            for (int j = 0; j < 3; j++)
            {
                pStruct->m_B[pStruct->m_nMeas * MAXLSQNX + j] = -rsVec[j];
            }

            var = Variance(prn, f, azel[1], pObsDatai->S[f], OBS_PI);

            if (iter > 0)
            {
                pStruct->m_V[pStruct->m_nMeas] = v;
                pStruct->m_R[pStruct->m_nMeas * MAXOBSLIM + pStruct->m_nMeas] = 1.0 / var; // p->m_R.x赋值给LSQ.p->m_P.x，权
            }
            else
            {
                pStruct->m_Resi[pStruct->m_nMeas] = v;
                pStruct->m_ResiSig[pStruct->m_nMeas] = var;
            }

            pStruct->m_azel[0][prn] = azel[0];
            pStruct->m_azel[1][prn] = azel[1];
            pStruct->m_pSatStatus[prn].azel[0] = azel[0];
            pStruct->m_pSatStatus[prn].azel[1] = azel[1];
            pStruct->m_OBSUsed[pStruct->m_nMeas].prn = prn;
            pStruct->m_OBSUsed[pStruct->m_nMeas].f = f;
            pStruct->m_nMeas++;
        }
    }

    // 观测值个数小于状态参数个数
    pStruct->m_nState = 3; // pos
    for (int i = 0; i < NSYS* NFREQ; i++)
    {
        if (pStruct->m_bSPPClk[i]) pStruct->m_nState++;
    }

    // 计算验后残差直接返回
    if (iter == 0) return true;

    if (pStruct->m_nMeas < pStruct->m_nState)
    {
        return false;
    }

    // 默认最大处理SPP_NY个观测值
    if (pStruct->m_nMeas > MAXOBSLIM)
    {
        return false;
    }

    // 形成LSQ方程
    InitLSQ(&pStruct->m_LSQ, pStruct->m_nState, pStruct->m_nMeas, true);

    for (int i = 0; i < pStruct->m_nMeas; i++)
    {
        int k = 0;
        for (int j = 0; j < MAXLSQNX; j++)
        {
            if (j >= 3 && pStruct->m_bSPPClk[j - 3] == false) continue;
            pStruct->m_LSQ.m_B[i * pStruct->m_LSQ.m_nState + k] = pStruct->m_B[i * MAXLSQNX + j]; k++;
        }

        pStruct->m_LSQ.m_L[i] = pStruct->m_V[i];
        for (int j = 0; j < pStruct->m_nMeas; j++) pStruct->m_LSQ.m_P[pStruct->m_LSQ.m_nMeas * i + j] = pStruct->m_R[i * MAXOBSLIM + j];
    }

    return true;
}


static bool Validate_Pos(CSPPoint* pStruct)
{
    if (!pStruct)return false;

    bool bGross = false;

    // 验后RMS检验
    if (pStruct->m_SPPLSQRMS > 5.0) bGross = true;

    // 验后残差检验
    int k = 0;
    setMEQ_Pos(pStruct, 0);
    getMaxPos(pStruct->m_Resi, pStruct->m_nMeas, &k, true);
    if (fabs(pStruct->m_Resi[k]) > 5.0) bGross = true;

    if (bGross)
    {
        const int prn = pStruct->m_OBSUsed[k].prn;
        const int f = pStruct->m_OBSUsed[k].f;
        pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] |= OBSEVT_Resi_v;	// 验后粗差
        return false;
    }

    return true;
}


static int LSQ_Pos(CSPPoint* pStruct)
{
    if (!pStruct)return GNSSMODE_NONE;

    int iSuc = GNSSMODE_NONE;

    SetValsD(pStruct->m_azel[0], 0.0, NSATMAX + 1);
    SetValsD(pStruct->m_azel[1], 0.0, NSATMAX + 1);

    ClearLSQ(&pStruct->m_LSQ);

    // Pos抗差迭代
    for (int iMLS = 0, nMeas = 0; (iMLS < nMeas / 2 && nMeas > 0) || nMeas == 0; iMLS++)
    {
        if (pStruct->m_PriorPosSigma > 999.0)
        {
            M31Zero(pStruct->m_SPPXYZ);
            SetValsD(pStruct->m_SPPClk, 0.0, NSYS * NFREQ);
        }
        else
        {
            M31EQU(pStruct->m_PriorPosClk, pStruct->m_SPPXYZ);
            for (int i = 0; i < NSYS * NFREQ; i++)
            {
                pStruct->m_SPPClk[i] = pStruct->m_PriorPosClk[3 + i];
            }
        }
        M33Zero(pStruct->m_SPPXYZP);

        bool flag = true;

        // 1. 迭代SPP
        for (int it = 1; it < 10; it++)
        {
            if (setMEQ_Pos(pStruct, it) == false) { flag = false; break; }
            if (nMeas == 0) nMeas = pStruct->m_nMeas;
            if (LSQ(&pStruct->m_LSQ, false) == false) { flag = false; break; }

            M31M312(pStruct->m_LSQ.m_StateX, pStruct->m_SPPXYZ, 1.0, 1.0);

            int k = 3;
            for (int i = 0; i < NSYS* NFREQ; i++)
            {
                if (pStruct->m_bSPPClk[i]) pStruct->m_SPPClk[i] += pStruct->m_LSQ.m_StateX[k++];
            }

            if (MatrixNorm2(3, 1, pStruct->m_LSQ.m_StateX) < 1E-2)
            {
                LSQ(&pStruct->m_LSQ, true); // 验后方差估计
                pStruct->m_SPPLSQRMS = pStruct->m_LSQ.m_RMS;
                flag = (pStruct->m_SPPLSQRMS == 0.0) ? false : true;
                break;
            }
        }
        if (flag == false) break; // 迭代未收敛

        // 2. 检验SPP结果,失败再次抗差迭代
        if (/*Validate_Pos(pStruct)*/1)
        {
            iSuc = GNSSMODE_SPP;
            pStruct->m_SPPnMeas = pStruct->m_nMeas;
            break;
        }
    }

    if (iSuc == GNSSMODE_SPP)
    {
        int s1[2] = { 0 }; int s2[2] = { 0, 0 };
        MatrixCopySub(s1, s2, 3, 3, pStruct->m_LSQ.m_nMeas, pStruct->m_LSQ.m_nState, pStruct->m_LSQ.m_StateP, 3, 3, pStruct->m_SPPXYZP, false);
        XYZ2LLH(pStruct->m_SPPXYZ, pStruct->m_SPPLLH, 0);

        for (int i = 0; i < NSYS * NFREQ; i++)
        {
            if (!pStruct->m_bSPPClk[i]) pStruct->m_SPPClk[i] = 0.0;
        }

        ComputeDOP(pStruct->m_azel[0], pStruct->m_azel[1], pStruct->m_pGNSSOpt->m_ElevMask, pStruct->m_SPPDOPs, NULL, false, false);
    
        if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
        {
            pStruct->m_pGNSSSol->m_LOG.SPPnMeas = pStruct->m_SPPnMeas;
            pStruct->m_pGNSSSol->m_LOG.SPPSolType = GNSSMODE_SPP;
        }
    }
    else
    {
        M33Zero(pStruct->m_SPPXYZP);
        M31Zero(pStruct->m_SPPXYZ);
        M31Zero(pStruct->m_SPPLLH);
        SetValsD(pStruct->m_SPPClk, 0.0, NSYS * NFREQ);
        SetValsB(pStruct->m_bSPPClk, false, NSYS * NFREQ);
    }

    return iSuc;
}



/** @brief  Obtain observation equations of TDCP. */
static bool setMEQ_Dis(CSPPoint* pStruct, const int iter)
{
    if (!pStruct) return false;

    double lam = 0.0, ddist = 0.0, dg = 0.0, dD = 0.0, rsVec1[3] = { 0.0 }, rsVec2[3] = { 0.0 }, azel[2] = { 0.0 };
    double v = 0.0, var = 0.0, M1[3] = { 0.0 }, M2[3] = { 0.0 }, sagnac1 = 0.0, sagnac2 = 0.0;
    double RcvPos1[3], RcvPos2[3], RcvLLH2[3] = { 0.0 }; // 前一时刻和当前时刻位置
    int prn = 0, f = 0;

    // 位置初值
    if (pStruct->m_PriorPosSigma < 999.0) M31EQU(pStruct->m_PriorPosClk, RcvPos1);
    else return false;
    M31M311(RcvPos1, pStruct->m_DisXYZ, RcvPos2, 1.0, 1.0);
    XYZ2LLH(RcvPos2, RcvLLH2, 0);

    // 初始化
    SetValsD(pStruct->m_B, 0.0, MAXOBSLIM * MAXDISNX);
    SetValsD(pStruct->m_V, 0.0, MAXOBSLIM);
    SetValsD(pStruct->m_R, 0.0, MAXOBSLIM * MAXOBSLIM);
    pStruct->m_nMeas = 0;
    pStruct->m_nState = 4;

    for (f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;

        for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            OBS_DATA_t* pObsDatai = &(pStruct->m_pOBSData->obs[i]);
            prn = pStruct->m_pOBSData->obs[i].prn;
            lam = pStruct->m_pSatStatus[prn].lam[f];
            const int j = pStruct->m_jOBSIndex[i];

            // 前一时刻不存在观测
            if (j < 0) continue;
            OBS_DATA_t* pObsDataj = &(pStruct->m_jObsData.obs[j]);

            // 判断是否为选取卫星
            if (!pStruct->m_pSatStatus[prn].OBSUsed[f]) continue;

            // 必须放在选星后，否则m_nMeas可能越界
            if (iter == 0)
            {
                pStruct->m_Resi[pStruct->m_nMeas] = pStruct->m_ResiSig[pStruct->m_nMeas] = 0.0;
            }

            // 判断是否星历切换
            if (pStruct->m_bDisUsedPrn[prn] == false) continue;

            // GF组合探测多频周跳，LLI标识探测单频周跳
            //if (!pStruct->m_pGNSSOpt->m_bRCS_GB && pStruct->m_pSatStatus[prn].CS_Type[f]) continue;

            // 判断观测值是否健康
            //if (!pStruct->m_pGNSSOpt->m_bRCS_GB && pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] >= OBSEVT_Exclude) continue;

            // 判断卫星健康状态
            if (pStruct->m_pSatStatus[prn].Satsvh != 0 || pStruct->m_jSatStatus_Satsvh[prn] != 0 || lam == 0.0)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_SSVH;
                continue;
            }

            // 判断观测值是否为空
            if (pObsDatai->L[f] == 0.0 || pObsDataj->L[f] == 0.0)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_XMEQ;
                continue;
            }

            // 计算卫地距
            if (geodist(pStruct->m_jSatStatus_SatPos[prn], RcvPos1, rsVec1) <= 0.0)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_XMEQ;
                continue;
            }

            if (geodist(pStruct->m_pSatStatus[prn].SatPos, RcvPos2, rsVec2) <= 0.0)
            {
                pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_XMEQ;
                continue;
            }

            // 判断高度角截止阈值
            if (satazel(RcvLLH2, rsVec2, azel) < /*pStruct->m_pGNSSOpt->m_ElevMask*/1.0 * D2R)
            {
                // 高度角会随着位置迭代而变化,所以只在验后进行赋值
                if (iter == 0) pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_ELEV;
                continue;
            }

            // sagnac效应影响很大!!!
            sagnac1 = gs_WGS84_OMGE * (pStruct->m_jSatStatus_SatPos[prn][0] * RcvPos1[1] - pStruct->m_jSatStatus_SatPos[prn][1] * RcvPos1[0]) / CLIGHT;
            sagnac2 = gs_WGS84_OMGE * (pStruct->m_pSatStatus[prn].SatPos[0] * RcvPos2[1] - pStruct->m_pSatStatus[prn].SatPos[1] * RcvPos2[0]) / CLIGHT;

            M31_M31(rsVec2, rsVec1, M1);
            MatrixMultiply(1, 3, M1, 3, 1, RcvPos1, &dg, 1.0);

            MatrixMultiply(1, 3, rsVec2, 3, 1, pStruct->m_pSatStatus[prn].SatPos, M1, 1.0);
            MatrixMultiply(1, 3, rsVec1, 3, 1, pStruct->m_jSatStatus_SatPos[prn], M2, 1.0);
            dD = M1[0] - M2[0];

            MatrixMultiply(1, 3, rsVec2, 3, 1, pStruct->m_DisXYZ, M2, 1.0);

            ddist = dD - dg - M2[0] + (sagnac2 - sagnac1);
            ddist += (pStruct->m_DisClkVel + (-CLIGHT * (pStruct->m_pSatStatus[prn].SatClk)) - (-CLIGHT * (pStruct->m_jSatStatus_SatClk[prn])));

            v = (pObsDatai->L[f] - pObsDataj->L[f]) * lam - ddist;
            var = Variance(prn, f, azel[1], pObsDatai->S[f], OBS_LI) + Variance(prn, f, azel[1], pObsDataj->S[f], OBS_LI);

            if (/*pStruct->m_pGNSSOpt->m_bRCS_GB*/1) // for RCS
            {
                bool bEst = false;
                if (pObsDatai->S[f] < 40.0 || azel[1] < 45.0 * D2R) bEst = true;
                if (iter == 1 && (pStruct->m_pSatStatus[prn].CS_Type[f] || (pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] & OBSEVT_XDOP) || bEst))
                {
                    pStruct->m_iPrnFrq_CS[f * NSATMAX + prn - 1] = pStruct->m_DisICS + pStruct->m_nEstCS;
                    pStruct->m_iCS_PrnFrq[pStruct->m_nEstCS++] = f * 1000 + prn;
                }
                if (pStruct->m_iPrnFrq_CS[f * NSATMAX + prn - 1] >= pStruct->m_DisICS)
                {
                    v -= pStruct->m_DisCSF[pStruct->m_iPrnFrq_CS[f * NSATMAX + prn - 1] - pStruct->m_DisICS] * lam;
                    if (iter > 0)
                    {
                        pStruct->m_B[pStruct->m_nMeas * MAXDISNX + pStruct->m_iPrnFrq_CS[f * NSATMAX + prn - 1]] = lam;
                    }
                }
            }

            if (iter > 0)
            {
                // dPos的线性化系数
                int s1[2] = { pStruct->m_nMeas, 0 }, s2[2] = { 0 };
                MatrixCopySub(s2, s1, 1, 3, 1, 3, rsVec2, MAXOBSLIM, MAXDISNX, pStruct->m_B, true);
                pStruct->m_B[pStruct->m_nMeas * MAXDISNX + 3] = 1.0;
                pStruct->m_V[pStruct->m_nMeas] = v;
                pStruct->m_R[pStruct->m_nMeas * MAXOBSLIM + pStruct->m_nMeas] = 1.0 / var; // Attention!!!
            }
            else
            {
                pStruct->m_Resi[pStruct->m_nMeas] = v;
                pStruct->m_ResiSig[pStruct->m_nMeas] = var;
            }

            pStruct->m_azel[0][prn] = azel[0];
            pStruct->m_azel[1][prn] = azel[1];
            pStruct->m_OBSUsed[pStruct->m_nMeas].prn = prn;
            pStruct->m_OBSUsed[pStruct->m_nMeas].f = f;
            pStruct->m_nMeas++;
        }
    }

    // 待估参数个数, for RCS
    if (/*pStruct->m_pGNSSOpt->m_bRCS_GB*/1) {
        pStruct->m_nState += pStruct->m_nEstCS;
    }

    // 真值强约束，for RCS
    if (1) 
    {
        double ref[3] = { 0.0, 0.0, 0.0 };
        if (iter > 0)
        {
            for (int i = 0; i < 3; i++) {
                pStruct->m_B[pStruct->m_nMeas * MAXDISNX + i] = 1.0;
                pStruct->m_V[pStruct->m_nMeas] = ref[i] - pStruct->m_DisXYZ[i];
                pStruct->m_R[pStruct->m_nMeas * MAXOBSLIM + pStruct->m_nMeas] = 1.0 / 0.000001;
                pStruct->m_nMeas++;
            }
        }
        else
        {
            for (int i = 0; i < 3; i++) {
                pStruct->m_Resi[pStruct->m_nMeas] = 0.0;
                pStruct->m_ResiSig[pStruct->m_nMeas] = 0.0001;
                pStruct->m_nMeas++;
            }
        }
    }

    // 计算验后残差直接返回
    if (iter == 0) return true;

    // 默认最大处理MAXOBSLIM个观测值
    if (pStruct->m_nMeas < pStruct->m_nState || pStruct->m_nMeas > MAXOBSLIM) return false;

    // 形成LSQ方程
    InitLSQ(&pStruct->m_LSQ, pStruct->m_nState, pStruct->m_nMeas, true);
    for (int i = 0; i < pStruct->m_nMeas; i++)
    {
        for (int j = 0; j < pStruct->m_LSQ.m_nState; j++) {
            pStruct->m_LSQ.m_B[i * pStruct->m_LSQ.m_nState + j] = pStruct->m_B[i * MAXDISNX + j];
        }
        pStruct->m_LSQ.m_L[i] = pStruct->m_V[i];
        for (int j = 0; j < pStruct->m_nMeas; j++) {
            pStruct->m_LSQ.m_P[pStruct->m_LSQ.m_nMeas * i + j] = pStruct->m_R[i * MAXOBSLIM + j];
        }
    }

    return true;
}


/** @brief  Post-processing quality control of TDCP. */
static bool Validate_Dis(CSPPoint* pStruct)
{
    if (!pStruct) return false;

    int k = 0, prn = 0, f = 0;
    bool bGross = false;

    // 验后RMS检验
    if (pStruct->m_DisLSQRMS > 0.05) bGross = true;

    // 验后残差检验
    setMEQ_Dis(pStruct, 0);
    getMaxPos(pStruct->m_Resi, pStruct->m_nMeas, &k, true);
    if (fabs(pStruct->m_Resi[k]) > 0.05) bGross = true;

    if (bGross)
    {
        prn = pStruct->m_OBSUsed[k].prn;
        f = pStruct->m_OBSUsed[k].f;
        pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] |= OBSEVT_Resi_v;
        pStruct->m_pSatStatus[prn].CS_Type[f] |= CST_Resi;
        return false;
    }

    return true;
}


/** @brief  Perform TDCP (single epoch). */
static int LSQ_Dis(CSPPoint* pStruct)
{
    if (!pStruct) return GNSSMODE_NONE;

    int iSuc = GNSSMODE_NONE;

    SetValsD(pStruct->m_azel[0], 0.0, NSATMAX + 1);
    SetValsD(pStruct->m_azel[1], 0.0, NSATMAX + 1);

    ClearLSQ(&pStruct->m_LSQ);

    ///< 1. 抗差迭代
    for (int iMLS = 0, nMeas = 0; (iMLS < nMeas / 2 && nMeas > 0) || nMeas == 0; iMLS++)
    {
        // 清空状态
        M31Zero(pStruct->m_DisXYZ);
        M33Zero(pStruct->m_DisXYZP);
        pStruct->m_DisClkVel = 0.0;
        if (/*pStruct->m_pGNSSOpt->m_bRCS_GB*/1) 
        {
            pStruct->m_nEstCS = 0;
            SetValsI(pStruct->m_iPrnFrq_CS, -1, NSATMAX * NFREQ);
            SetValsD(pStruct->m_DisCSF, 0.0, MAXOBSLIM);
        }

        bool flag = true;

        // 迭代DIS
        for (int it = 1; it < 10; it++)
        {
            if (setMEQ_Dis(pStruct, it) == false) { flag = false; break; }
            if (nMeas == 0) nMeas = pStruct->m_nMeas;
            if (LSQ(&pStruct->m_LSQ, false) == false) { flag = false; break; }

            M31M312(pStruct->m_LSQ.m_StateX, pStruct->m_DisXYZ, 1.0, 1.0);
            pStruct->m_DisClkVel += pStruct->m_LSQ.m_StateX[3];
            if (/*pStruct->m_pGNSSOpt->m_bRCS_GB*/1)
            {
                for (int i = 0; i < pStruct->m_nEstCS; i++)
                {
                    pStruct->m_DisCSF[i] += pStruct->m_LSQ.m_StateX[pStruct->m_DisICS + i];
                }
            }

            if (MatrixNorm2(3, 1, pStruct->m_LSQ.m_StateX) < 1E-3)
            {
                LSQ(&pStruct->m_LSQ, true); // 验后方差估计
                pStruct->m_DisLSQRMS = pStruct->m_LSQ.m_RMS;
                flag = (pStruct->m_DisLSQRMS == 0.0) ? false : true;
                break;
            }
        }
        if (flag == false) break; // 迭代未收敛

        // 检验结果, 失败再次抗差迭代
        if (Validate_Dis(pStruct))
        {
            iSuc = GNSSMODE_SPP;
            pStruct->m_DisnMeas = pStruct->m_nMeas;
            break;
        }
    }

    ///< 2. 质量控制
    if (iSuc == GNSSMODE_SPP)
    {
        double posLLH[3] = { 0.0 }, disENU[3] = { 0.0 }, rel[9] = { 0.0 };
        if (pStruct->m_PriorPosSigma < 999.0)
        {
            // 高程位移检核
            XYZ2LLH(pStruct->m_PriorPosClk, posLLH, 0);
            Rxyz2enu(posLLH, rel);
            M33XM31(rel, pStruct->m_DisXYZ, disENU);
            if (pStruct->m_GapTime > 0.0 && fabs(disENU[2] / pStruct->m_GapTime) > /*pStruct->m_pGNSSOpt->m_MaxVerVel*/1.2)
            {
                iSuc = GNSSMODE_NONE;
            }

            // PDOP检核
            ComputeDOP(pStruct->m_azel[0], pStruct->m_azel[1], pStruct->m_pGNSSOpt->m_ElevMask, pStruct->m_DisDOPs, NULL, false, false);
            if (pStruct->m_DisDOPs[1] > 6.0)
            {
                iSuc = GNSSMODE_NONE;
            }

            // 用于周跳修复, LAMBDA方法将周跳固定为整数值, for RCS
            if (/*pStruct->m_pGNSSOpt->m_bRCS_GB*/1 && iSuc == GNSSMODE_SPP && pStruct->m_nEstCS > 0)
            {
                if (1) // LAMBDA算法整体搜索
                {
                    LSQ(&pStruct->m_LSQ, false);
                    pStruct->m_ILS.m_method = 6;
                    pStruct->m_ILS.m_FTThresBound[0] = 1.001;
                    pStruct->m_ILS.m_FTThresBound[1] = 1.001;
                    pStruct->m_ILS.m_ADOPThres = 0.0;
                    pStruct->m_ILS.m_SRBSThres = 0.0;
                    // 初始化ILS估计器
                    InitLambda(&pStruct->m_ILS, pStruct->m_nEstCS);
                    for (int i = 0; i < pStruct->m_nEstCS; i++)
                    {
                        pStruct->m_ILS.m_Float[i] = pStruct->m_DisCSF[i] * 2.0;
                        for (int j = 0; j < pStruct->m_nEstCS; j++)
                        {
                            pStruct->m_ILS.m_QFloat[i * pStruct->m_nEstCS + j] = 4.0 * pStruct->m_LSQ.m_StateP[(i + pStruct->m_DisICS) * (pStruct->m_DisICS + pStruct->m_nEstCS) + j + pStruct->m_DisICS];
                        }
                    }

                    // 固定模糊度
                    if (LAMBDA(&pStruct->m_ILS, false))
                    {
                        for (int i = 0; i < pStruct->m_nEstCS; i++)
                        {
                            pStruct->m_DisCSF[i] = pStruct->m_ILS.m_afixed[i] / 2.0;
                        }
                        setMEQ_Dis(pStruct, 0);
                        for (int i = 0; i < pStruct->m_nMeas - 3; i++)
                        {
                            int prn = pStruct->m_OBSUsed[i].prn;
                            int frq = pStruct->m_OBSUsed[i].f;
                            int iCS = pStruct->m_iPrnFrq_CS[frq * NSATMAX + prn - 1] - pStruct->m_DisICS;
                            double CS = (iCS >= 0 && pStruct->m_DisCSF[iCS] != 0.0) ? pStruct->m_DisCSF[iCS] : 0.0;
                            char sprn[5];
                            satprn2nos(prn, sprn);
                            if (pStruct->m_CSEstFp)
                            {
                                fprintf(pStruct->m_CSEstFp, "%4d,%8.1f,%3s,%3d,%d,%8.3f,%8.3f,%8.3f,%6.1f\n",
                                    pStruct->m_pOBSData->gt.GPSWeek,
                                    GetGPSTIMESow(pStruct->m_pOBSData->gt),
                                    sprn, prn, frq,
                                    pStruct->m_pSatStatus[prn].S[frq],
                                    pStruct->m_pSatStatus[prn].azel[1] * R2D,
                                    pStruct->m_Resi[i],
                                    CS);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            iSuc = GNSSMODE_NONE;
        }
    }

    ///< 3. 更新状态
    if (iSuc == GNSSMODE_SPP)
    {
        int s1[2] = { 0 }, s2[2] = { 0 };
        MatrixCopySub(s1, s2, 3, 3, pStruct->m_LSQ.m_nMeas, pStruct->m_LSQ.m_nState, pStruct->m_LSQ.m_StateP, 3, 3, pStruct->m_DisXYZP, false);

        // 将位移转换为速度
        if (pStruct->m_GapTime > 0.0)
        {
            MatrixScaleMultiply(1.0 / pStruct->m_GapTime, 3, 1, pStruct->m_DisXYZ);
            pStruct->m_DisClkVel /= pStruct->m_GapTime;
        }

        // 输出Log信息方便调试
        if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
        {
            pStruct->m_pGNSSSol->m_LOG.SPVSolType = 1;
            pStruct->m_pGNSSSol->m_LOG.SPVnMeas[0] = 0;
            pStruct->m_pGNSSSol->m_LOG.SPVnMeas[1] = pStruct->m_DisnMeas;
            pStruct->m_pGNSSSol->m_LOG.SPVXYZ[0] = pStruct->m_DisXYZ[0];
            pStruct->m_pGNSSSol->m_LOG.SPVXYZ[1] = pStruct->m_DisXYZ[1];
            pStruct->m_pGNSSSol->m_LOG.SPVXYZ[2] = pStruct->m_DisXYZ[2];
        }
    }
    else
    {
        M33Zero(pStruct->m_DisXYZ);
        pStruct->m_DisClkVel = 0.0;
    }

    // 清空状态, 不影响后续流程
    for (int prn = 0; prn <= NSATMAX; prn++)
    {
        for (int f = 0; f < NFREQ; f++)
        {
            if (pStruct->m_pGNSSOpt->m_bFreqUsed[f] == false) continue;
            pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] &= ~OBSEVT_Resi_v;
            pStruct->m_pSatStatus[prn].CS_Type[f] &= ~CST_Resi;
        }
    }

    return iSuc;
}


static bool runSPP_PVD(CSPPoint* pStruct)
{
    if (!pStruct)return false;

    pStruct->m_SPPSolType = LSQ_Pos(pStruct); // 执行单点定位
    pStruct->m_DisSolType = LSQ_Dis(pStruct); // 执行单点测速

    return true;
}


static bool UpdateSolution(CSPPoint* pStruct)
{
    if (!pStruct)return false;

    // 1.结果写文件
    pStruct->m_SolTime = pStruct->m_OBSTime;
    if (pStruct->m_pGNSSSol)
    {
        pStruct->m_pGNSSSol->m_time = pStruct->m_OBSTime;
        pStruct->m_pGNSSSol->m_nsat = pStruct->m_pOBSData->nsat;
        pStruct->m_pGNSSSol->m_PDOP = pStruct->m_SPPDOPs[1];
        pStruct->m_pGNSSSol->m_DDOP = pStruct->m_SPPDOPs[1] * pStruct->m_SPPDOPs[1];
        pStruct->m_pGNSSSol->m_HDOP = pStruct->m_SPPDOPs[2];
        pStruct->m_pGNSSSol->m_VDOP = pStruct->m_SPPDOPs[3];
        pStruct->m_pGNSSSol->m_SolType = pStruct->m_SPPSolType;
        M31EQU(pStruct->m_SPPXYZ, pStruct->m_pGNSSSol->m_XYZPos);
        M31EQU(pStruct->m_SPPLLH, pStruct->m_pGNSSSol->m_LLHPos);
        M33EQU(pStruct->m_SPPXYZP, pStruct->m_pGNSSSol->m_XYZPosP);
        //pStruct->m_pGNSSSol->m_VelType = pStruct->m_dXYZType;

        pStruct->m_pGNSSSol->m_pSatStatus = pStruct->m_pSatStatus;
        pStruct->m_pGNSSSol->m_pOBSData = pStruct->m_pOBSData;

        for (int i = 0; i < NSYS; i++)
        {
            pStruct->m_pGNSSSol->m_ngnss[i] = pStruct->m_pOBSData->ngnss[i];
        }
    }

    // 2.数据备份赋值
    memcpy(&pStruct->m_jObsData, pStruct->m_pOBSData, sizeof(OBS_DATA));
    pStruct->m_jPosSolType = pStruct->m_SPPSolType;
    M31EQU(pStruct->m_SPPXYZ, pStruct->m_jSPPXYZ);
    for (int i = 0; i < NSYS * NFREQ; i++)pStruct->m_jSPPClk[i] = pStruct->m_SPPClk[i];

    for (int i = 0; i <= NSATMAX; i++)
    {
        pStruct->m_jSatStatus_Satsvh[i] = pStruct->m_pSatStatus[i].Satsvh;
        pStruct->m_jSatStatus_SatPos[i][0] = pStruct->m_pSatStatus[i].SatPos[0];
        pStruct->m_jSatStatus_SatPos[i][1] = pStruct->m_pSatStatus[i].SatPos[1];
        pStruct->m_jSatStatus_SatPos[i][2] = pStruct->m_pSatStatus[i].SatPos[2];
        pStruct->m_jSatStatus_SatClk[i] = pStruct->m_pSatStatus[i].SatClk;
    }

    // 3.重要参数更新
    if (pStruct->m_SPPSolType == GNSSMODE_SPP)
    {
        pStruct->m_PriorPosSigma = pStruct->m_SPPLSQRMS;
        M31EQU(pStruct->m_SPPXYZ, pStruct->m_PriorPosClk);
        for (int i = 3; i < MAXSPPNX; i++) pStruct->m_PriorPosClk[i] = pStruct->m_SPPClk[i - 3];
    }
    else
    {
        pStruct->m_PriorPosSigma = 999.9;
        SetValsD(pStruct->m_PriorPosClk, 0.0, MAXSPPNX);
    }

    return true;
}


bool runOneSPP(CSPPoint* pStruct, OBS_DATA* pOBSData)
{
    if (!pStruct)return false;

    // check license
    if (!CheckDogPermission(&pOBSData->gt)) return false;

    if (IsGPSTIMEEqual(pOBSData->gt, pStruct->m_OBSTime)) return true;
    pStruct->m_pOBSData = pOBSData;
    pStruct->m_OBSTime = pStruct->m_pOBSData->gt;

    if (PrepareSPP(pStruct) == false) return false;
    if (runSPP_PVD(pStruct) == false) return false;

    return UpdateSolution(pStruct);
}