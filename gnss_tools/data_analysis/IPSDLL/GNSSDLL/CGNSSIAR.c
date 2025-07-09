#include "CGNSSIAR.h"
#include "GNSSCmnFunc.h"
#include "BaseMath.h"


///< Constructor function, to be called when defining
void InitCGNSSIAR(CGNSSIAR* pStruct)
{
    if (!pStruct)return;

    // 外部初始化信息
    pStruct->m_pGNSSOpt = NULL;
    pStruct->m_GLS = NULL;
    pStruct->m_SatStatus[0] = pStruct->m_SatStatus[1] = NULL;
    pStruct->m_pOBSData[0] = pStruct->m_pOBSData[1] = NULL;
    pStruct->m_pGLSInfo = NULL;
    pStruct->m_pGNSSSol = NULL;
    for (int f = 0; f < NFREQ; f++)
    {
        pStruct->m_bFreqUsed[f] = true;
        pStruct->m_bReinitAllAmb[f] = false;
    }

    // 其他信息
    pStruct->m_bUseILS = true;
    pStruct->m_bUseBIE = false;
    pStruct->m_bFixWL = false;

    pStruct->m_ARRatio = 0.0;
    //pStruct->m_ARRatioPre = 0.0;
    pStruct->m_AmbState = ARSTATE_NONE;
    pStruct->m_AmbStatePre = ARSTATE_NONE;
    pStruct->m_ADOP = 0.0;
    pStruct->m_SR_ADOP = 0.0;
    pStruct->m_SR_Bootstrap = 0.0;
    pStruct->m_nFixed = 0;
    pStruct->m_ADOPFull = 0.0;
    pStruct->m_SR_ADOPFull = 0.0;
    pStruct->m_SR_BootstrapFull = 0.0;
    pStruct->m_nSatARHold = 0;
    pStruct->m_nARHold = 0;
    SetValsI(pStruct->m_nPara, 0, 4);
    SetValsI(pStruct->m_BasePrns[0], 0, NFREQ * NSYS);

    pStruct->m_bILSFixed = false;
    pStruct->m_bBIEFixed = false;
    pStruct->m_bFixQuality = false;
    pStruct->m_bFixQualityPre = false;

    InitCILSEstimator(&pStruct->m_ILS);
}


bool InitIAR(CGNSSIAR* pStruct, CGNSSOption* pGNSSOpt, CGNSSSolution* pGNSSSol, bool bFreqUsed[NFREQ])
{
    if (!pStruct || !pGNSSOpt)return false;

    pStruct->m_pGNSSOpt = pGNSSOpt;
    pStruct->m_pGNSSSol = pGNSSSol;

    for (int f = 0; f < NFREQ; f++) pStruct->m_bFreqUsed[f] = bFreqUsed[f];

    pStruct->m_ILS.m_ARThres = 2.0;

    return true;
}


static bool PrepareIAR(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    // 状态清空与赋值
    pStruct->m_ARRatio = 0.0;            ///< ARRatio值
    pStruct->m_AmbState_WL[0] = pStruct->m_AmbState_WL[1] = ARSTATE_FLOAT;
    pStruct->m_AmbState = ARSTATE_FLOAT;
    pStruct->m_ADOP = 0.0;
    pStruct->m_SR_ADOP = 0.0;            ///< ADOP成功率,上界
    pStruct->m_SR_Bootstrap = 0.0;       ///< BS成功率,下界
    pStruct->m_ADOPFull = 0;             ///< 进入到LAMBDA中的浮点模糊度的ADOP值
    pStruct->m_SR_ADOPFull = 0;          ///< 进入到LAMBDA中的浮点模糊度的ADOP成功率
    pStruct->m_SR_BootstrapFull = 0;     ///< 进入到LAMBDA中的浮点模糊度的Bootstrapping成功率
    pStruct->m_nFixed = 0;               ///< 固定的卫星数(双频时也是卫星数,非模糊度个数)
    pStruct->m_nSatARHold = 0;
    pStruct->m_bILSFixed = false;        ///< ILS解算标识
    pStruct->m_bBIEFixed = false;        ///< BIE解算标识
    pStruct->m_bFixQuality = false;      ///< ILS固定解质量
    pStruct->m_bUseILSconsBIE = false;   ///< 是否使用ILS约束BIE的候选解搜索

    EQU_StateXP(pStruct->m_pGLSInfo->StateX, pStruct->m_pGLSInfo->StateP, pStruct->m_pGLSInfo->StateXa, pStruct->m_pGLSInfo->StatePa, MAXRTKNX);

    //bool bReinitAllAmb = true;
    //for (int f = 0; f < NFREQ; f++)
    //{
    //    if (pStruct->m_bReinitAllAmb[f] == false)
    //        bReinitAllAmb = false;
    //}

    for (int f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_bFreqUsed[f] == false) continue;

        for (int prn = 1; prn <= NSATMAX; prn++)
        {
            int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pStruct->m_SatStatus[IROVE][prn].sys, &prn);
            if (skip == 1) continue;
            else if (skip == 2) break;

            if (pStruct->m_SatStatus[IROVE][prn].lock[f] <= 1) 
                pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] = pStruct->m_bReinitAllAmb[f] ? -1 : -pStruct->m_pGNSSOpt->m_ARMinLock;

            if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f]) pStruct->m_SatStatus[IROVE][prn].AR_Lock[f]++;
        }
    }

    return true;
}


static bool getFPS(CGNSSIAR* pStruct, int StateIndex, int* frq, int* prn, int* sys)
{
    if (!pStruct)return false;

    if (StateIndex < pStruct->m_pGLSInfo->IAmb || StateIndex >= pStruct->m_pGLSInfo->EAmb) return false;

    int i;
    for (i = 0; i < NSATMAX * NFREQ; i++) if (StateIndex == pStruct->m_pGLSInfo->m_AmbIndex[i]) break;
    if (i >= NSATMAX * NFREQ) return false;

    int f = i / NSATMAX;
    if (frq) *frq = f;                      //计算当前模糊度参数对应的频率
    if (prn) *prn = i + 1 - f * NSATMAX;    //计算当前模糊度参数对应的prn号
    if (sys) { int prnt = i + 1 - f * NSATMAX; *sys = pStruct->m_SatStatus[IROVE][prnt].sys; }

    return true;
}


static double SelectBaseSat_BDSVar(CGNSSOption* pGNSSOpt, int prn, double el)
{
    if (!pGNSSOpt)return 1.0;

    double var = pGNSSOpt->m_Error_a * pGNSSOpt->m_Error_a + pGNSSOpt->m_Error_b * pGNSSOpt->m_Error_b / (sin(el) * sin(el));

    int BDSType = BDSSatelliteType(prn);

    // BDS GEO和IGSO再放大
    if (BDSType == 1)
    {
        var *= 25;
        var += 0.25;
    }
    else if (BDSType == 2)
    {
        var *= 4;
    }

    return var;
}


static bool SelectBaseSat(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    SetValsI(pStruct->m_BasePrns[0], 0, NFREQ * NSYS);

    SATSTATUS* pSatStatus = NULL;

    int prn, isys;
    double var, el, elevs[NSYS];


    for (int f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_bFreqUsed[f] == false) continue;

        SetValsD(elevs, 0.0, NSYS);

        elevs[ISYSBD2] = 999999999.0;
        elevs[ISYSBD3] = 999999999.0;

        for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
        {
            prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
            pSatStatus = pStruct->m_SatStatus[IROVE] + prn;

            // 该卫星必须是可固定
            if (pSatStatus->AR_Status[f] != ARSTATE_FIXED) continue;

            // 相位必须验证通过
            if (pSatStatus->OBSValid[OBS_LI][f] == false) continue;

            if (pStruct->m_bReinitAllAmb[f] == false && pSatStatus->CS_Type[f])continue;

            if (pStruct->m_AmbStatePre >= ARSTATE_FIXED && pSatStatus->AR_StatusPre[f] < ARSTATE_FIXED)
                continue;

            isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
            el = pStruct->m_SatStatus[IROVE][prn].azel[1];

            // 如果都小于,则高度角除以2
            el = el / 2.0;
            for (int fi = 0; fi < NFREQ; fi++)
            {
                if (pStruct->m_SatStatus[IROVE][prn].lock[fi] >= 3)
                {
                    el = el * 2.0;
                    break;
                }
            }

            // 选择高度角最高的一颗卫星
            if (isys == ISYSBD2 || isys == ISYSBD3)
            {
                // BDS卫星的参考星选取根据方差来
                var = SelectBaseSat_BDSVar(pStruct->m_pGNSSOpt, prn, el);
                if (elevs[isys] > var)
                {
                    elevs[isys] = var;
                    pStruct->m_BasePrns[f][isys] = prn;
                }
            }
            else
            {
                if (elevs[isys] < el)
                {
                    elevs[isys] = el;
                    pStruct->m_BasePrns[f][isys] = prn;
                }
            }
        }
    }

    return true;
}


static bool ValidAmbs(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;
    if (pStruct->m_pGLSInfo->nStateIndex <= pStruct->m_pGLSInfo->IAmb)return false;

    pStruct->m_nPara[0] = pStruct->m_nPara[1] = pStruct->m_nPara[2] = pStruct->m_nPara[3] = 0;
    while (pStruct->m_pGLSInfo->StateIndex[pStruct->m_nPara[0]] < pStruct->m_pGLSInfo->IAmb) pStruct->m_nPara[0]++;

    int prn = 0, isys = ISYSNON, sys = 0, f = 0, prn_base = 0, ambLoc = 0, ambLoc_base = 0, iNL = 0, i = 0, nValidPRNs = 0;
    double NL_none = 0.0, NL_base = 0.0, dNL = 0, vNL = 0, sr = 0;
    const int n = pStruct->m_pGLSInfo->nStateIndex;

    /// 1.对各个卫星是否可以进入固定解做初步判定
    for (i = 0; i < n; i++)
    {
        // 排除非模糊度参数
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false) continue;

        if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f] == false)
        {
            // 因为验前残差超限而排除,但其实模糊度还是一直在跟踪,所以模糊度还是能够固定???
            if ((ambLoc = pStruct->m_pGLSInfo->m_AmbIndex[f * NSATMAX + (prn - 1)]) < pStruct->m_pGLSInfo->IAmb)  return false;
            if (pStruct->m_pGLSInfo->StatePa[ambLoc * MAXRTKNX + ambLoc] <= 0.0 || pStruct->m_SatStatus[IROVE][prn].lock[f] <= 3) continue;
        }

        pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;

        if (sys == SYSGLO)                                          continue; // GLO不固定
        if (pStruct->m_SatStatus[IROVE][prn].azel[1] < 10.0 * D2R)  continue; // 高度角限定
        if (pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] < 0)		continue; // 跟踪历元数

        pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FIXED;
        nValidPRNs++;
    }

    // 判断可送入固定卫星数
    if (nValidPRNs < pStruct->m_pGNSSOpt->m_AR_nSat) return false;

    ///2. 得到窄巷的参考星
    if (SelectBaseSat(pStruct) == false) return 0;

    // 多系统的时候,如果某个系统卫星都没通过固定条件,那参考星状态也要为FLOAT
    // 该设置必须放在这,放到下面循环中,可能prn_base会错过赋值
    for (f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_bFreqUsed[f] == false) continue;

        for (isys = 0; isys < NSYS; isys++)
        {
            prn_base = pStruct->m_BasePrns[f][isys];
            if (prn_base > 0) pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FLOAT;
        }
    }

    /// 3. 进一步判断是否可以进入固定解
    for (i = 0; i < n; i++)
    {
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false) continue;
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

        prn_base = pStruct->m_BasePrns[f][isys];
        if (prn_base <= 0)
        {
            pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;
            continue;
        }
        if (prn == prn_base)
        {
            pStruct->m_nPara[1]++; // 基准星个数
            pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.vL[f] = 0.0;
            pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f] = 0.0;
            continue;
        }

        pStruct->m_nPara[3]++; // 所有的单差模糊度个数

        if (pStruct->m_SatStatus[IROVE][prn].AR_Status[f] != ARSTATE_FIXED) continue;

        pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;

        // 参考星(单位周)
        if ((ambLoc_base = pStruct->m_pGLSInfo->m_AmbIndex[f * NSATMAX + (prn_base - 1)]) < pStruct->m_pGLSInfo->IAmb) return false;
        NL_base = pStruct->m_pGLSInfo->StateXa[ambLoc_base];

        // 非参考星(单位周)
        if ((ambLoc = pStruct->m_pGLSInfo->m_AmbIndex[f * NSATMAX + (prn - 1)]) < pStruct->m_pGLSInfo->IAmb) return false;
        NL_none = pStruct->m_pGLSInfo->StateXa[ambLoc];

        dNL = NL_none - NL_base;
        iNL = RoundNum(dNL);
        dNL = dNL - iNL;
        vNL = pStruct->m_pGLSInfo->StatePa[ambLoc_base * MAXRTKNX + ambLoc_base]
            + pStruct->m_pGLSInfo->StatePa[ambLoc * MAXRTKNX + ambLoc]
            - 2.0 * pStruct->m_pGLSInfo->StatePa[ambLoc_base * MAXRTKNX + ambLoc];
        vNL = xsqrt(vNL);
        sr = SR_Rounding(dNL, vNL);

        // 固定条件1: 标准差
        if (vNL > 10.0) continue;

        // 固定条件2: 小数部分，0.9相当于没起作用
        if (fabs(dNL) > 0.9) continue;

        // 固定条件3: 直接取整成功率
        if (sr < 0.2) continue;

        //if (fabs(GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt) - 279293.0) < 0.1)
        //{
        //    if (prn == 76 && f == 1)
        //        continue;
        //}

        pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.vL[f] = vNL;
        pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f] = fabs(dNL) / vNL;

        pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FIXED;
        pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FIXED;
        pStruct->m_nPara[2]++;
        //printf("%3d,%d\n", prn, f);
    }

    // 判断送入后续固定的双差浮点模糊度数量
    if (pStruct->m_nPara[2] < pStruct->m_pGNSSOpt->m_AR_nSat) return false;

    return true;
}


static bool getFloatDDAmb(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    const int n = pStruct->m_pGLSInfo->nStateIndex;
    const int nPara = pStruct->m_nPara[0];
    const int nBase = pStruct->m_nPara[1];
    const int nvAmb = pStruct->m_nPara[2];
    int prn = 0, f = 0, sys = 0, isys = 0, i = 0, j = 0, prn_base = 0, Bi[2] = { 0 }, Hi = 0, Xk = 0;
    int BaseLocs[NFREQ][NSYS] = { {0} }, BaseLocs1[NFREQ][NSYS] = { {0} };
    bool bAmb[MAXRTKNX] = { false };

    // m_HoldD.mat(nvAmb, n)，单差转双差阵，单差中包括非模糊度参数
    // m_HoldStateP.mat(n, n)，转换前单差方差阵，单差中包括非模糊度参数
    SetValsC(pStruct->m_PrnFrq_iAmb, 0, NSATMAX * NFREQ);
    pStruct->m_niAmb_PrnFrq = 0; // m_iAmb_PrnFrq 中元素数量
    SetValsD(pStruct->m_HoldD, 0, MAXOBSLIM * MAXRTKNX); // 一定要清空
    SetValsD(pStruct->m_HoldStateX, 0, MAXRTKNX);
    SetValsD(pStruct->m_HoldStateP, 0, MAXRTKNX * MAXRTKNX);

    // 1. 得到双差映射矩阵
    for (i = 0; i < n; i++)
    {
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false)
        {
            bAmb[i] = true; Hi++; Bi[0]++; Bi[1]++;
            continue;
        }

        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

        if ((prn_base = pStruct->m_BasePrns[f][isys]) < 1)
        {
            Bi[1]++;
            continue;
        }

        if (prn == prn_base)
        {
            BaseLocs[f][isys] = Bi[0]++;
            BaseLocs1[f][isys] = Bi[1]++;
            bAmb[i] = true;
            continue;
        }

        if (pStruct->m_SatStatus[IROVE][prn].AR_Status[f] != ARSTATE_FIXED)
        {
            Bi[1]++;
            continue;
        }

        bAmb[i] = true;
        pStruct->m_HoldD[(Hi - nPara) * n + Bi[1]] = 1.0;
        Hi++; Bi[0]++; Bi[1]++;
    }

    // 2. 对模糊度个数进行验证
    if ((Hi + nBase != Bi[0]) || (Hi != nPara + nvAmb)) return false;

    // 3. 填充基准星的系数
    Hi = nPara;
    for (i = nPara; i < n; i++)
    {
        if (bAmb[i] == false) continue;
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false) break;
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        prn_base = pStruct->m_BasePrns[f][isys];
        if (prn_base == prn) continue;
        Bi[0] = BaseLocs[f][isys];
        Bi[1] = BaseLocs1[f][isys];
        pStruct->m_HoldD[(Hi - nPara) * n + Bi[1]] = -1.0;
        Hi++;
    }

    // 4. 得到浮点模糊度的方差
    for (i = 0; i < n; i++)
    {
        pStruct->m_HoldStateX[i] = pStruct->m_pGLSInfo->StateXa[pStruct->m_pGLSInfo->StateIndex[i]];
        for (j = 0; j < n; j++)
            pStruct->m_HoldStateP[i * n + j] = pStruct->m_pGLSInfo->StatePa[pStruct->m_pGLSInfo->StateIndex[i] * MAXRTKNX + pStruct->m_pGLSInfo->StateIndex[j]];

        if (bAmb[i] == false) continue;

        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys))
        {
            isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

            // 双差模糊度序号对应的卫星
            if (prn != pStruct->m_BasePrns[f][isys])
            {
                pStruct->m_PrnFrq_iAmb[f * NSATMAX + prn - 1] = Xk + 1; // -1是因为prn从1开始
                pStruct->m_iAmb_PrnFrq[pStruct->m_niAmb_PrnFrq++] = f * 1000 + prn;
                Xk++;
            }
        }
    }

    //pStruct->m_FLOAmb.mat(nvAmb, 1); pStruct->m_FLOAmbP.mat(nvAmb, nvAmb); pStruct->m_FIXAmb.mat(nvAmb, 1);

    // 5. 得到双差浮点模糊度及其方差阵
    SetValsD(pStruct->m_FLOAmb, 0, MAXOBSLIM);
    SetValsD(pStruct->m_FLOAmbP, 0, MAXOBSLIM * MAXOBSLIM);
    SetValsD(pStruct->m_FIXAmb, 0, MAXOBSLIM);

    MatrixMultiply(nvAmb, n, pStruct->m_HoldD, n, 1, pStruct->m_HoldStateX, pStruct->m_FLOAmb, 1.0);
    MatrixMultiply_HPHT(nvAmb, n, pStruct->m_HoldD, pStruct->m_HoldStateP, false, pStruct->m_FLOAmbP);

    return true;
}


static void DetectAmbOutlier(CGNSSIAR* pStruct)
{
    if (!pStruct)return;

    const int nAmb = pStruct->m_nPara[2];///< 存储参数个数,0:非模糊度参数,1:基准星,2:有效的单差模糊度,3:所有的单差模糊度
    int f, prn, i, j, k, m, nProirV = 0;
    double ProirV[MAXOBSLIM], resi_avg, resi_std, resiAvg, resiStd, value;

    for (i = 0; i < nAmb; i++)
    {
        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
        ProirV[nProirV++] = pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f];
    }

    if (nProirV == 0)return;
    SortArr(ProirV, nProirV, true);
    m = nAmb / 2;
    resiAvg = ProirV[m];
    resiStd = 3.0;
    j = 1;
    while (m - j > 0 && m + j < nAmb)
    {
        k = 0;
        resi_avg = resi_std = 0;
        for (i = m - j; i <= m + j; i++)
        {
            if (i == m) continue;
            if (fabs(ProirV[i] - resiAvg) <= 3.0 * resiStd)
            {
                resi_avg += ProirV[i];
                resi_std += (ProirV[i] - resiAvg) * (ProirV[i] - resiAvg);
                k++;
            }
        }

        if (k > 1)
        {
            resiAvg = resi_avg / k;
            value = xsqrt(resi_std / (k - 1));
            resiStd = (resiStd > value) ? resiStd : value;
        }
        j++;
    }

    for (i = 0; i < nAmb; i++)
    {
        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
        if (fabs(pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f] - resiAvg) > 3.0 * resiStd || fabs(pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f]) > 5.0)
            pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[f] = true;
        else pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[f] = false;
    }

}


#define NIARExcInd  4
// mode 0:信噪比,1:高度角,2:模糊度方差
static bool BuildPARIndex(CGNSSIAR* pStruct, const int mode)
{
    if (!pStruct)return false;

    int prn, f, i, j, k, iind[NIARExcInd] = { 0 };
    float ind[NIARExcInd][MAXOBSLIM];
    double val = 0.0;

    //SetValsI(pStruct->m_PARIndex[mode], 1, MAXOBSLIM);

    for (i = 0; i < pStruct->m_niAmb_PrnFrq; i++)
    {
        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;

        //pStruct->m_PARIndex[i] = pStruct->m_PrnFrq_iAmb[f * NSATMAX + prn - 1] - 1;
        pStruct->m_PARIndex[mode][i] = i;

        // 1. 是否首次固定
        if (pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] == 0)
        {
            ind[0][iind[0]++] = 1;
        }
        else
        {
            ind[0][iind[0]++] = 0;
        }

        // 2. 前一历元没固定,这样可以保持连续
        if (pStruct->m_bFixQualityPre == true/*pStruct->m_AmbStatePre >= ARSTATE_FIXED*/)
        {
            if (pStruct->m_SatStatus[IROVE][prn].AR_StatusPre[f] < ARSTATE_FIXED)
            {
                ind[1][iind[1]++] = 1;
            }
            else
            {
                ind[1][iind[1]++] = 0;
            }
        }
        else
        {
            ind[1][iind[1]++] = 1;
        }

        //// 3. 模糊度方差是否匹配，不匹配的话会导致R-ratio值较低
        //if (pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[f])
        //{
        //    ind[2][iind[2]++] = 1;
        //}
        //else
        //{
            ind[2][iind[2]++] = 0;
        //}

        // 4. 高度角(小于60°才有)、信噪比、模糊度方差
        switch (mode)
        {
        case 0: {val = 1 / pStruct->m_SatStatus[IROVE][prn].S[f]; break; }
        case 1: {val = PI / pStruct->m_SatStatus[IROVE][prn].azel[1]; break; }
        case 2: {val = pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.vL[f]; break; }
        default:val = 0;
        }
        ind[3][iind[3]++] = (float)val;
    }

    const int n = pStruct->m_niAmb_PrnFrq;

    for (i = 0; i < n; i++)
    {
        k = i;
        for (j = i + 1; j < n; j++)
        {
            for (int p = 0; p < NIARExcInd; p++)
            {
                if (iind[p] == 0) continue;
                if (ind[p][j] == ind[p][k]) continue; // 相等就不比较,等待下一个
                if (ind[p][j] > ind[p][k]) k = j;     // j大,将j赋给k
                break;
            }
        }

        // 此时k为最大,进行交换
        if (k != i)
        {
            for (int p = 0; p < NIARExcInd; p++)
            {
                if (iind[p] == 0) continue;
                SwapF(&ind[p][i], &ind[p][k]);
            }
            SwapI(&pStruct->m_PARIndex[mode][i], &pStruct->m_PARIndex[mode][k]);
        }
    }

    return true;
}


// 返回有效元素的最大值和最小值之间的差异
static double GetMinMaxDif(double arr[], int size) {
    double min = 1e14, max = -1e14;

    // 遍历数组找到最大和最小的有效值（非零值）
    for (int i = 0; i < size; i++) {
        if (arr[i] != 0) {
            if (arr[i] < min) {
                min = arr[i];
            }
            if (arr[i] > max) {
                max = arr[i];
            }
        }
    }

    // 检查是否找到有效的非零值
    if (min == 1e14 || max == -1e14) {
        return 0; // 没有找到有效的非零值，返回0
    }

    return (max - min); // 返回最大值和最小值之间的差异
}


// 返回数组内出现次数最多元素
static int FindMostFrequent(int arr[], int size) {
    int max_freq = 0;  // 最多出现次数
    int most_frequent = arr[0];  // 初始化为数组的第一个元素

    for (int i = 0; i < size; i++) {
        int count = 0;
        for (int j = 0; j < size; j++) {
            if (arr[j] == arr[i]) {
                count++;
            }
        }
        // 更新最大频率和相应的元素
        if (count > max_freq) {
            max_freq = count;
            most_frequent = arr[i];
        }
    }

    return most_frequent;
}


// for test，执行双频模糊度一致性检验，未通过优先剔除
static bool AnalyzeDFAmb1(CGNSSIAR* pStruct, int nAmb, bool* bAmb, int* vAmb, int* exi)
{
    // 输入为m_iAmb_PrnFrq的索引
    double lam, ddGs[NSATMAX][NFREQ], ddG, dFAmb, maxdFAmb = 0;
    int f, prn, rprn, isys, maxprn = -1, rprns[NFREQ];
    memset(ddGs, 0, sizeof(double) * NSATMAX * NFREQ);
    bool bExclude = false, bCheck[NSYS][NFREQ];
    memset(bCheck, false, sizeof(bool) * NSYS * NFREQ);

    // 不同频点参考星相同才能执行一致性检验
    for (int i = 0; i < NSYS; i++)
    {
        for (int j = 0; j < NFREQ; j++)rprns[j] = pStruct->m_BasePrns[j][i];
        rprn = FindMostFrequent(rprns, NFREQ);
        for (int j = 0; j < NFREQ; j++)
        {
            if (rprn > 0 && rprn == pStruct->m_BasePrns[j][i])
                bCheck[i][j] = true;
        }
    }

    // 先计算所有卫星：双差观测值-双差模糊度，m
    for (int i = 0; i < nAmb; i++)
    {
        if (bAmb[i] == false)continue;

        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
        lam = pStruct->m_SatStatus[IROVE][prn].lam[f];
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        rprn = pStruct->m_BasePrns[f][isys]; // 参考星

        if (bCheck[isys][f] == false)continue;
        if (prn <= 0 || prn > NSATMAX || rprn <= 0 || rprn > NSATMAX)continue;

        ddG = lam * (pStruct->m_SatStatus[IROVE][prn].SD_L[f] - pStruct->m_SatStatus[IROVE][rprn].SD_L[f] - pStruct->m_FIXAmb[i]);
        ddGs[prn - 1][f] = (ddG == 0.0) ? IPS_EPSILON : ddG;
    }

    // 进行双频一致性检验
    for (int i = 0; i < nAmb; i++)
    {
        if (bAmb[i] == false)continue;

        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;

        if (prn <= 0 || prn > NSATMAX)continue;

        dFAmb = GetMinMaxDif(ddGs[prn - 1], NFREQ);
        if (dFAmb > maxdFAmb)
        {
            maxdFAmb = dFAmb;
            maxprn = prn;
        }
    }

    if (fabs(maxdFAmb) > 2)
    {
        int a = 1;
    }

    // 剔除双频一致性检验最差卫星
    if (maxprn > 0 && maxdFAmb > 0.02)
    {
        for (int i = 0; i < nAmb; i++)
        {
            if (bAmb[i] == false)continue;

            f = pStruct->m_iAmb_PrnFrq[i] / 1000;
            prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
            isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

            if (bCheck[isys][f] == false)continue;
            if (prn <= 0 || prn > NSATMAX)continue;

            if (prn == maxprn)
            {
                bAmb[i] = false;
                if (vAmb)*vAmb = *vAmb - 1;
                if (exi)*exi = *exi + 1;
                bExclude = true;
            }
        }
    }

    return bExclude;
}


static bool runPAR(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    const int nmode = 3;
    const int nAmb = pStruct->m_nPara[2]; // 注意应该有 m_nPara[2] == m_PARIndex.size() == m_iAmb_PrnFrq.size()
    bool bFixed = false, bPARs[3], bAmb[3][MAXOBSLIM]/*, bSyns[3]*/;
    int vAmb = nAmb, i = 0, j = 0, ai = 0, aj = 0, exi = 0, f = 0, prn = 0, k = 0, isys, prn_base, /*synsExi[3] = { 0 }, */mode = 0, maxexi = 0;
    int MaxAmbScoreMode = 0, MaxRatioScoreMode = 0, MaxAmbScore = 0, nAmbScore[3] = { 0 };
    double MaxRatioScore = 0.0, ratioScore[3] = { 0 }, ADOP[3] = { 0.0 }, SR_ADOP[3] = { 0.0 }, SR_Bootstrap[3] = { 0.0 }, afixed[3][MAXOBSLIM];

    // 初始化
    SetValsB(pStruct->m_bAmbFixed, false, nAmb); // 用于模糊度更新基线参数，赋值为最后固定的bAmb
    SetValsB(bPARs, false, 3);
    //SetValsB(bSyns, true, 3);
    SetValsB(bAmb[0], true, nAmb);
    SetValsB(bAmb[1], true, nAmb);
    SetValsB(bAmb[2], true, nAmb);

    pStruct->m_ILS.m_method = 6/*pStruct->m_pGNSSOpt->pStruct->m_LambdaMethod*/;
    pStruct->m_ILS.m_FTThresBound[0] = 1.5;
    pStruct->m_ILS.m_FTThresBound[1] = 5.0;
    //for (i = 0; i < NFREQ; i++) // 一旦某频点模糊度全部重置，严格阈值以提高可靠性
    //{
    //    if (pStruct->m_bReinitAllAmb[i])
    //    {
    //        pStruct->m_ILS.m_FTThresBound[0] = 2.5; // 经验阈值
    //        break;
    //    }
    //}

    // 探测模糊度异常值，优先剔除精度和方差不匹配的模糊度，edit by Lying
    DetectAmbOutlier(pStruct);

    // 构建模糊度剔除索引
    for (mode = 0; mode < nmode; mode++)BuildPARIndex(pStruct, mode);
    //for (i = 0; i < nAmb; i++)
    //{
    //    for (mode = 1; mode < nmode; mode++)
    //    {
    //        if (pStruct->m_PARIndex[mode - 1][i] != pStruct->m_PARIndex[mode][i])
    //        {
    //            bSyns[mode - 1] = false;
    //        }
    //        if (bSyns[mode - 1])synsExi[mode - 1]++;
    //    }
    //}

    // 考虑效率，将模糊度剔除至25个(经验阈值）
    while (vAmb > 25)    
    {
        for (mode = 0; mode < nmode; mode++)
        {
            k = pStruct->m_PARIndex[mode][exi];
            bAmb[mode][k] = false;
        }
        vAmb--; exi++;
    }

    maxexi = vAmb * 0.5 + exi; // PAR迭代次数不能太多
    // 信噪比,高度角,验后残差, 每种都来一遍部分模糊度固定, 取固定个数最多的为最终结果
    while (vAmb >= 6)
    {        
        for (mode = 0; mode < nmode; mode++)
        {
            // 初始化ILS估计器
            InitLambda(&pStruct->m_ILS, vAmb);

            for (i = ai = 0; i < nAmb; i++)
            {
                if (bAmb[mode][i])
                {
                    for (j = aj = 0; j < nAmb; j++)
                    {
                        if (bAmb[mode][j]) { pStruct->m_ILS.m_QFloat[ai * vAmb + aj] = pStruct->m_FLOAmbP[i * nAmb + j]; aj++; }
                    }
                    pStruct->m_ILS.m_Float[ai] = pStruct->m_FLOAmb[i];  ai++;
                }
            }

            // 固定模糊度
            bPARs[mode] = LAMBDA(&pStruct->m_ILS, false);

            //// for test
            //printf("vAmb = %d, r = %6.2f, TOW = %8.1f\n", vAmb, pStruct->m_ILS.m_sqnorm[1] / pStruct->m_ILS.m_sqnorm[0], GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt));
            //for (f = 0; f < NFREQ; f++)
            //    printf("%3d,%3d,%3d,%3d\n",
            //        pStruct->m_BasePrns[f][ISYSGPS],
            //        pStruct->m_BasePrns[f][ISYSBD2],
            //        pStruct->m_BasePrns[f][ISYSBD3],
            //        pStruct->m_BasePrns[f][ISYSGAL]);
            //for (i = 0; i < nAmb; i++)
            //{
            //    k = pStruct->m_PARIndex[mode][i];
            //    if (bAmb[mode][k])
            //    {
            //        ai = 0;
            //        for (j = 0; j < k; j++)
            //        {
            //            if (bAmb[mode][j])
            //                ai++;
            //        }
            //        f = pStruct->m_iAmb_PrnFrq[k] / 1000;
            //        prn = pStruct->m_iAmb_PrnFrq[k] % 1000;
            //        int a = pStruct->m_ILS.m_afixed[ai];
            //        char sprn[5];
            //        satprn2nos(prn, sprn);
            //        printf("%s(%3d),%d,%3d,%5.1f,%5.1f,%8.2f,%8.3f,%4d,%6.1f,%6.1f\n",
            //            sprn, prn, f,
            //            pStruct->m_SatStatus[IROVE][prn].AR_Lock[f],
            //            pStruct->m_SatStatus[IROVE][prn].S[f],
            //            pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.NL[f],
            //            pStruct->m_FLOAmb[k], pStruct->m_FLOAmbP[k * nAmb + k], a,
            //            pStruct->m_SatStatus[IROVE][prn].azel[0] * R2D,
            //            pStruct->m_SatStatus[IROVE][prn].azel[1] * R2D);
            //    }
            //}

            if (bPARs[mode])
            {
                nAmbScore[mode] = vAmb;
                ratioScore[mode] = pStruct->m_ILS.m_ARRatio;
                ADOP[mode] = pStruct->m_ILS.m_ADOP;
                SR_ADOP[mode] = pStruct->m_ILS.m_SR_ADOP;
                SR_Bootstrap[mode] = pStruct->m_ILS.m_SR_Bootstrap;
                for (i = 0; i < vAmb; i++)
                    afixed[mode][i] = pStruct->m_ILS.m_afixed[i];

                if (MaxAmbScore < nAmbScore[mode])
                {
                    MaxAmbScore = nAmbScore[mode];
                    MaxAmbScoreMode = mode;
                }

                if (MaxRatioScore < ratioScore[mode])
                {
                    MaxRatioScore = ratioScore[mode];
                    MaxRatioScoreMode = mode;
                }
                bFixed = true;
                break;
            }

            if (exi == 0 /*|| exi < synsExi[mode]*/) break;
        }
        if (bFixed || exi >= maxexi/*nAmb*/ /** 0.5*/)break;
        else
        {
            for (i = 0; i < 1; i++) // 每次剔除一个（剔除过多停止剔除以提高可靠性）
            {
                for (mode = 0; mode < nmode; mode++)
                {
                    k = pStruct->m_PARIndex[mode][exi];
                    bAmb[mode][k] = false;
                }
                vAmb--; exi++;
            }
        }
    }

    // 固定模式选择
    if (nAmbScore[MaxRatioScoreMode] == MaxAmbScore)
        bFixed = bPARs[MaxRatioScoreMode];
    else 
        bFixed = bPARs[MaxAmbScoreMode];

    // 5. 固定解状态赋值与更新
    for (f = 0; f < NFREQ; f++) // 记得先处理参考星
    {
        for (isys = 0; isys < NSYS; isys++)
        {
            prn_base = pStruct->m_BasePrns[f][isys];
            if (prn_base > 0) pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FLOAT;
        }
    }
    if (bFixed) // 固定解检核
    {
        // 根据Ratio值判断模糊度固定质量
        if (/*pStruct->m_ILS.m_ARRatio*/ratioScore[MaxAmbScoreMode] > 2.0) // 只是初步判断
        {
            pStruct->m_bFixQuality = true;
            pStruct->m_pGNSSOpt->m_ARFixAmbRMS = 5.0;
            pStruct->m_pGNSSOpt->m_AR_PDOP = 10.0;
        }
        else
        {
            pStruct->m_bFixQuality = false;
            pStruct->m_pGNSSOpt->m_ARFixAmbRMS = 3.0;
            pStruct->m_pGNSSOpt->m_AR_PDOP = 5.0;
        }

        // 固定解检核：模糊度浮点解与固定解差异
        double ambRms = 0;
        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[MaxAmbScoreMode][i])
            {
                ambRms += (afixed[MaxAmbScoreMode][ai] - pStruct->m_FLOAmb[i]) * (afixed[MaxAmbScoreMode][ai] - pStruct->m_FLOAmb[i]); ai++;
            }
        }
        if (ai > 0)ambRms = sqrt(ambRms / ai);
        if (ambRms > pStruct->m_pGNSSOpt->m_ARFixAmbRMS && pStruct->m_pGNSSOpt->m_ARFixAmbRMS > 0.0)bFixed = false;

        // 固定解检核，固定模糊度PDOP
        if (bFixed)
        {
            double DOPs[4] = { 0.0 };
            ComputeDOP(NULL, NULL, pStruct->m_pGNSSOpt->m_ElevMask, DOPs, pStruct->m_SatStatus[IROVE], false, true);
            if (DOPs[1] > pStruct->m_pGNSSOpt->m_AR_PDOP && pStruct->m_pGNSSOpt->m_AR_PDOP > 0.0)bFixed = false;
        }
    }

    if (bFixed) // 固定解更新
    {
        for (i = ai = 0; i < nAmb; i++)
        {
            f = pStruct->m_iAmb_PrnFrq[i] / 1000;
            prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
            if (bAmb[MaxAmbScoreMode][i])
            {
                pStruct->m_FIXAmb[i] = afixed[MaxAmbScoreMode][ai]; ai++; // 处理非参考星
                pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FIXED;
                pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = (int)pStruct->m_FIXAmb[i];

                isys = pStruct->m_SatStatus[IROVE][prn].sys_id; // 处理参考星
                prn_base = pStruct->m_BasePrns[f][isys];
                pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FIXED;
                pStruct->m_SatStatus[IROVE][prn_base].AR_Amb[f] = 0;
            }
            else
            {
                pStruct->m_FIXAmb[i] = pStruct->m_FLOAmb[i];
                pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;
                pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = 0;
            }
            pStruct->m_bAmbFixed[i] = bAmb[MaxAmbScoreMode][i];
        }
        pStruct->m_nFixed = nAmbScore[MaxAmbScoreMode]; // 模糊度固定数量
        pStruct->m_bILSFixed = true;
        pStruct->m_AmbState = ARSTATE_FIXED;
        pStruct->m_ARRatio = ratioScore[MaxAmbScoreMode];
        pStruct->m_ADOP = ADOP[MaxAmbScoreMode];
        pStruct->m_SR_ADOP = SR_ADOP[MaxAmbScoreMode];
        pStruct->m_SR_Bootstrap = SR_Bootstrap[MaxAmbScoreMode];
    }
    else
    {
        for (i = 0; i < nAmb; i++)
        {
            f = pStruct->m_iAmb_PrnFrq[i] / 1000;
            prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
            pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;
            pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = 0;
        }
        pStruct->m_AmbState = ARSTATE_FLOAT;
    }

    // For logging
    if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
    {
        pStruct->m_pGNSSSol->m_LOG.valAmbs = nAmb;
        if (bFixed)pStruct->m_pGNSSSol->m_LOG.fixAmbs = nAmbScore[MaxAmbScoreMode];
        pStruct->m_pGNSSSol->m_LOG.fixRatio = ratioScore[MaxAmbScoreMode];
        pStruct->m_pGNSSSol->m_LOG.fixPDOP = /*DOPs[1]*/0.0;
    }

    return (pStruct->m_AmbState > ARSTATE_FLOAT);
}


static bool runBIE(CGNSSIAR* pStruct)
{
    if (!pStruct || pStruct->m_bILSFixed == true) return false; // ILS固定成功就不计算BIE了

    const int nAmb = pStruct->m_nPara[2]; // 注意应该有 m_nPara[2] == m_PARIndex.size() == m_iAmb_PrnFrq.size()
    bool bFixed = false, bAmb[NSATMAX * NFREQ];
    int vAmb = nAmb, i = 0, j = 0, ai = 0, aj = 0, exi = 0, f = 0, prn = 0, k = 0, isys, prn_base;

    // 初始化
    SetValsB(pStruct->m_bAmbFixed, false, nAmb);
    SetValsB(bAmb, true, nAmb);
    pStruct->m_ILS.m_method = 9;
    pStruct->m_ILS.m_ncands = NCANDS;
    pStruct->m_ILS.m_BIEWeightMethod = 1;

    if (/*pStruct->m_bUseILSconsBIE*/0) // ILS约束搜索
    {

    }
    else
    {
        if (pStruct->m_bUseILS == false) // 如果ILS开启，这些流程是已经进行了
        {
            DetectAmbOutlier(pStruct);
            BuildPARIndex(pStruct, 0);
        }

        // 考虑效率，将模糊度剔除至25个(经验阈值）
        while (vAmb > 25)
        {
            k = pStruct->m_PARIndex[0][exi];
            bAmb[k] = false;
            vAmb--; exi++;
        }

        // BIE估计
        InitLambda(&pStruct->m_ILS, vAmb);

        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[i])
            {
                for (j = aj = 0; j < nAmb; j++)
                {
                    if (bAmb[j]) { pStruct->m_ILS.m_QFloat[ai * vAmb + aj] = pStruct->m_FLOAmbP[i * nAmb + j]; aj++; }
                }
                pStruct->m_ILS.m_Float[ai] = pStruct->m_FLOAmb[i];  ai++;
            }
        }

        bFixed = LAMBDA(&pStruct->m_ILS, pStruct->m_bUseILSconsBIE/*false*/);
    }

    // 固定解状态赋值与更新
    for (f = 0; f < NFREQ; f++) // 记得先处理参考星
    {
        for (isys = 0; isys < NSYS; isys++)
        {
            prn_base = pStruct->m_BasePrns[f][isys];
            if (prn_base > 0) pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FLOAT;
        }
    }
    if (bFixed) // 固定解检核
    {
        // 固定解检核：模糊度浮点解与固定解差异
        double ambRms = 0;
        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[i])
            {
                ambRms += (pStruct->m_ILS.m_afixed[ai] - pStruct->m_FLOAmb[i]) * (pStruct->m_ILS.m_afixed[ai] - pStruct->m_FLOAmb[i]); ai++;
            }
        }
        if (ai > 0)ambRms = sqrt(ambRms / ai);
        pStruct->m_pGNSSOpt->m_ARFixAmbRMS = 5.0; // BIE估计严格点
        if (ambRms > pStruct->m_pGNSSOpt->m_ARFixAmbRMS && pStruct->m_pGNSSOpt->m_ARFixAmbRMS > 0.0)bFixed = false;
    }

    if (bFixed) // 固定解更新
    {
        for (i = ai = 0; i < nAmb; i++)
        {
            f = pStruct->m_iAmb_PrnFrq[i] / 1000;
            prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
            if (bAmb[i])
            {
                pStruct->m_FIXAmb[i] = pStruct->m_ILS.m_afixed[ai]; ai++; // 处理非参考星
                pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOFIX;
                pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = (int)pStruct->m_FIXAmb[i];

                isys = pStruct->m_SatStatus[IROVE][prn].sys_id; // 处理参考星
                prn_base = pStruct->m_BasePrns[f][isys];
                pStruct->m_SatStatus[IROVE][prn_base].AR_Status[f] = ARSTATE_FLOFIX;
                pStruct->m_SatStatus[IROVE][prn_base].AR_Amb[f] = 0;
            }
            else
            {
                pStruct->m_FIXAmb[i] = pStruct->m_FLOAmb[i];
                pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;
                pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = 0;
            }
            pStruct->m_bAmbFixed[i] = bAmb[i];
        }
        pStruct->m_nFixed = pStruct->m_ILS.m_nfixed; // 模糊度固定数量
        pStruct->m_bBIEFixed = true;
        pStruct->m_AmbState = ARSTATE_FLOFIX;
    }
    else
    {
        for (i = 0; i < nAmb; i++)
        {
            f = pStruct->m_iAmb_PrnFrq[i] / 1000;
            prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
            pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;
            pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] = 0;
        }
        pStruct->m_AmbState = ARSTATE_FLOAT;
    }

    // For logging
    if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
    {
        pStruct->m_pGNSSSol->m_LOG.valAmbs = nAmb;
        if (bFixed)pStruct->m_pGNSSSol->m_LOG.fixAmbs = vAmb;
        pStruct->m_pGNSSSol->m_LOG.fixRatio = 0.0;
        pStruct->m_pGNSSSol->m_LOG.fixPDOP = 0.0;
    }

    return (pStruct->m_AmbState > ARSTATE_FLOAT);
}


static bool UpdateAmbStatus(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    //if (pStruct->m_bILSFixed)
    //{
        //pStruct->m_ARRatio = pStruct->m_ILS.m_ARRatio;
        //pStruct->m_ADOP = pStruct->m_ILS.m_ADOP;
        //pStruct->m_SR_ADOP = pStruct->m_ILS.m_SR_ADOP;
        //pStruct->m_SR_Bootstrap = pStruct->m_ILS.m_SR_Bootstrap;
        //pStruct->m_ADOPFull = pStruct->m_ILS.m_ADOPFull; // 仅在m_method=5时有效
        //pStruct->m_SR_ADOPFull = pStruct->m_ILS.m_SR_ADOPFull;
        //pStruct->m_SR_BootstrapFull = pStruct->m_ILS.m_SR_BootstrapFull;
    //}

    // 进行卫星的模糊度固定状态更新
    //const int n1 = pStruct->m_CommonPrns.size();
    //const int n0 = pStruct->m_CommonPrnsPre.size();
    int i, j, prn, f, prn_base, iL[2], nFixAmb = 0, nHoldSats = 0;
    int nbase = 0, max_base = 0, max_pos = 0;
    int CommonPrns[MAXOBS], nCommonPrns = 0;
    bool HoldSats[NSATMAX];
    SetValsB(HoldSats, false, NSATMAX);

    // 1. 寻找前历元与当前历元的交集
    for (f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_bFreqUsed[f] == false) continue;

        nCommonPrns = 0;
        for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
        {
            if (pStruct->m_pGLSInfo->MEQ[i].bMEQ == false) continue;

            prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

            // 如果当前状态为浮点,则所有卫星都要清空Amb状态
            if (pStruct->m_AmbState < ARSTATE_FIXED) pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FLOAT;

            if (pStruct->m_SatStatus[IROVE][prn].AR_Status[f] < ARSTATE_FIXED) continue;
            if (pStruct->m_SatStatus[IROVE][prn].AR_StatusPre[f] < ARSTATE_FIXED) continue;

            CommonPrns[nCommonPrns++] = prn;
        }

        if (nCommonPrns < 2)continue;

        max_base = max_pos = 0;

        // 寻找一个共同基准星,使得前后固定的模糊度相等个数最多
        for (j = 0; j < nCommonPrns; j++)
        {
            prn_base = CommonPrns[j];
            nbase = 0;

            for (i = 0; i < nCommonPrns; i++)
            {
                prn = CommonPrns[i];
                if (prn == prn_base)continue;
                iL[0] = pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] - pStruct->m_SatStatus[IROVE][prn_base].AR_Amb[f];
                iL[1] = pStruct->m_SatStatus[IROVE][prn].AR_AmbPre[f] - pStruct->m_SatStatus[IROVE][prn_base].AR_AmbPre[f];
                if (iL[0] == iL[1]) nbase++;
            }

            if (nbase > max_base)
            {
                max_base = nbase;
                max_pos = j;
            }
        }

        // 得到基准星
        prn_base = CommonPrns[max_pos];
        pStruct->m_SatStatus[IROVE][prn_base].AR_nFixed[f]++;

        for (i = 0; i < nCommonPrns; i++)
        {
            prn = CommonPrns[i];
            if (prn == prn_base)continue;
            iL[0] = pStruct->m_SatStatus[IROVE][prn].AR_Amb[f] - pStruct->m_SatStatus[IROVE][prn_base].AR_Amb[f];
            iL[1] = pStruct->m_SatStatus[IROVE][prn].AR_AmbPre[f] - pStruct->m_SatStatus[IROVE][prn_base].AR_AmbPre[f];
            if (iL[0] == iL[1] && iL[0] != 0.0) // 参考星相减为0
            {
                pStruct->m_SatStatus[IROVE][prn].AR_nFixed[f]++;

                if (pStruct->m_SatStatus[IROVE][prn].AR_nFixed[f] >= 0)
                {
                    pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_FIXED; // 满足Hold条件
                    nFixAmb++;
                }
                if (pStruct->m_SatStatus[IROVE][prn].AR_nFixed[f] >= pStruct->m_pGNSSOpt->m_ARHold_nFixEpoch)
                {
                    pStruct->m_SatStatus[IROVE][prn].AR_Status[f] = ARSTATE_HOLD; // 满足Hold条件
                    //printf("%3d,%d,%3d\n", prn, f, iL[0]);
                    if (HoldSats[prn - 1] == false)
                    {
                        nHoldSats++;
                        HoldSats[prn - 1] = true;
                    }
                }
            }
            else
            {
                pStruct->m_SatStatus[IROVE][prn].AR_nFixed[f] = 0;
            }
        }
    }

    pStruct->m_nSatARHold = nHoldSats;

    // 2. 清空前一历元状态
    for (f = 0; f < NFREQ; f++)
    {
        if (pStruct->m_bFreqUsed[f] == false) continue;

        for (i = 0; i < pStruct->m_nCommonPrnsPre; i++)
        {
            prn = pStruct->m_CommonPrnsPre[i];

            pStruct->m_SatStatus[IROVE][prn].AR_StatusPre[f] = ARSTATE_NONE;
            pStruct->m_SatStatus[IROVE][prn].AR_AmbPre[f] = 0;
        }
    }

    // 3. 更新当前状态
    pStruct->m_nCommonPrnsPre = 0;
    for (i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
    {
        if (pStruct->m_pGLSInfo->MEQ[i].bMEQ == false) continue;

        prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;
        pStruct->m_CommonPrnsPre[pStruct->m_nCommonPrnsPre++] = prn;

        for (f = 0; f < NFREQ; f++)
        {
            if (pStruct->m_bFreqUsed[f] == false) continue;

            // 不能使用pStruct->m_iAmb_PrnFrq,它不能包含基准星
            pStruct->m_SatStatus[IROVE][prn].AR_StatusPre[f] = pStruct->m_SatStatus[IROVE][prn].AR_Status[f];
            pStruct->m_SatStatus[IROVE][prn].AR_AmbPre[f] = pStruct->m_SatStatus[IROVE][prn].AR_Amb[f];

            // 该卫星为首次进入固定,且当前没有被固定,则Lock变为-1,下次重新被标记为首次固定
            if (pStruct->m_AmbState >= ARSTATE_FIXED && pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] == 0 && pStruct->m_SatStatus[IROVE][prn].AR_Status[f] < ARSTATE_FIXED)
            {
                pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] = -1;
            }
        }
    }

    if (pStruct->m_bILSFixed == true && /*nFixAmb*/pStruct->m_nSatARHold < pStruct->m_pGNSSOpt->m_AR_nSat) // 只在ILS时对其进行判断
    {
        pStruct->m_AmbState = ARSTATE_FLOFIX;
        return false;
    }

    return true;
}


// for test，执行双频模糊度一致性检验，未通过放大固定解方差
static bool AnalyzeDFAmb(CGNSSIAR* pStruct, int nAmb, bool* bAmb, double* var)
{
    // 输入为m_iAmb_PrnFrq的索引
    double lam, ddGs[NSATMAX][NFREQ], ddG, dFAmb;
    int f, prn, rprn, isys, rprns[NFREQ];
    bool bCheck[NSYS][NFREQ], bPass = true;
    memset(ddGs, 0, sizeof(double) * NSATMAX * NFREQ);
    memset(bCheck, false, sizeof(bool) * NSYS * NFREQ);

    // 不同频点参考星相同才能执行一致性检验
    for (int i = 0; i < NSYS; i++)
    {
        for (int j = 0; j < NFREQ; j++)rprns[j] = pStruct->m_BasePrns[j][i];
        rprn = FindMostFrequent(rprns, NFREQ);
        for (int j = 0; j < NFREQ; j++)
        {
            if (rprn > 0 && rprn == pStruct->m_BasePrns[j][i])
                bCheck[i][j] = true;
        }
    }

    // 先计算所有卫星：双差观测值-双差模糊度，m
    for (int i = 0; i < nAmb; i++)
    {
        if (bAmb[i] == false)continue;

        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
        lam = pStruct->m_SatStatus[IROVE][prn].lam[f];
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        rprn = pStruct->m_BasePrns[f][isys]; // 参考星

        if (bCheck[isys][f] == false)continue;
        if (prn <= 0 || prn > NSATMAX || rprn <= 0 || rprn > NSATMAX)continue;

        ddG = lam * (pStruct->m_SatStatus[IROVE][prn].SD_L[f] - pStruct->m_SatStatus[IROVE][rprn].SD_L[f] - pStruct->m_FIXAmb[i]);
        ddGs[prn - 1][f] = (ddG == 0.0) ? IPS_EPSILON : ddG;
    }

    // 进行双频一致性检验
    for (int i = 0; i < nAmb; i++)
    {
        if (bAmb[i] == false)continue;

        f = pStruct->m_iAmb_PrnFrq[i] / 1000;
        prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;

        if (bCheck[isys][f] == false)continue;
        if (prn <= 0 || prn > NSATMAX)continue;

        dFAmb = GetMinMaxDif(ddGs[prn - 1], NFREQ);
        if (fabs(dFAmb) > 0.05)
        {
            bAmb[i] = false;
            //var[i] = dFAmb * dFAmb;
            bPass = false;
        }
        else
        {
            var[i] = 1e-6;
        }

        //if (fabs(dFAmb) > 0.05)
        //{
        //    int a = 1;
        //}
    }

    return bPass;
}


static bool UpdateAmbPara(CGNSSIAR* pStruct, bool bHold)
{
    if (!pStruct)return false;

    // RTK短基线用0.001和公式更新的是一样的,说明公式更新是认为固定的模糊度没有误差
    double var[MAXOBSLIM] = { 0 };
    const int nAmb = pStruct->m_nPara[2], nState = pStruct->m_pGLSInfo->nStateIndex;
    int i, j, prn, f, vAmb = 0, ai;
    bool bAmb[MAXOBSLIM], bPassDFCheck;
    SetValsB(bAmb, false, nAmb);

    if (pStruct->m_bILSFixed || pStruct->m_bBIEFixed || bHold)
    {
        for (i = 0; i < nAmb; i++)
        {
            if (bHold)
            {
                f = pStruct->m_iAmb_PrnFrq[i] / 1000;
                prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
                if (pStruct->m_SatStatus[IROVE][prn].AR_Status[f] == ARSTATE_HOLD)
                {
                    bAmb[i] = true; vAmb++;
                }
            }
            else
            {
                //if (fabs(GetGPSTIMESow(pStruct->m_pOBSData[IROVE]->gt) - 279293.0) < 0.1)
                //{
                //    f = pStruct->m_iAmb_PrnFrq[i] / 1000;
                //    prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
                //    if (prn == 76 && f == 1)
                //    {
                //        pStruct->m_FIXAmb[i] += 0.5;
                //    }
                //}
                if (/*GetFrac(pStruct->m_FIXAmb[i]) == 0.0*/pStruct->m_bAmbFixed[i])
                {
                    bAmb[i] = true; vAmb++;
                }
            }
        }

        //// for test
        bPassDFCheck = AnalyzeDFAmb(pStruct, nAmb, bAmb, var); // 双频一致性检验，超过则剔除
        if (bPassDFCheck == false)
        {
            vAmb = 0;
            for (i = 0; i < nAmb; i++)
            {
                if (bAmb[i])
                {
                    vAmb++;
                    //f = pStruct->m_iAmb_PrnFrq[i] / 1000;
                    //prn = pStruct->m_iAmb_PrnFrq[i] % 1000;
                    //printf("%3d,%d,%8.1f,%8.1f\n", prn, f, pStruct->m_SatStatus[IROVE][prn].azel[0] * R2D, pStruct->m_SatStatus[IROVE][prn].azel[1] * R2D);
                }
            }
            if (pStruct->m_bFixQuality == false)pStruct->m_AmbState = ARSTATE_FLOFIX; // 固定质量差且没通过检验，则为不信任固定解
            if (vAmb < 6)
            {
                pStruct->m_AmbState = ARSTATE_FLOAT;
                return false;
            }
        }

        // 初始化GLS估计器
        InitGLS(pStruct->m_GLS, nState, vAmb, true);

        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[i] == false) continue;
            for (j = 0; j < nState; j++)
            {
                pStruct->m_GLS->m_H[ai * nState + j] = pStruct->m_HoldD[i * nState + j];
            }

            pStruct->m_GLS->m_Inno[ai] = pStruct->m_FIXAmb[i] - pStruct->m_FLOAmb[i];
            pStruct->m_GLS->m_R[ai * vAmb + ai] = var[i];
            ai++;
        }
    }

    MatrixCopy(nState, 1, pStruct->m_HoldStateX, pStruct->m_GLS->m_StateXp);
    MatrixCopy(nState, nState, pStruct->m_HoldStateP, pStruct->m_GLS->m_StatePp);
    GLS(pStruct->m_GLS);

    if (bHold)
    {
        for (i = 0; i < nState; i++)
        {
            pStruct->m_pGLSInfo->StateX[pStruct->m_pGLSInfo->StateIndex[i]] = pStruct->m_GLS->m_StateX[i];
            for (j = 0; j < nState; j++)
            {
                pStruct->m_pGLSInfo->StateP[pStruct->m_pGLSInfo->StateIndex[i] * MAXRTKNX + pStruct->m_pGLSInfo->StateIndex[j]] = pStruct->m_GLS->m_StateP[i * nState + j];
            }
        }

        pStruct->m_AmbState = ARSTATE_HOLD;
    }
    else
    {
        for (i = 0; i < nState; i++)
        {
            pStruct->m_pGLSInfo->StateXa[pStruct->m_pGLSInfo->StateIndex[i]] = pStruct->m_GLS->m_StateX[i];
            for (j = 0; j < nState; j++)
            {
                pStruct->m_pGLSInfo->StatePa[pStruct->m_pGLSInfo->StateIndex[i] * MAXRTKNX + pStruct->m_pGLSInfo->StateIndex[j]] = pStruct->m_GLS->m_StateP[i * nState + j];
            }
        }

        //pStruct->m_AmbState = ARSTATE_FIXED;

        // 固定解检核：基线浮点解与固定解差异
        double dxyz[3] = { 0.0 };
        M31_M31(pStruct->m_pGLSInfo->StateXa, pStruct->m_pGLSInfo->StateX, dxyz);
        if (pStruct->m_bFixQuality == false)
            pStruct->m_pGNSSOpt->m_FixConsistCheck = 3.0;
        else
            pStruct->m_pGNSSOpt->m_FixConsistCheck = 5.0;
        if (MatrixNorm2(3, 1, dxyz) > pStruct->m_pGNSSOpt->m_FixConsistCheck && pStruct->m_pGNSSOpt->m_FixConsistCheck > 0.0)
        {
            pStruct->m_AmbState = ARSTATE_FLOFIX; // 最后一步固定解检核，即使不通过也让它作为较差固定解输出
        }

        // For logging
        if (pStruct->m_pGNSSSol && pStruct->m_pGNSSSol->m_bUseLOG)
        {
            pStruct->m_pGNSSSol->m_LOG.fixPosDif = MatrixNorm2(3, 1, dxyz);
            //pStruct->m_pGNSSSol->m_LOG.fixAmbRms = xsqrt(AmbRms);
        }
    }

    return true;
}


static bool SelectBaseSat_WL(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    SetValsI(pStruct->m_BasePrns[0], 0, NFREQ * NSYS);

    double var = 0.0, el = 0.0, elevs[NSYS] = { 0.0 };
    int prn = 0, isys = 0;

    elevs[ISYSBD2] = 999999999.0;
    elevs[ISYSBD3] = 999999999.0;

    for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
    {
        prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

        // 该卫星必须是可固定
        if (pStruct->m_Prn_iAmb_WL[prn - 1] < 0) continue;

        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        el = pStruct->m_SatStatus[IROVE][prn].azel[1];

        // 选择高度角最高的一颗卫星
        if (isys == ISYSBD2 || isys == ISYSBD3)
        {
            // BDS卫星的参考星选取根据方差来
            var = SelectBaseSat_BDSVar(pStruct->m_pGNSSOpt, prn, el);
            if (elevs[isys] > var)
            {
                elevs[isys] = var;
                pStruct->m_BasePrns[0][isys] = prn;
            }
        }
        else
        {
            if (elevs[isys] < el)
            {
                elevs[isys] = el;
                pStruct->m_BasePrns[0][isys] = prn;
            }
        }
    }

    return true;
}


#define NIARExcIndWL  3
static bool BuildPARIndex_WL(CGNSSIAR* pStruct, int iFrq[2])
{
    if (!pStruct)return false;

    int prn, i, j, k, iind[NIARExcIndWL] = { 0 }, iPARIndex = 0;
    float ind[NIARExcIndWL][MAXOBSLIM];
    double val = 0.0;

    SetValsI(pStruct->m_PARIndex[0], 0, MAXOBSLIM);

    for (i = 0; i < pStruct->m_niAmb_Prn_WL; i++)
    {
        prn = pStruct->m_iAmb_Prn_WL[i];
        if (pStruct->m_Prn_iAmb_WL[prn - 1] < 0)continue;
        pStruct->m_PARIndex[0][iPARIndex++] = pStruct->m_Prn_iAmb_WL[prn - 1];

        // 1. 小数部分大于0.3
        if (pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[0])
        {
            ind[0][iind[0]++] = 1;
        }
        else
        {
            ind[0][iind[0]++] = 0;
        }

        // 2. 周跳
        if (pStruct->m_SatStatus[IROVE][prn].CS_Type[iFrq[0]] || pStruct->m_SatStatus[IROVE][prn].CS_Type[iFrq[1]])
        {
            ind[1][iind[1]++] = 1;
        }
        else
        {
            ind[1][iind[1]++] = 0;
        }

        // 3. 高度角
        val = (pStruct->m_SatStatus[IROVE][prn].azel[1] > 0) ? (PI / pStruct->m_SatStatus[IROVE][prn].azel[1]) : 1.0;
        ind[2][iind[2]++] = (float)val;
    }

    const int n = iPARIndex;

    for (i = 0; i < n; i++)
    {
        k = i;
        for (j = i + 1; j < n; j++)
        {
            for (int p = 0; p < NIARExcIndWL; p++)
            {
                if (iind[p] == 0) continue;
                if (ind[p][j] == ind[p][k]) continue; // 相等就不比较,等待下一个
                if (ind[p][j] > ind[p][k]) k = j;     // j大,将j赋给k
                break;
            }
        }

        // 此时k为最大,进行交换
        if (k != i)
        {
            for (int p = 0; p < NIARExcIndWL; p++)
            {
                if (iind[p] == 0) continue;
                SwapF(&ind[p][i], &ind[p][k]);
            }
            SwapI(&pStruct->m_PARIndex[0][i], &pStruct->m_PARIndex[0][k]);
        }
    }

    return true;
}


static bool FixWL(CGNSSIAR* pStruct, int iFrq[2], int mode)
{
    if (!pStruct)return false;

    double val = pStruct->m_pGLSInfo->StateP[0] + pStruct->m_pGLSInfo->StateP[MAXRTKNX + 1] + pStruct->m_pGLSInfo->StateP[2 * MAXRTKNX + 2];
    if (sqrt(val) > 0.2)
        return false;
    if (pStruct->m_bFixWL == false)return false;
    if (pStruct->m_pGLSInfo->nStateIndex <= pStruct->m_pGLSInfo->IAmb)return false;

    /// 1. 确定哪些卫星能组要求的组合观测值
    const int n = pStruct->m_pGLSInfo->nStateIndex;
    int i, j, f, prn, sys, isys, nValidWL = 0, Xk = 0, Hi = 0, Bi = 0;
    char ValidPRNs[NSATMAX] = { 0 }; // 用于标记宽巷卫星号，标识为2的即为可组宽巷卫星

    // 对各个卫星是否可以进入固定解并组成宽巷观测值做初步判定
    for (i = 0; i < n; i++)
    {
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false)   continue;   // 非模糊度参数
        if (pStruct->m_SatStatus[IROVE][prn].OBSValid[OBS_LI][f] == false)                  continue;   // 未参与浮点解算
        if (sys == SYSGLO) continue;                                                                    // GLO不固定    
        if (pStruct->m_SatStatus[IROVE][prn].azel[1] < 10.0 * D2R)		                    continue;   // 高度角限定
        //if (pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] < 0)					            continue;   // 跟踪历元数限定
        //if (pStruct->m_SatStatus[IROVE][prn].lock[f] <= 1)					            continue;   // 跟踪历元数限定

        if (f == iFrq[0] || f == iFrq[1])
        {
            if (++ValidPRNs[prn - 1] == 2)nValidWL++; // 可组宽巷卫星数
        }
    }

    if (nValidWL < 4)return false;

    /// 2. 构建转换矩阵，非组合基线+模糊度->组合浮点模糊度，即StateX_SWL=pStruct->m_HoldT*pStruct->m_HoldStateX
    SetValsD(pStruct->m_HoldStateX, 0, MAXRTKNX);
    SetValsD(pStruct->m_HoldStateP, 0, MAXRTKNX * MAXRTKNX);
    double HoldT[MAXOBSLIM * MAXRTKNX] = { 0.0 };
    double StateX_SWL[MAXOBSLIM] = { 0.0 };
    double StateP_SWL[MAXOBSLIM * MAXOBSLIM] = { 0.0 };

    // 得到各卫星在StateX_SWL中的索引，即第几行
    SetValsC(pStruct->m_Prn_iAmb_WL, -1, NSATMAX);
    SetValsI(pStruct->m_iAmb_Prn_WL, 0, MAXOBSLIM);
    pStruct->m_niAmb_Prn_WL = 0;
    for (i = 0; i < n; i++)
    {
        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false)continue;
        if (ValidPRNs[prn - 1] != 2) continue;
        if (f == iFrq[0])pStruct->m_Prn_iAmb_WL[prn - 1] = Hi++; // 防止重复加，所以只判断iFrq[0]
    }

    // 转换矩阵和浮点模糊度及其方差
    for (i = 0; i < n; i++)
    {
        pStruct->m_HoldStateX[i] = pStruct->m_pGLSInfo->StateX[pStruct->m_pGLSInfo->StateIndex[i]];
        for (j = 0; j < n; j++) pStruct->m_HoldStateP[i * n + j] = pStruct->m_pGLSInfo->StateP[pStruct->m_pGLSInfo->StateIndex[i] * MAXRTKNX + pStruct->m_pGLSInfo->StateIndex[j]];

        if (getFPS(pStruct, pStruct->m_pGLSInfo->StateIndex[i], &f, &prn, &sys) == false) { Bi++; continue; }
        if (pStruct->m_Prn_iAmb_WL[prn - 1] < 0) { Bi++; continue; }
        if (f != iFrq[0] && f != iFrq[1]) { Bi++; continue; }

        HoldT[pStruct->m_Prn_iAmb_WL[prn - 1] * n + Bi] = (f == iFrq[0]) ? 1.0 : -1.0;
        Bi++;
        if (f == iFrq[0])pStruct->m_iAmb_Prn_WL[pStruct->m_niAmb_Prn_WL++] = prn; // 防止重复加，所以只判断iFrq[0]
    }

    MatrixMultiply(nValidWL, n, HoldT, n, 1, pStruct->m_HoldStateX, StateX_SWL, 1.0);
    MatrixMultiply_HPHT(nValidWL, n, HoldT, pStruct->m_HoldStateP, false, StateP_SWL);

    /// 3. 选取参考星
    if (SelectBaseSat_WL(pStruct) == false) return false;

    /// 4. 对模糊度是否能固定进行初步判断
    int prn_base = 0, ambLoc = 0, ambLoc_base = 0, iNL = 0, BaseLocs[NSYS] = { -1 };
    double NL_none = 0.0, NL_base = 0.0, lam = 0.0, dNL = 0, vNL = 0, sr = 0;
    pStruct->m_nPara[0] = pStruct->m_nPara[1] = pStruct->m_nPara[2] = pStruct->m_nPara[3] = 0;
    SetValsC(ValidPRNs, 0, NSATMAX);
    for (i = 0; i < pStruct->m_niAmb_Prn_WL; i++)
    {
        prn = pStruct->m_iAmb_Prn_WL[i];
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        prn_base = pStruct->m_BasePrns[0][isys];
        if (prn_base <= 0)   continue;
        if (prn == prn_base)
        {
            pStruct->m_nPara[1]++; // 基准星个数
            BaseLocs[isys] = i;
            continue;
        }
        // 参考星的L1(单位周)
        ambLoc = pStruct->m_Prn_iAmb_WL[prn - 1];
        ambLoc_base = pStruct->m_Prn_iAmb_WL[prn_base - 1];
        NL_none = StateX_SWL[ambLoc];
        NL_base = StateX_SWL[ambLoc_base];

        dNL = NL_none - NL_base;
        iNL = RoundNum(dNL);
        dNL = dNL - iNL;
        vNL = StateP_SWL[ambLoc_base * nValidWL + ambLoc_base] +
              StateP_SWL[ambLoc * nValidWL + ambLoc] -
              2.0 * StateP_SWL[ambLoc_base * nValidWL + ambLoc];
        vNL = xsqrt(vNL);
        sr = SR_Rounding(dNL, vNL);

        // 宽巷固定条件1:WL的std
        if (vNL > 10.0) continue;

        // 宽巷固定条件2:WL的小数部分不超过0.3c
        if (fabs(dNL) > 0.3)continue;
        /*pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[0] = true;
        else pStruct->m_SatStatus[IROVE][prn].AR_AmbCtrl.bBad[0] = false;*/

        // 宽巷固定条件3:WL的直接取整的成功率不小于0.9
        //if (sr < 0.9) continue;

        ValidPRNs[prn - 1] = 1;  // 注意，PRN进去后按大小排序，所以并不和模糊度索引对应
        pStruct->m_nPara[2]++;   // 有效的单差/双差模糊度个数
    }

    if (pStruct->m_nPara[2] < 4)return false;

    /// 5. 构建转换矩阵，单差组合模糊度->双差组合模糊度，即StateX_SWL->pStruct->m_FLOAmb
    SetValsD(pStruct->m_HoldD, 0, MAXOBSLIM* MAXRTKNX);
    SetValsD(pStruct->m_FLOAmb, 0, MAXOBSLIM);
    SetValsD(pStruct->m_FLOAmbP, 0, MAXOBSLIM* MAXOBSLIM);
    SetValsD(pStruct->m_FIXAmb, 0, MAXOBSLIM);
    Hi = Bi = 0;
    SetValsC(pStruct->m_Prn_iAmb_WL, -1, NSATMAX);
    for (i = 0; i < pStruct->m_niAmb_Prn_WL; i++)
    {
        prn = pStruct->m_iAmb_Prn_WL[i];
        if (ValidPRNs[prn - 1] == 0) { Bi++; continue; }
        pStruct->m_Prn_iAmb_WL[prn - 1] = Hi; // 建立了送入固定模糊度与PRN号的索引关系
        isys = pStruct->m_SatStatus[IROVE][prn].sys_id;
        prn_base = pStruct->m_BasePrns[0][isys];
        pStruct->m_HoldD[Hi * nValidWL + Bi] = 1.0;
        pStruct->m_HoldD[Hi * nValidWL + BaseLocs[isys]] = -1.0;
        //printf("%3d,%d\n", prn, Hi);
        Hi++;
        Bi++;
    }

    MatrixMultiply(pStruct->m_nPara[2], nValidWL, pStruct->m_HoldD, nValidWL, 1, StateX_SWL, pStruct->m_FLOAmb, 1.0);
    MatrixMultiply_HPHT(pStruct->m_nPara[2], nValidWL, pStruct->m_HoldD, StateP_SWL, false, pStruct->m_FLOAmbP);

    /// 6. 模糊度固定
    int nAmb = pStruct->m_nPara[2], vAmb = pStruct->m_nPara[2], ai, aj, exi = 0, k;
    bool bAmb[MAXOBSLIM];
    SetValsB(bAmb, true, MAXOBSLIM);
    bool bFix = false;
    pStruct->m_ILS.m_method = 6;
    pStruct->m_ILS.m_FTThresBound[0] = 3.0;
    pStruct->m_ILS.m_FTThresBound[1] = 5.0;

    BuildPARIndex_WL(pStruct, iFrq);

    while (vAmb >= 4)
    {
        InitLambda(&pStruct->m_ILS, vAmb);
        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[i])
            {
                for (j = aj = 0; j < nAmb; j++)
                {
                    if (bAmb[j]) { pStruct->m_ILS.m_QFloat[ai * vAmb + aj] = pStruct->m_FLOAmbP[i * nAmb + j]; aj++; }
                }
                pStruct->m_ILS.m_Float[ai] = pStruct->m_FLOAmb[i];  ai++;
            }
        }
        bFix = LAMBDA(&pStruct->m_ILS, false);
        if (!bFix)
        {
            // 否则进入迭代剔除卫星
            k = pStruct->m_PARIndex[0][exi];
            bAmb[k] = false;
            vAmb--; exi++;
        }
        else break;
    }

    /// 7. 固定解更新
    const int nState = pStruct->m_pGLSInfo->nStateIndex;
    double HoldTD[MAXOBSLIM * MAXRTKNX] = { 0.0 };
    MatrixMultiply(pStruct->m_nPara[2], nValidWL, pStruct->m_HoldD, nValidWL, n, HoldT, HoldTD, 1.0);

    pStruct->m_AmbState_WL[mode] = ARSTATE_FLOAT;
    if (bFix == true)
    {
        InitGLS(pStruct->m_GLS, nState, vAmb, true);

        for (i = ai = 0; i < nAmb; i++)
        {
            if (bAmb[i] == false) continue;
            for (j = 0; j < nState; j++)
            {
                pStruct->m_GLS->m_H[ai * nState + j] = HoldTD[i * n + j];
            }
            pStruct->m_GLS->m_Inno[ai] = pStruct->m_ILS.m_afixed[ai] - pStruct->m_FLOAmb[i]; // m_afixed可能有问题
            pStruct->m_GLS->m_R[ai * vAmb + ai] = 1e-6;
            ai++;
        }

        MatrixCopy(nState, 1, pStruct->m_HoldStateX, pStruct->m_GLS->m_StateXp);
        MatrixCopy(nState, nState, pStruct->m_HoldStateP, pStruct->m_GLS->m_StatePp);
        GLS(pStruct->m_GLS);
 
        // 备份
        EQU_StateXP(pStruct->m_pGLSInfo->StateXa, pStruct->m_pGLSInfo->StatePa, pStruct->m_pGLSInfo->MKFStateX, pStruct->m_pGLSInfo->MKFStateP, MAXRTKNX);

        // 更新
        for (i = 0; i < nState; i++)
        {
            pStruct->m_pGLSInfo->StateXa[pStruct->m_pGLSInfo->StateIndex[i]] = pStruct->m_GLS->m_StateX[i];
            for (j = 0; j < pStruct->m_pGLSInfo->nStateIndex; j++)
            {
                pStruct->m_pGLSInfo->StatePa[pStruct->m_pGLSInfo->StateIndex[i] * MAXRTKNX + pStruct->m_pGLSInfo->StateIndex[j]] = pStruct->m_GLS->m_StateP[i * nState + j];
            }
        }

        // 检验
        double dxyz[3] = { 0.0 }, ambRMS = 0.0;
        M31_M31(pStruct->m_pGLSInfo->StateXa, pStruct->m_pGLSInfo->MKFStateX, dxyz);
        for (i = 0; i < vAmb; i++)ambRMS += pStruct->m_GLS->m_Inno[i] * pStruct->m_GLS->m_Inno[i];
        ambRMS /= vAmb;
        if (MatrixNorm2(3, 1, dxyz) < 3.0 && xsqrt(ambRMS) < 0.3) pStruct->m_AmbState_WL[mode] = ARSTATE_FIXED;
        else EQU_StateXP(pStruct->m_pGLSInfo->MKFStateX, pStruct->m_pGLSInfo->MKFStateP, pStruct->m_pGLSInfo->StateXa, pStruct->m_pGLSInfo->StatePa, MAXRTKNX);

        //// 检验PDOP
        //double azel[2][NSATMAX + 1] = { 0.0 }, DOPs[4] = { 0.0 };
        //map<int, int>::iterator it;
        //for (it = pStruct->m_Prn_iAmb_WL.begin(); it != pStruct->m_Prn_iAmb_WL.end(); it++)
        //{
        //    prn = it->first;
        //    i = pStruct->m_Prn_iAmb_WL[prn];
        //    if (bAmb[i] == false) continue;
        //    azel[0][prn] = pStruct->m_SatStatus[IROVE][prn].azel[0];
        //    azel[1][prn] = pStruct->m_SatStatus[IROVE][prn].azel[1];
        //}
        //ComputeDOP(azel[0], azel[1], pStruct->m_pGNSSOpt->pStruct->m_ElevMask, DOPs);
        //if (fp11)fprintf(fp11, "%8.1f,%d,%2d,%2d,%8.1f,%8.1f,%8.3f,%8.3f\n", _TOW(pStruct->m_pOBSData[IROVE]->gt), mode, nAmb, vAmb,
        //    pStruct->m_ILSs.pStruct->m_sqnorm.x[1] / pStruct->m_ILSs.pStruct->m_sqnorm.x[0], DOPs[2], MatrixNorm2(3, 1, dxyz), xsqrt(ambRMS));
    }

    return true;
}


static bool runSubIAR(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    // 1. 验证模糊度,计算模糊度相关的指标,并选择参考星
    if (ValidAmbs(pStruct) == false) return false;

    // 2. 获取双差浮点模糊度及其协方差
    if (getFloatDDAmb(pStruct) == false) return false;

    // 3. 模糊度固定
    if (pStruct->m_bUseILS)runPAR(pStruct); // 执行ILS
    if (pStruct->m_bUseBIE)runBIE(pStruct); // 执行BIE
    
    if (pStruct->m_bILSFixed == false && pStruct->m_bBIEFixed == false)return false;
    UpdateAmbStatus(pStruct);

    if (pStruct->m_AmbState >= ARSTATE_FLOFIX) UpdateAmbPara(pStruct, false);

    return (pStruct->m_AmbState >= ARSTATE_FLOFIX);
}


static bool runSubIAR_Hold(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    if (pStruct->m_pGNSSOpt->m_ARMode < ARMODE_FIXHOLD) return false;

    bool flag = true;

    // 使用内置模糊度固定不进行hold
    if (pStruct->m_bILSFixed == false) flag = false;

    // 连续跟踪的卫星数
    if (pStruct->m_nSatARHold < pStruct->m_pGNSSOpt->m_ARHold_nSat) flag = false;

    // ADOP值、BootstrapSR限制
    if (pStruct->m_SR_Bootstrap < pStruct->m_pGNSSOpt->m_ARHold_SRBS || pStruct->m_ADOP > pStruct->m_pGNSSOpt->m_ARHold_ADOP || pStruct->m_ADOP < 0.0 || pStruct->m_ARRatio < pStruct->m_pGNSSOpt->m_ARHold_Ratio) 
        flag = false;

    // 模糊度稳态持续历元数归零
    if (flag == false) pStruct->m_nARHold = 0;

    // 模糊度稳态持续历元
    if (++pStruct->m_nARHold < pStruct->m_pGNSSOpt->m_ARHold_nEpoch) flag = false;

    if (flag)
    {
        UpdateAmbPara(pStruct, true);
    }

    return flag;
}


bool runOneIAR(CGNSSIAR* pStruct)
{
    if (!pStruct)return false;

    bool flag = false;

    PrepareIAR(pStruct);

    flag = runSubIAR(pStruct);

    if (flag)
    {
        runSubIAR_Hold(pStruct);
    }
    else
    {
        pStruct->m_nARHold = 0;

        //int iFrq[2];
        //iFrq[0] = 0; iFrq[1] = 2;
        //FixWL(pStruct, iFrq, 1); // 固定宽巷
        //if (pStruct->m_AmbState_WL[1] != ARSTATE_FIXED)
        //{
        //    iFrq[0] = 1; iFrq[1] = 2;
        //    FixWL(pStruct, iFrq, 0); // 固定超宽巷
        //}
    }

    // 更新状态
    pStruct->m_AmbStatePre = pStruct->m_AmbState;
    pStruct->m_bFixQualityPre = pStruct->m_bFixQuality;
    if (pStruct->m_AmbState < ARSTATE_FIXED)
    {
        pStruct->m_ARRatio = 0.0;
        pStruct->m_bFixQualityPre = false;
        /*for (int i = 0; i < pStruct->m_pOBSData[IROVE]->nsat; i++)
        {
            if (pStruct->m_pGLSInfo->MEQ[i].bMEQ == false) continue;

            int prn = pStruct->m_pOBSData[IROVE]->obs[i].prn;

            for (int f = 0; f < NFREQ; f++)
            {
                if (pStruct->m_bFreqUsed[f] == false) continue;

                // 该卫星为首次进入固定,且当前没有被固定,则Lock变为-1,下次重新被标记为首次固定
                if (pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] == 0)
                {
                    pStruct->m_SatStatus[IROVE][prn].AR_Lock[f] = -1;
                }
            }
        }*/
    }

    return true;
}