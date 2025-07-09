#include "CCycleSlipDetection.h"
#include "GNSSCmnFunc.h"
#include "BaseTime.h"
#include "BaseMath.h"


///< Constructor function, to be called when defining
void InitCCycleSlipDetection(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    pStruct->m_pOBSData = NULL;
    pStruct->m_pGNSSOpt = NULL;
    pStruct->m_pSatStatus = NULL;
    InitGPSTIME(&pStruct->m_OBSTime);
    for (int i = 0; i < NSATMAX + 1; i++)InitOBSDATAT_PL(pStruct->m_OBSDataPre + i);
    //pStruct->m_MaxOutage = 3;
}


bool InitCS(CCycleSlipDetection* pStruct, CGNSSOption* pGNSSOpt, SATSTATUS* pSatStatus, int index)
{
    if (!pStruct)return false;
    if (!pGNSSOpt || !pSatStatus) return false;

    pStruct->m_pGNSSOpt = pGNSSOpt;
    pStruct->m_pSatStatus = pSatStatus;

    double dt = pGNSSOpt->m_SampleTime[index];
    if (dt < 10e-5) return false;

    // PPPCLS时,如果某些数据算不好,可调成GF=0.025,MW=1.5
    if (true/*pStruct->m_pGNSSOpt->m_CSGFThres == 0.0*/)
    {
        double gf_thres = 0.05; // for test
        if (dt <= 1.0)        pStruct->m_GFThres = gf_thres;
        else if (dt <= 20.0)  pStruct->m_GFThres = (gf_thres) / (20.0) * dt + gf_thres;
        else if (dt <= 60.0)  pStruct->m_GFThres = gf_thres * 2.0;
        else if (dt <= 100.0) pStruct->m_GFThres = gf_thres * 4.0;
        else                  pStruct->m_GFThres = gf_thres * 7.0;
    }
    else
    {
        pStruct->m_GFThres = 0.0/*pStruct->m_pGNSSOpt->m_CSGFThres*/;
    }

    pStruct->m_GF2Thres = 0.065;


    if (true/*pStruct->m_pGNSSOpt->m_CSMWThres == 0.0*/)
    {
        double mw_thres = 2.5;
        if (dt <= 1.0)       pStruct->m_MWThres = mw_thres;
        else if (dt <= 20.0) pStruct->m_MWThres = 0.03 * mw_thres * dt + mw_thres;
        else if (dt <= 60.0) pStruct->m_MWThres = 1.6 * mw_thres;
        else                 pStruct->m_MWThres = 3.0 * mw_thres;
    }
    else
    {
        pStruct->m_MWThres = 0.0/*pStruct->m_pGNSSOpt->pStruct->m_CSMWThres*/;
    }

    return true;
}


static void RepairRcvClockJump(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    // 一旦所有模糊度重新初始化,所有CJ状态也初始化
    if (pStruct->m_bReinitCJByAmb)
    {
        pStruct->m_bReinitCJByAmb = false;
        pStruct->m_TotalClkJump = 0;
        for (int i = 1; i < NSATMAX + 1; i++)
        {
            pStruct->m_OBSDataPre[i].prn = 0;
        }
    }

    int CurrentClkJump = 0; // 当前钟跳(ms), 伪距上跳的值和钟差保持一致

    double dP1 = 0, dL1 = 0, d1 = 0, totalT = 0.0;
    int prn = 0, vsat_num = 0, cj_num = 0;
    //bool bcj_P = false; // 伪距存在钟跳

    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        OBS_DATA_t* pOBSDatai = &(pStruct->m_pOBSData->obs[i]);
        prn = pOBSDatai->prn;
        if (pStruct->m_pSatStatus[prn].bExclude) continue;

        if (pOBSDatai->P[0] == 0 || pOBSDatai->L[0] == 0)
        {
            continue;
        }

        if (pStruct->m_OBSDataPre[prn].prn == 0)
        {
            pStruct->m_OBSDataPre[prn].prn = prn;
            for (int f = 0; f < NFREQ; f++)
            {
                pStruct->m_OBSDataPre[prn].L[f] = pOBSDatai->L[f];
                pStruct->m_OBSDataPre[prn].P[f] = pOBSDatai->P[f];
            }
            continue;
        }

        vsat_num++;

        double lam = pStruct->m_pSatStatus[prn].lam[0];
        dP1 = pOBSDatai->P[0] - pStruct->m_OBSDataPre[prn].P[0];
        dL1 = pOBSDatai->L[0] - pStruct->m_OBSDataPre[prn].L[0];
        dL1 *= lam;

        d1 = dP1 - dL1;

        if (fabs(d1) > 290000)
        {
            totalT += d1;
            cj_num++;
            //if (fabs(dP1) > 290000)
                //bcj_P = true;
        }

        pStruct->m_OBSDataPre[prn].prn = prn;
        for (int f = 0; f < NFREQ; f++)
        {
            pStruct->m_OBSDataPre[prn].L[f] = pOBSDatai->L[f];
            pStruct->m_OBSDataPre[prn].P[f] = pOBSDatai->P[f];
        }
    }

    double cj_T = 0.0;

    if (vsat_num != 0 && cj_num == vsat_num)
    {
        cj_T = totalT / cj_num;

        //当所有卫星的相位观测值都发生大周跳即满足钟跳探测条件，而各卫星相位跳值大小不等时，
        //Mean_delta取均值并不能正确修复钟跳，即使如此，错加一定的周数并不影响最终定位，因为能被探测为周跳。
        double CJ_F1 = 0;//钟跳判断标记
        double CJ_F2 = 0;
        CJ_F1 = cj_T / CLIGHT * 1000;
        CJ_F2 = RoundNum(CJ_F1);//四舍五入取整

        //阈值过小(1E-5)会导致部分钟跳数据无法被探测出!!!!
        //如GENO1000.11o数据，大多数钟跳都能有效探测并修复，但少数时候算的钟跳实数值为-5.014、4.976，
        //取整后无法满足下式，而这两处经验证确实是钟跳。因此建议将阈值适当放宽为5E-2(即0.05)
        if (fabs(CJ_F1 - CJ_F2) < 5E-2)	//精度满足1E-5????
        {
            CurrentClkJump = (int)(CJ_F2);
            pStruct->m_TotalClkJump += CurrentClkJump;
        }
    }

    // 修复钟跳, 只要有钟跳累积，每次都要进行钟跳修复
    if (pStruct->m_TotalClkJump != 0)
    {
        for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
        {
            OBS_DATA_t* pOBSDatai = &(pStruct->m_pOBSData->obs[i]);
            prn = pOBSDatai->prn;
            if (pStruct->m_pSatStatus[prn].bExclude) continue;

            for (int f = 0; f < NFREQ; f++)
            {
                double lam = pStruct->m_pSatStatus[prn].lam[f];

                /* 修复相位模式 */
                if (pOBSDatai->L[f] != 0.0)
                {
                    pOBSDatai->L[f] += pStruct->m_TotalClkJump * CLIGHT / 1000.0 / lam;
                }

                /* 修复伪距模式*/
                //修复伪距就会导致接收机钟差累积,钟差参数越来越大
            }
        }
    }

    /* 将未观测到的卫星数据清零 */
    for (int i = 1, j = 0; i < NSATMAX + 1; i++)
    {
        if (pStruct->m_pSatStatus[i].bExclude) continue;

        if (pStruct->m_pOBSData->obs[j].prn == i)
        {
            if (j < pStruct->m_pOBSData->nsat - 1) j++;
            continue;
        }

        if (pStruct->m_OBSDataPre[i].prn != 0)
        {
            pStruct->m_OBSDataPre[i].prn = 0;
        }
    }
}


static void DetectCS_LLI(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    int prn = 0;
    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        OBS_DATA_t* pOBSDatai = &(pStruct->m_pOBSData->obs[i]);
        prn = pOBSDatai->prn;
        if (pStruct->m_pSatStatus[prn].bExclude) continue;

        for (int frq = 0; frq < NFREQ; frq++)
        {
            if (pOBSDatai->L[frq] == 0.0) continue;

            if (pOBSDatai->LLI[frq] & 1)
            {
                pStruct->m_pSatStatus[prn].CS_Type[frq] |= CST_LLI;

                pStruct->m_pSatStatus[prn].lock[frq] = 0;

                // 初始化GF，注意直接对frq初始化多频情况有Bug
                //pStruct->m_pSatStatus[prn].CS_dGF[frq] = 0.0;

                // 初始化MW，注意直接对frq初始化多频情况有Bug
                //pStruct->m_pSatStatus[prn].CS_bMW = false;
                //pStruct->m_pSatStatus[prn].CS_nMW[frq] = 0;
                //pStruct->m_pSatStatus[prn].CS_MW[frq] = 0.0;
                //pStruct->m_pSatStatus[prn].CS_MWVar[frq] = 0.0;
            }
        }
    }
}


static void DetectCS_GF(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    double  gf0 = 0.0, gf1 = 0.0, GF = 0.0;
    double  dgf = 0.0, ddgf = 0.0, elev = 0.0, fact = 0.0;
    double  dt = 0.0;
    double  thres1 = 0.0, thres2 = 0.0;
    double* lam = NULL;
    int lock = 0, outc = 0, prn = 0;
    bool bCS = false;

    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        OBS_DATA_t* pOBSDatai = &(pStruct->m_pOBSData->obs[i]);
        prn = pOBSDatai->prn;
        if (pStruct->m_pSatStatus[prn].bExclude) continue;

        lam = pStruct->m_pSatStatus[prn].lam;
        //lock = pStruct->m_pSatStatus[prn].lock[pStruct->m_iDF];
        int lock1 = pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[0]];
        int lock2 = pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[1]];
        lock = (lock1 > lock2) ? lock2 : lock1;

        if ((GF = TFLC_GF_L(lam, pStruct->m_iFreq, pOBSDatai->L)) == 0.0) continue;

        gf0 = pStruct->m_pSatStatus[prn].CS_GF[pStruct->m_iDF];
        gf1 = GF;
        dgf = (gf0 == 0.0) ? 0.0 : (gf1 - gf0);
        ddgf = pStruct->m_pSatStatus[prn].CS_dGF[pStruct->m_iDF];
        ddgf = (ddgf == 0.0) ? 0.0 : (dgf - ddgf);
        dt = fabs(MinusGPSTIME(pStruct->m_OBSTime, pStruct->m_pSatStatus[prn].CS_GFTime[pStruct->m_iDF]));
        outc = RoundNum(dt / pStruct->m_SampleTime);

        pStruct->m_pSatStatus[prn].CS_GF[pStruct->m_iDF] = gf1;
        pStruct->m_pSatStatus[prn].CS_GFTime[pStruct->m_iDF] = pStruct->m_OBSTime;
        //pStruct->m_pSatStatus[prn].CS_dGF[pStruct->m_iDF] = (lock > 0) ? dgf : 0.0;      // 锁定时才有dgf
        pStruct->m_pSatStatus[prn].CS_dGF[pStruct->m_iDF] = dgf;      // 锁定时才有dgf
        if (outc > pStruct->m_pGNSSOpt->m_MaxOutage) // 采样率为0.1s时，此处是否不合适
        {
            pStruct->m_pSatStatus[prn].CS_dGF[pStruct->m_iDF] = 0.0;//不管周跳修复成功与否,dgf都置0
            continue; // 间断时间太长，继续探测会造成周跳误探
        }

        elev = pStruct->m_pSatStatus[prn].azel[1];

        if (gf0 == 0.0) continue;

        // 经验周跳阈值因子
        if (elev < pStruct->m_pGNSSOpt->m_ElevMask) elev = pStruct->m_pGNSSOpt->m_ElevMask;
        elev *= R2D;
        fact = (elev >= 15.0) ? 1.0 : (-elev / 15.0 + 2.0);
        thres1 = pStruct->m_GFThres * fact;
        thres2 = pStruct->m_GF2Thres * fact;

        // BDS GEO卫星,有些数据该阈值太小了

        bCS = false;

        // 采样率小,就用时间,否则用历元数,其实应该乘上电离层变化量
        if (outc > 1)
        {
            if (pStruct->m_SampleTime < 5.0)
            {
                thres1 = thres1 * (dt + 1.0);
                thres1 = (thres1 < 0.35) ? thres1 : 0.35;
            }
            else
            {
                thres1 = thres1 * outc;
                thres1 = (thres1 < 0.35) ? thres1 : 0.35;
            }
        }

        // 整体解时,ddgf容易多判周跳,导致分段过多,结果不好,CLS只进行dgf判断
        if (ddgf == 0.0)
        {
            if (fabs(dgf) > thres1) bCS = true;
        }
        else
        {
            if (fabs(ddgf) > thres2 || fabs(dgf) > thres1 /*+ 0.15*/) bCS = true;
        }

        if (bCS)
        {
            pStruct->m_pSatStatus[prn].CS_Type[pStruct->m_iFreq[0]] |= CST_GF;
            pStruct->m_pSatStatus[prn].CS_Type[pStruct->m_iFreq[1]] |= CST_GF;
            pStruct->m_pSatStatus[prn].CS_GFNum[pStruct->m_iFreq[0]]++;
            pStruct->m_pSatStatus[prn].CS_GFNum[pStruct->m_iFreq[1]]++;
            if (pStruct->m_pSatStatus[prn].CS_GFNum[pStruct->m_iFreq[0]] > 1 || pStruct->m_pSatStatus[prn].CS_GFNum[pStruct->m_iFreq[1]] > 1)
                pStruct->m_pSatStatus[prn].CS_bGFMulti = true;

            pStruct->m_pSatStatus[prn].CS_dGF[pStruct->m_iDF] = 0.0;

            // 初始化MW
            if ((pStruct->m_pSatStatus[prn].CS_Type[pStruct->m_iFreq[0]] & CST_MW) == 0)
            {
                double MW = TFLC_MW_LP(lam, pStruct->m_iFreq, pOBSDatai->L, pOBSDatai->P);
                if (MW != 0.0)
                {
                    pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] = 1;
                    pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF] = MW;
                }
                else
                {
                    pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] = 0;
                    pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF] = 0.0;
                }
            }

            //// 未来多频处理,这里会是个bug，统一在KFPre_Amb中将lock清空
            //pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[0]] = 0;
            //pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[1]] = 0;
        }
    }
}


static void DetectCS_MW(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    int prn = 0, outc = 0, nValid = 0, nSlip = 0;
    double fact_dt = 1.0, fact = 0.0, elev = 0.0, dmw = 0.0, mw0 = 0.0, mw1 = 0.0, thres = 0.0;
    double* lam = NULL;

    //以下阈值适用于30s采样率的情况
    if (pStruct->m_SampleTime >= 29.5)
    {
        if (pStruct->m_nOutEpoch <= 2.0)	    fact_dt = 1.0;
        else if (pStruct->m_nOutEpoch <= 4.0)	fact_dt = 1.25;
        else if (pStruct->m_nOutEpoch <= 6.0)	fact_dt = 1.5;
        else							        fact_dt = 2.0;
    }

    for (int i = 0; i < pStruct->m_pOBSData->nsat; i++)
    {
        OBS_DATA_t* pOBSDatai = &(pStruct->m_pOBSData->obs[i]);
        prn = pOBSDatai->prn;
        if (pStruct->m_pSatStatus[prn].bExclude) continue;

        lam = pStruct->m_pSatStatus[prn].lam;
        nValid++;

        //// SPP探测出伪距粗差,MW肯定被误判为周跳，把周跳探测流程提前到SPP前，应暂时关闭
        //if (pStruct->m_pSPP)
        //{
        //    if (pStruct->m_pSPP->getSatStatus(prn)->OBSEvent[OBS_PI][pStruct->m_iFreq[0]] > OBSEVT_Outlier) continue;
        //    if (pStruct->m_pSPP->getSatStatus(prn)->OBSEvent[OBS_PI][pStruct->m_iFreq[1]] > OBSEVT_Outlier) continue;
        //}

        // 获取当前MW组合值, 宽巷模糊度
        double MW = TFLC_MW_LP(lam, pStruct->m_iFreq, pOBSDatai->L, pOBSDatai->P);

        if ((mw1 = MW) == 0.0) continue;

        //outc = pStruct->m_pSatStatus[prn].outc[pStruct->m_iDF];
        int out1 = pStruct->m_pSatStatus[prn].outc[pStruct->m_iFreq[0]];
        int out2 = pStruct->m_pSatStatus[prn].outc[pStruct->m_iFreq[1]];
        outc = (out1 > out2) ? out1 : out2;

        if (pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] == 0/* || outc > pStruct->m_pGNSSOpt->m_MaxOutage*/)
        {
            pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] = 1;
            pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF] = mw1;
            continue;
        }

        mw0 = pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF];
        dmw = mw1 - mw0;

        elev = pStruct->m_pSatStatus[prn].azel[1];
        if (elev < pStruct->m_pGNSSOpt->m_ElevMask) elev = pStruct->m_pGNSSOpt->m_ElevMask;
        elev *= R2D;
        fact = (elev >= 20.0) ? 1.0 : (-elev / 10.0 + 3.0);
        thres = fact * fact_dt * pStruct->m_MWThres;
        thres = (thres < 6.0) ? thres : 6.0;

        pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF]++;
        SequenceAverage(&pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF], pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF], mw1);

        // lock没锁定时,不需要探测(RTK中比较难处理,所以注释掉,其主要作用也仅仅是减少多余操作)
        if (fabs(dmw) > thres)
        {
            pStruct->m_pSatStatus[prn].CS_Type[pStruct->m_iFreq[0]] |= CST_MW;
            pStruct->m_pSatStatus[prn].CS_Type[pStruct->m_iFreq[1]] |= CST_MW;

            // 初始化MW
            pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] = 1;
            pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF] = mw1;

            // 统一在KFPre_Amb中将lock清空
            //pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[0]] = 0;
            //pStruct->m_pSatStatus[prn].lock[pStruct->m_iFreq[1]] = 0;

            nSlip++;
        }

        // 周跳太多,对其进行分析，待补充

        // 一定要放在此处初始化,为了得到所有的MW以进行异常分析
        if ((outc + pStruct->m_nOutEpoch) > pStruct->m_pGNSSOpt->m_MaxOutage/* && pStruct->m_pSatStatus[prn].CS_Fixed[pStruct->m_iDF] == 0*/)
        {
            pStruct->m_pSatStatus[prn].CS_nMW[pStruct->m_iDF] = 1;
            pStruct->m_pSatStatus[prn].CS_MW[pStruct->m_iDF] = mw1;
        }
    }
}


static void runOneCSDSub(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return;

    pStruct->m_iDF = TFLC_IFIndex1(pStruct->m_iFreq); // 得到双频组合索引号

    // TurboEdit周跳探测(先MW,后GF,当MW有异常时,GF不会干扰)
    if (pStruct->m_pGNSSOpt->m_bCS_MW)  DetectCS_MW(pStruct);
    if (pStruct->m_pGNSSOpt->m_bCS_GF)  DetectCS_GF(pStruct);
    if (pStruct->m_pGNSSOpt->m_bCS_LLI) DetectCS_LLI(pStruct);
}


bool runOneCSD(CCycleSlipDetection* pStruct)
{
    if (!pStruct)return false;

    // step 1 : 钟跳探测与修复
    if (pStruct->m_pGNSSOpt->m_bCJ) RepairRcvClockJump(pStruct);

    // step 2 : TurboEdit周跳探测
    if (NFREQ == 1)
    {
        DetectCS_LLI(pStruct);
    }
    else if (NFREQ == 2)
    {
        pStruct->m_iFreq[0] = TFLC_L1;
        pStruct->m_iFreq[1] = TFLC_L2;
        runOneCSDSub(pStruct);
    }
    else if (NFREQ == 3)
    {
        if (pStruct->m_bFreqUsed[0] && pStruct->m_bFreqUsed[1])
        {
            pStruct->m_iFreq[0] = TFLC_L1;
            pStruct->m_iFreq[1] = TFLC_L2;

            pStruct->m_iDF = TFLC_IFIndex1(pStruct->m_iFreq); // 得到双频组合索引号

            // TurboEdit周跳探测(先MW,后GF,当MW有异常时,GF不会干扰)
            if (pStruct->m_pGNSSOpt->m_bCS_MW)  DetectCS_MW(pStruct);
            if (pStruct->m_pGNSSOpt->m_bCS_GF)  DetectCS_GF(pStruct);
        }

        if (pStruct->m_bFreqUsed[0] && pStruct->m_bFreqUsed[2])
        {
            pStruct->m_iFreq[0] = TFLC_L1;
            pStruct->m_iFreq[1] = TFLC_L3;

            pStruct->m_iDF = TFLC_IFIndex1(pStruct->m_iFreq); // 得到双频组合索引号

            // TurboEdit周跳探测(先MW,后GF,当MW有异常时,GF不会干扰)
            if (pStruct->m_pGNSSOpt->m_bCS_MW)  DetectCS_MW(pStruct);
            if (pStruct->m_pGNSSOpt->m_bCS_GF)  DetectCS_GF(pStruct);
        }

        if (pStruct->m_bFreqUsed[1] && pStruct->m_bFreqUsed[2])
        {
            pStruct->m_iFreq[0] = TFLC_L2;
            pStruct->m_iFreq[1] = TFLC_L3;

            pStruct->m_iDF = TFLC_IFIndex1(pStruct->m_iFreq); // 得到双频组合索引号

            // TurboEdit周跳探测(先MW,后GF,当MW有异常时,GF不会干扰)
            if (pStruct->m_pGNSSOpt->m_bCS_MW)  DetectCS_MW(pStruct);
            if (pStruct->m_pGNSSOpt->m_bCS_GF)  DetectCS_GF(pStruct);
        }

        if (pStruct->m_pGNSSOpt->m_bCS_LLI) DetectCS_LLI(pStruct);
    }
    else
    {
        return false;
    }

    return true;
}