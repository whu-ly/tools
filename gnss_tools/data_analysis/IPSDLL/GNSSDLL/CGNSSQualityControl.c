#include "CGNSSQualityControl.h"
#include "GNSSCmnFunc.h"


///< Constructor function, to be called when defining
void InitCGNSSQualityControl(CGNSSQualityControl* pStruct)
{
    if (!pStruct)return;

    pStruct->m_pSatStatus = NULL;
    pStruct->m_pGNSSOpt = NULL;
    pStruct->m_pOBSDataRove = NULL;
    pStruct->m_pMEQ = NULL;

    for (int f = 0; f < NFREQ; f++)
        pStruct->m_bReinitAllAmb[f] = false;
    pStruct->m_PMode = -1;
}


///< Init QC
bool InitQC(CGNSSQualityControl* pStruct, int PMode, CGNSSOption* pGNSSOpt, SATSTATUS* pSatStatus, OBS_DATA* pOBSDataRove, GNSS_MEQ* pMEQ)
{
    if (!pStruct)return false;

    pStruct->m_PMode = PMode;
    pStruct->m_pGNSSOpt = pGNSSOpt;
    pStruct->m_pSatStatus = pSatStatus;
    pStruct->m_pOBSDataRove = pOBSDataRove;
    pStruct->m_pMEQ = pMEQ;
    return true;
}


static double getIGG3Weight(int PMode, bool bL, double v, double sig)
{
    double norm_v = fabs(v / sig);
    double k0 = 0, k1 = 0;
    double eqw = 0.0;

    if (PMode == 1 || PMode == 3)
    {
        k0 = bL ? 1.25 : 1.0;
        k1 = 5;
    }

    if (PMode == 2 || PMode == 4)
    {
        k0 = bL ? 1.75 : 3.0;
        k1 = bL ? 5.0 : 6.0;
    }

    if (norm_v <= k0)
    {
        eqw = 1.0;
    }
    else if (norm_v <= k1)
    {
        eqw = norm_v * (k1 - k0) * (k1 - k0) / (k0 * (k1 - norm_v) * (k1 - norm_v));
    }
    else
    {
        //	eqw = maxfactor;
        eqw = 1E5;
    }

    return eqw;
}


static bool AnalyzePosterResi3(CGNSSQualityControl* pStruct)
{
    if (!pStruct)return false;
    if (pStruct->m_PMode < 0) return true;

    bool flag = true;
    int f = 0, prn = 0, OBSIndex = 0;
    double eqw = 0.0, a0L = 1.75, a0P = 3.0;

    // 处理相位
    for (int i = 0; i < pStruct->m_nvL; i++)
    {
        f = pStruct->m_Lindex[i] / 1000;
        prn = pStruct->m_Lindex[i] % 1000;
        OBSIndex = pStruct->m_pSatStatus[prn].OBSIndex;

        if (fabs(pStruct->m_normv_L[i]) <= a0L) continue;
        if (fabs(pStruct->m_v_L[i]) > 0.1)
        {
            flag = false;

            // 如果原先没有周跳,那么当前就认为,通过残差分析发现了新周跳
            if (pStruct->m_pSatStatus[prn].CS_Type[f] == 0) pStruct->m_bFoundNewCS = true;

            pStruct->m_pSatStatus[prn].CS_Type[f] |= CST_Resi;

            pStruct->m_pMEQ[OBSIndex].AmbFlag[f] = 9;

            // 模糊度初始化后,观测值权阵置为1
            pStruct->m_pMEQ[OBSIndex].RFactLCur[f] = 1.0;
            pStruct->m_pMEQ[OBSIndex].RFactL[f] = 1.0;
        }
    }

    // 处理伪距
    for (int i = 0; i < pStruct->m_nvP; i++)
    {
        f = pStruct->m_Pindex[i] / 1000;
        prn = pStruct->m_Pindex[i] % 1000;
        OBSIndex = pStruct->m_pSatStatus[prn].OBSIndex;

        if (fabs(pStruct->m_normv_P[i]) <= a0P) continue;
        flag = false;

        eqw = getIGG3Weight(pStruct->m_PMode, false, pStruct->m_normv_P[i], 1.0);
        if (eqw > 1.0)
        {
            pStruct->m_pMEQ[OBSIndex].RFactPCur[f] = eqw;
            pStruct->m_pMEQ[OBSIndex].RFactP[f] *= eqw;
            if (eqw > 225.0) pStruct->m_pMEQ[OBSIndex].RFactP[f] = 225;
        }
    }

    return flag;
}


bool runAnalyzeResi(CGNSSQualityControl* pStruct, bool bPosteriori)
{
    if (!pStruct)return false;
    if (!pStruct->m_pGNSSOpt || !pStruct->m_pSatStatus || !pStruct->m_pMEQ)return false;

    pStruct->m_bFoundNewCS = false;

    // 只在验后残差分析时初始化
    if (bPosteriori)
    {
        for (int f = 0; f < NFREQ; f++)
        {
            for (int prn = 0; prn <= NSATMAX; prn++)
            {
                int skip = SkipPrn(pStruct->m_pGNSSOpt->m_SatSYS, pStruct->m_pSatStatus[prn].sys, &prn);
                if (skip == 1) continue;
                else if (skip == 2) break;

                if (pStruct->m_pSatStatus[prn].OBSIndex >= 0)
                {
                    const int OBSIndex = pStruct->m_pSatStatus[prn].OBSIndex;

                    // 有时候第一次的位置不准,验前残差有可能不准
                    // 因此, 前几次可以把验前残差置零
                    if (pStruct->m_MIKF < 3 && pStruct->m_MIKF > 0)
                    {
                        //pStruct->m_pSatStatus[prn].OBSEvent[OBS_PI][f] &= ~(OBSEVT_Inno_v); // 若设置验前粗差伪距不参与解算，这里需注释
                        pStruct->m_pSatStatus[prn].OBSEvent[OBS_LI][f] &= ~(OBSEVT_Inno_v);
                        pStruct->m_pSatStatus[prn].OBSEvent[OBS_DI][f] &= ~(OBSEVT_Inno_v);
                    }

                    pStruct->m_pMEQ[OBSIndex].AmbFlag[f] = 0;
                }
            }
        }

        return AnalyzePosterResi3(pStruct);
    }

    return true;
}


