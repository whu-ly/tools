#include "CLSEstimator.h"
#include "BaseMath.h"
#include "BaseMatrix.h"


void InitCLSEstimator(CLSEstimator* pStruct)
{
    if (!pStruct)return;

    pStruct->m_nState = 0;
    pStruct->m_nMeas = 0;
    pStruct->m_bPDiagonal = false;
    pStruct->m_bInit = false;
    pStruct->m_Sigma = 1.0;
    pStruct->m_SigmaPre = 1.0;
    pStruct->m_RMS = 0.0;
}


bool InitLSQ(CLSEstimator* pStruct, int nState, int nMeas, bool bPDiagonal)
{
    if (!pStruct)return false;

    SetValsD(pStruct->m_StateX, 0, nState);
    SetValsD(pStruct->m_StateP, 0, nState * nState);
    SetValsD(pStruct->m_B, 0, nMeas * nState);
    SetValsD(pStruct->m_P, 0, nMeas * nMeas);
    SetValsD(pStruct->m_L, 0, nMeas);
    SetValsD(pStruct->m_W, 0, nState);
    SetValsD(pStruct->m_V, 0, nMeas);
    
    pStruct->m_nState = nState;
    pStruct->m_nMeas = nMeas;
    pStruct->m_bPDiagonal = bPDiagonal;
    pStruct->m_bInit = true;

    return true;
}


void ClearLSQ(CLSEstimator* pStruct)
{
    if (!pStruct)return;

    SetValsD(pStruct->m_B, 0, MAXOBSLIM * MAXLSQNX);
    SetValsD(pStruct->m_P, 0, MAXOBSLIM * MAXOBSLIM);
    SetValsD(pStruct->m_L, 0, MAXOBSLIM);
}


bool LSQ(CLSEstimator* pStruct, bool bPostEst)
{
    if (!pStruct)return false;
    if (pStruct->m_bInit == false)return false;

    MatrixMultiply_BTPB(pStruct->m_nMeas, pStruct->m_nState, pStruct->m_B, pStruct->m_P, pStruct->m_bPDiagonal, pStruct->m_StateP);
    MatrixMultiply_BTPL(pStruct->m_nMeas, pStruct->m_nState, pStruct->m_B, pStruct->m_P, pStruct->m_L, pStruct->m_bPDiagonal, pStruct->m_W);

    if (MatrixInvSP(pStruct->m_nState, pStruct->m_nState, pStruct->m_StateP) == false)
    {
        return false;
    }

    MatrixMultiply(pStruct->m_nState, pStruct->m_nState, pStruct->m_StateP, pStruct->m_nState, 1, pStruct->m_W, pStruct->m_StateX, 1.0);

    MatrixMultiply(pStruct->m_nMeas, pStruct->m_nState, pStruct->m_B, pStruct->m_nState, 1, pStruct->m_StateX, pStruct->m_V, 1.0);
    MatrixSubtraction2(pStruct->m_nMeas, 1, false, pStruct->m_L, pStruct->m_V, 1.0, 1.0);

    if (bPostEst)
    {
        pStruct->m_Sigma = 1.0;
        pStruct->m_RMS = 0.0;

        if (pStruct->m_nMeas > pStruct->m_nState)
        {
            MatrixMultiply_BTPB(pStruct->m_nMeas, 1, pStruct->m_V, pStruct->m_P, pStruct->m_bPDiagonal, &pStruct->m_Sigma);
            pStruct->m_Sigma = pStruct->m_Sigma / (pStruct->m_nMeas - pStruct->m_nState);
            if (pStruct->m_Sigma < 1.0) pStruct->m_Sigma = 1.0; // sigma不要小于1.0
            MatrixScaleMultiply(pStruct->m_Sigma, pStruct->m_nState, pStruct->m_nState, pStruct->m_StateP);
            MatrixScaleMultiply(1.0 / pStruct->m_Sigma, pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_P);

            pStruct->m_RMS = MatrixDot2(pStruct->m_nMeas, 1, pStruct->m_V, pStruct->m_V);
            pStruct->m_RMS = xsqrt(pStruct->m_RMS / pStruct->m_nMeas);
        }
        else
        {
            MatrixScaleMultiply(pStruct->m_SigmaPre, pStruct->m_nState, pStruct->m_nState, pStruct->m_StateP);
            MatrixScaleMultiply(1.0 / pStruct->m_SigmaPre, pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_P);
        }

        if (pStruct->m_nMeas > pStruct->m_nState) pStruct->m_SigmaPre = pStruct->m_Sigma;
        pStruct->m_Sigma = xsqrt(pStruct->m_SigmaPre);
    }

    return true;
}








// ************************************************************************************************************************************** //
//  previous class: CPPPointBase ;     next class : CPPPointIFLC                                                                          //
// ************************************************************************************************************************************** //




void InitCGLSEstimator(CGLSEstimator* pStruct)
{
    if (!pStruct)return;

    pStruct->m_bInit = false;
    pStruct->m_bMDB = false;
    pStruct->m_bPDiagonal = false;
    pStruct->m_bInnoChisq = false;
    pStruct->m_bResiChisq = false;
    pStruct->m_nState = 0;
    pStruct->m_nMeas = 0;
}


void ClearGLS(CGLSEstimator* pStruct)
{
    if (!pStruct)return;

    SetValsD(pStruct->m_H, 0, pStruct->m_nMeas * pStruct->m_nState);
    SetValsD(pStruct->m_R, 0, pStruct->m_nMeas * pStruct->m_nMeas);
    SetValsD(pStruct->m_Inno, 0, pStruct->m_nMeas);
}


bool InitGLS(CGLSEstimator* pStruct, int nState, int nMeas, bool bPDiagonal)
{
    if (!pStruct)return false;

    ClearGLS(pStruct);

    pStruct->m_nState = nState;
    pStruct->m_nMeas = nMeas;
    pStruct->m_bPDiagonal = bPDiagonal;
    pStruct->m_bInit = true;
    pStruct->m_InnoChisq = 0.0;
    pStruct->m_ResiChisq = 0.0;
    pStruct->m_InnoSigma0 = 0.0;
    pStruct->m_ResiSigma0 = 0.0;
    pStruct->m_RMS = 0.0;

    return true;
}


bool GLS(CGLSEstimator* pStruct)
{
    if (!pStruct)return false;
    if (pStruct->m_bInit == false)return false;
    pStruct->m_bInit = false;

    int i = 0;

    // 对称化
    MatrixSymmetrization(pStruct->m_nState, pStruct->m_nState, pStruct->m_StatePp);

    // Rp = H*Pp*HT + R
    MatrixMultiply_HPHT(pStruct->m_nMeas, pStruct->m_nState, pStruct->m_H, pStruct->m_StatePp, false, pStruct->m_Rp);

    // 对称化
    MatrixAddition2(pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_R, pStruct->m_Rp, 1.0, 1.0);
    MatrixSymmetrization(pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_Rp);

    if (MatrixInvSP(pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_Rp) == false)
    {
        return false;
    }

    double HT[MAXGLSNX * MAXGLSNX]; // 后面要复用给M，所以定义大一点
    MatrixTranspose1(pStruct->m_nMeas, pStruct->m_nState, pStruct->m_H, HT);

    // K = Pp * (HT) * (Rp_1)
    MatrixMultiply(pStruct->m_nState, pStruct->m_nState, pStruct->m_StatePp, pStruct->m_nState, pStruct->m_nMeas, HT, pStruct->m_K, 1.0);
    MatrixMultiply(pStruct->m_nState, pStruct->m_nMeas, pStruct->m_K, pStruct->m_nMeas, pStruct->m_nMeas, pStruct->m_Rp, HT, 1.0);
    for (i = 0; i < pStruct->m_nState * pStruct->m_nMeas; i++)pStruct->m_K[i] = HT[i];

    // X = Xp + K * Inno
    MatrixMultiply(pStruct->m_nState, pStruct->m_nMeas, pStruct->m_K, pStruct->m_nMeas, 1, pStruct->m_Inno, pStruct->m_StateX, 1.0);
    pStruct->m_dXl = MatrixNorm2(pStruct->m_nState, 1, pStruct->m_StateX);
    MatrixAddition2(pStruct->m_nState, 1, pStruct->m_StateXp, pStruct->m_StateX, 1.0, 1.0);

    // Joseph 形式, P = (I-KH)*Pp(I-KH)T + K*R*KT

    // M = I - K*H
    double* M = HT; SetValsD(M, 0, pStruct->m_nState * pStruct->m_nState); // 节约空间
    MatrixMultiply(pStruct->m_nState, pStruct->m_nMeas, pStruct->m_K, pStruct->m_nMeas, pStruct->m_nState, pStruct->m_H, M, 1.0);
    for (i = 0; i < pStruct->m_nState; i++) M[i * pStruct->m_nState + i] -= 1.0;
    for (i = 0; i < pStruct->m_nState * pStruct->m_nState; i++) M[i] *= -1.0;

    // (I-KH)*Pp(I-KH)T
    MatrixMultiply_HPHT(pStruct->m_nState, pStruct->m_nState, M, pStruct->m_StatePp, false, pStruct->m_StateP);

    // P = K * R * KT;
    MatrixMultiply_HPHT(pStruct->m_nState, pStruct->m_nMeas, pStruct->m_K, pStruct->m_R, pStruct->m_bPDiagonal, M);
    MatrixAddition2(pStruct->m_nState, pStruct->m_nState, M, pStruct->m_StateP, 1.0, 1.0);
    MatrixSymmetrization(pStruct->m_nState, pStruct->m_nState, pStruct->m_StateP);

    M = NULL;

    // 检查P对角线是否小于0
    for (i = 0; i < pStruct->m_nState; i++)
    {
        if (pStruct->m_StateP[i * pStruct->m_nState + i] <= 0.0)
        {
            return false;
        }
    }

    return true;
}


void SetGLSXP(CGLSEstimator* pStruct, int* index, int size, double* X, double* P, int row)
{
    if (!pStruct || !index || !X || !P)return;

    int i = 0, j = 0, n = size;
    for (i = 0; i < n; i++)
    {
        pStruct->m_StateXp[i] = X[index[i]];
        for (j = 0; j < pStruct->m_nState; j++) pStruct->m_StatePp[pStruct->m_nState * i + j] = P[index[i] * row + index[j]];
    }
}


void GetGLSXP(CGLSEstimator* pStruct, int* index, int size, double* X, double* P, int row)
{
    if (!pStruct || !index || !X || !P)return;

    int i = 0, j = 0, n = size;
    for (i = 0; i < n; i++)
    {
        X[index[i]] = pStruct->m_StateX[i];
        for (j = 0; j < pStruct->m_nState; j++) P[index[i] * row + index[j]] = pStruct->m_StateP[pStruct->m_nState * i + j];
    }
}



