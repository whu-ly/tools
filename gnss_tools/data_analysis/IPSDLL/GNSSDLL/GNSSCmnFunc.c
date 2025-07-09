#include "GNSSCmnFunc.h"
#include "BaseMath.h"
#include "BaseSDC.h"
#include "BaseCmnFunc.h"

/**
* @brief       GNSS卫星号转程序prn号
* @param[in]   no          char     卫星号(Gno,Rno,Cno,Eno,Jno)
* @return      int         程序自定义卫星序号, 1,2 int来区分各卫星号(1,2,....), 0:error
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int satno2prn1(const char* no)
{
    int  prn = 0;
    char code;

    if (sscanf(no, "%d", &prn) == 1) {
        if (1 <= prn && prn <= NSATGPS) prn += PRNGPS;
        else prn = 0;
        return prn;
    }

    if (sscanf(no, "%c%d", &code, &prn) < 2) return 0;

    switch (code)
    {
    case 'G': if (prn > NSATGPS || prn < 1)return 0; prn += PRNGPS; break;
    case 'R': if (prn > NSATGLO || prn < 1)return 0; prn += PRNGLO; break;
    case 'E': if (prn > NSATGAL || prn < 1)return 0; prn += PRNGAL; break;
    case 'J': if (prn > NSATQZS || prn < 1)return 0; prn += PRNQZS; break;
    case 'C':
    {
        // BD3试验星不参与BD3解算,因为和BD3的信号体制不完全一样
        if (prn == 31 || prn == 56 || prn == 57 || prn == 58) return 0;

        if (NSATBD2 == 0 && prn > 0 && prn <= 18)           return 0; // edit by ly0314

        if (prn > 0 && prn <= NSATBD2)          prn += PRNBD2;
        else if (prn > 18 && prn <= 18 + NSATBD3)      prn += PRNBD3 - 18;
        else    return 0;

        break;
    }

    default: return 0;
    }

    return prn;
}


/**
* @brief       GNSS卫星号转程序prn号
* @param[in]   no          int      系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   no          int      每个系统内的卫星号(1,2,3,4.....)
* @return      int         程序自定义卫星序号, 1,2 int来区分各卫星号(1,2,....), 0:error
* @note                    输入BD3卫星号的从1开始
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int satno2prn2(const int sys, const int no)
{
    int  prn = no;

    // BD3试验星不参与BD3解算,因为和BD3的信号体制不完全一样
    if (sys == SYSBD3)
    {
        if (prn == 31 || prn == 56 || prn == 57 || prn == 58) return 0;
    }

    switch (sys)
    {
    case SYSGPS: if (prn > NSATGPS || prn < 1)return 0; prn += PRNGPS; break;
    case SYSGLO: if (prn > NSATGLO || prn < 1)return 0; prn += PRNGLO; break;
    case SYSBD2: if (prn > NSATBD2 || prn < 1)return 0; prn += PRNBD2; break;
    case SYSBD3: if (prn > NSATBD3 || prn < 1)return 0; prn += PRNBD3; break;
    case SYSGAL: if (prn > NSATGAL || prn < 1)return 0; prn += PRNGAL; break;
    case SYSQZS: if (prn > NSATQZS || prn < 1)return 0; prn += PRNQZS; break;
    default: return 0;
    }

    return prn;
}


/**
* @brief       程序prn号转GNSS卫星号
* @param[in]   prn         int      程序自定义卫星序号, 由一个int来区分各卫星号
* @param[out]  sys         int      系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @return      int         每个系统内的卫星号(1,2,3,4.....)
* @note                    返回的BD3卫星号的从1开始
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int satprn2no(const int prn, int* sys)
{
    int sat = prn;

    int SYS = SYSNON;

    if (prn <= 0 || NSATMAX < prn)
    {
        sat = 0;
    }
    else if ((prn - PRNGPS) <= NSATGPS)
    {
        SYS = SYSGPS;
        sat -= PRNGPS;
    }
    else if (((prn - PRNGLO) <= NSATGLO) && NSATGLO)
    {
        SYS = SYSGLO;
        sat -= PRNGLO;
    }
    else if ((prn - PRNBD2) <= NSATBD2)
    {
        SYS = SYSBD2;
        sat -= PRNBD2;
    }
    else if ((prn - PRNBD3) <= NSATBD3)
    {
        SYS = SYSBD3;
        sat -= PRNBD3;
    }
    else if ((prn - PRNGAL) <= NSATGAL)
    {
        SYS = SYSGAL;
        sat -= PRNGAL;
    }
    else if ((prn - PRNQZS) <= NSATQZS)
    {
        SYS = SYSQZS;
        sat -= PRNQZS;
    }
    else
    {
        sat = 0;
    }

    if (sys) *sys = SYS;

    return sat;

}


/**
* @brief       程序prn号转GNSS卫星字符号
* @param[in]   prn         int      程序自定义卫星序号, 由一个int来区分各卫星号
* @return      string      Gnn,Rnn,Cnn,Enn,Jnn, error: Nnn
* @note                    返回的BD3卫星号的从18开始
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void satprn2nos(const int prn, char* str)
{
    int sat, sys;
    sat = satprn2no(prn, &sys);
    char s;

    switch (sys)
    {
    case SYSGPS: s = 'G'; break;
    case SYSGLO: s = 'R'; break;
    case SYSBD2: s = 'C'; break;
    case SYSBD3: s = 'C'; break;
    case SYSGAL: s = 'E'; break;
    case SYSQZS: s = 'J'; break;
    default: s = 'N';
    }

    // BD3从19开始
    if (sys == SYSBD3) sat += 18;

    sprintf(str, "%c%02d", s, sat);
}


/**
* @brief       NSATMAX循环时,对于单系统加快速度
* @param[in]   SatSYS      int      m_pGNSSOpt->m_SatSYS
* @param[in]   sys         int      当前prn的卫星系统
* @param[out]  prn         int      当前prn卫星
* @return      int         0:OK, 1:continue, 2:break
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int SkipPrn(const int SatSYS, const int sys, int* prn)
{
    if (*prn < 1) *prn = 1;
    if (*prn > NSATMAX) *prn = NSATMAX;

    if (sys == SYSGPS && (SatSYS & SYSGPS) == 0)
    {
        *prn += (NSATGPS - 1);
        return 1;
    }
    else if (sys == SYSGLO && (SatSYS & SYSGLO) == 0)
    {
        *prn += (NSATGLO - 1);
        return 1;
    }
    else if (sys == SYSBD2 && (SatSYS & SYSBD2) == 0)
    {
        *prn += (NSATBD2 - 1);
        return 1;
    }
    else if (sys == SYSBD3 && (SatSYS & SYSBD3) == 0)
    {
        *prn += (NSATBD3 - 1);
        return 1;
    }
    else if (sys == SYSGAL && (SatSYS & SYSGAL) == 0)
    {
        *prn += (NSATGAL - 1);
        return 1;
    }
    else if (sys == SYSQZS && (SatSYS & SYSQZS) == 0)
    {
        return 2;
    }

    return 0;
}


/**
* @brief       根据prn号确定BDS卫星的类型
* @param[in]   prn         int      prn号
* @return      int         1:GEO,2:IGSO,3:MEO,0:error
* @note        BD2和BD3都可以用
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int BDSSatelliteType(int prn)
{
    int sys = 0, sat = 0;
    sat = satprn2no(prn, &sys);
    if (sys == SYSBD2)
    {
        if (sat <= 5 || sat == 18) return 1;
        else if ((sat > 5 && sat <= 10) || sat == 13 || sat == 16) return 2;
        else if (sat == 11 || sat == 12 || sat == 14) return 3;
        else return 0;
    }
    else if (sys == SYSBD3)
    {
        //sat += NSATBD2;
        sat += 18;

        if (sat == 59 || sat == 60 || sat == 61) return 1;
        else if (sat == 31 || sat == 38 || sat == 39 || sat == 40 || sat == 56) return 2;
        else if (sat <= 61) return 3;
        else return 0;
    }

    return 0;
}


/**
* @brief       根据系统类型返回索引号
* @param[in]   sys         int      卫星系统类型,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @return      int         系统索引号,[0,NSYS-1]
* @note        BD2和BD3都可以用
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int Sys2Index(int sys)
{
    switch (sys)
    {
    case SYSGPS: return ISYSGPS;
    case SYSGLO: return ISYSGLO;
    case SYSBD2: return ISYSBD2;
    case SYSBD3: return ISYSBD3;
    case SYSGAL: return ISYSGAL;
    case SYSQZS: return ISYSQZS;
    case SYSIRN: return ISYSIRN;
    case SYSLEO: return ISYSLEO;
    }

    return -1;
}


/**
* @brief       根据索引号返回系统类型
* @param[in]   index       int      系统索引号,[0,NSYS-1]
* @return      int         卫星系统类型,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int Index2Sys(int index)
{
    if (index < 0 || index >= NSYS) return SYSNON;

    return gs_Index2Sys[index];
}


/**
* @brief       计算卫地距(包含sagnac效应改正)
* @param[in]   rs          double   ECEF系下的卫星坐标(m)
* @param[in]   rr          double   ECEF系下的测站坐标(m)
* @param[out]  e           double   卫地单位向量, 测站指向卫星
* @return      double      卫地距(包含sagnac效应改正), -1.0:error, 没有卫星位置
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double geodist(const double rs[3], const double rr[3], double e[3])
{
    double r;
    e[0] = rs[0] - rr[0]; e[1] = rs[1] - rr[1]; e[2] = rs[2] - rr[2];
    r = sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (r < gs_WGS84_a) return -1.0;
    e[0] = e[0] / r; e[1] = e[1] / r; e[2] = e[2] / r;
    return r + gs_WGS84_OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}


/**
* @brief       计算卫星方位角和高度角
* @param[in]   LLH         double   测站经纬度
* @param[in]   e           double   ECEF系下, 测站指向卫星的单位向量
* @param[out]  azel        double   azimuth/elevation {az,el} (rad)
* @return      double      elevation angle (rad)
* @note        (0.0<=azel[0]<2*pi,-pi/2<=azel[1]<=pi/2)
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double satazel(const double LLH[3], const double e[3], double* azel)
{
    double az = 0.0, el = PI / 2.0;
    double enu[3];

    if (LLH[2] > -gs_WGS84_a)
    {
        Vxyz2enu(LLH, e, enu);
        az = MatrixDot1(3, enu, enu) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
        if (az < 0.0) az += 2 * PI;
        el = atan(enu[2] / sqrt(enu[0] * enu[0] + enu[1] * enu[1]));
    }

    if (azel) { azel[0] = az; azel[1] = el; }

    return el;
}


/**
* @brief       初始化Sat的基本信息,主要是prn,sys,sys_id
* @param[in]   pSatStatus  SATSTATUS   卫星状态结构体
* @param[in]   GLO_iFreq   int         GLO频率数,NULL的话用内部的
* @return
* @note
*/
bool initSatStatus(SATSTATUS* pSatStatus, int* GLO_iFreq)
{
    if (!pSatStatus)return false;

    int glo_frq = 0;

    for (int prn = 1; prn <= NSATMAX; prn++)
    {
        pSatStatus[prn].sat = satprn2no(prn, &(pSatStatus[prn].sys));
        pSatStatus[prn].sys_id = Sys2Index(pSatStatus[prn].sys);

        if (GLO_iFreq)
        {
            glo_frq = GLO_iFreq[(int)pSatStatus[prn].sat];
        }
        else
        {
            glo_frq = gs_GLO_iFreq[(int)pSatStatus[prn].sat];
        }

        getGNSSFrequency(pSatStatus[prn].sys, pSatStatus[prn].frq, glo_frq);

        for (int f = 0; f < NFREQ; f++)
        {
            pSatStatus[prn].lam[f] = getGNSSWavelen(pSatStatus[prn].sys, f, glo_frq, false);
        }
    }

    return true;
}


/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   sys         int       系统标识,SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   GLO_iFreq   int       GLO卫星的频数
* @param[out]  frq         double    frq[3]三个频点上的频率
* @return      bool        true or false
* @note        f0 = 10.23MHZ, lam0 = 29.31m
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool getGNSSFrequency(const int sys, double frq[NFREQ], const int GLO_iFreq)
{
    if (sys == SYSGLO && GLO_iFreq < -999) return false;

    static double GPS_Frq[3] = { FREQ1_GPS , FREQ2_GPS , FREQ3_GPS };   // L1 L2 L5
    static double BD2_Frq[3] = { FREQ1_BD2 , FREQ2_BD2 , FREQ5_BD3 };   // B1I B2I B2a
    static double BD3_Frq[3] = { FREQ1_BD3 , FREQ2_BD3 , FREQ5_BD3 };   // B1I B2I B2a
    static double GAL_Frq[3] = { FREQ1_GAL , FREQ3_GAL , FREQ2_GAL };   // E1 E5b E5a
    static double QZS_Frq[3] = { FREQ1_QZS , FREQ2_QZS , FREQ3_QZS };
    static double GLO_Frq[3];
    GLO_Frq[0] = gs_GLO_Freq[0] + gs_GLO_dFreq[0] * GLO_iFreq;
    GLO_Frq[1] = gs_GLO_Freq[1] + gs_GLO_dFreq[1] * GLO_iFreq;
    GLO_Frq[2] = gs_GLO_Freq[2] + gs_GLO_dFreq[2] * GLO_iFreq;

    for (int f = 0; f < NFREQ; f++)
    {
        if (sys == SYSGPS) frq[f] = GPS_Frq[f];
        if (sys == SYSBD2) frq[f] = BD2_Frq[f];
        if (sys == SYSBD3) frq[f] = BD3_Frq[f];
        if (sys == SYSGAL) frq[f] = GAL_Frq[f];
        if (sys == SYSQZS) frq[f] = QZS_Frq[f];
        if (sys == SYSGLO) frq[f] = GLO_Frq[f];
    }

    if (gs_bSwitchGNSSFrq)
    {
        if (sys == SYSGPS)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strGPSFrq[f], "L1"))         frq[f] = FREQ1_GPS;
                else if (!strcmp(gs_strGPSFrq[f], "L2"))    frq[f] = FREQ2_GPS;
                else if (!strcmp(gs_strGPSFrq[f], "L5"))    frq[f] = FREQ3_GPS;
            }
        }
        else if (sys == SYSGLO)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strGLOFrq[f], "G1"))		    frq[f] = (gs_GLO_Freq[0] + gs_GLO_dFreq[0] * GLO_iFreq);
                else if (!strcmp(gs_strGLOFrq[f], "G2"))	frq[f] = (gs_GLO_Freq[1] + gs_GLO_dFreq[1] * GLO_iFreq);
                else if (!strcmp(gs_strGLOFrq[f], "G3"))	frq[f] = (gs_GLO_Freq[2] + gs_GLO_dFreq[2] * GLO_iFreq);
            }
        }
        else if (sys == SYSBD2)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strBD2Frq[f], "B1I"))        frq[f] = FREQ1_BD2;
                else if (!strcmp(gs_strBD2Frq[f], "B2I"))   frq[f] = FREQ2_BD2;
                else if (!strcmp(gs_strBD2Frq[f], "B3I"))   frq[f] = FREQ3_BD2;
            }
        }
        else if (sys == SYSBD3)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strBD3Frq[f], "B1I"))        frq[f] = FREQ1_BD3;
                else if (!strcmp(gs_strBD3Frq[f], "B2I"))   frq[f] = FREQ2_BD3;
                else if (!strcmp(gs_strBD3Frq[f], "B2b"))   frq[f] = FREQ2_BD3;
                else if (!strcmp(gs_strBD3Frq[f], "B3I"))   frq[f] = FREQ3_BD3;
                else if (!strcmp(gs_strBD3Frq[f], "B1C"))   frq[f] = FREQ4_BD3;
                else if (!strcmp(gs_strBD3Frq[f], "B2a"))   frq[f] = FREQ5_BD3;
            }
        }
        else if (sys == SYSGAL)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strGALFrq[f], "E1"))         frq[f] = FREQ1_GAL;
                else if (!strcmp(gs_strGALFrq[f], "E5a"))   frq[f] = FREQ2_GAL;
                else if (!strcmp(gs_strGALFrq[f], "E5b"))   frq[f] = FREQ3_GAL;
            }
        }
        else if (sys == SYSQZS)
        {
            for (int f = 0; f < NFREQ; f++)
            {
                if (!strcmp(gs_strQZSFrq[f], "L1"))		    frq[f] = FREQ1_QZS;
                else if (!strcmp(gs_strQZSFrq[f], "L2"))	frq[f] = FREQ2_QZS;
                else if (!strcmp(gs_strQZSFrq[f], "L5"))	frq[f] = FREQ3_QZS;
            }
        }
    }

    return true;
}

/**
* @brief       计算卫星载波频率或波长
* @param[in]   sys         int      卫星系统, SYSGPS,SYSGLO,SYSBD2,SYSBD3,SYSGAL,SYSQZS
* @param[in]   frq         int      频率 (0:L1,B1,E1; 1:L2,B2,E5a; 2:L5,B3,E5b; 3:E5a+b)
* @param[in]   GLO_iFreq     int      GLO卫星的频率号, 从导航文件中读取
* @param[in]   bfrq        bool     true: 给出频率(HZ), false: 给出波长(m)
* @return      double      载波频率你或波长(HZ,m) (0.0: error)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double getGNSSWavelen(const int sys, const int frq, const int GLO_iFreq, const bool bfrq)
{
    double frqs[NFREQ] = { 0.0 };
    if (getGNSSFrequency(sys, frqs, GLO_iFreq) == false || frqs[frq] <= 0.0) return 0.0;
    return (bfrq ? frqs[frq] : CLIGHT / frqs[frq]);
}


/**
* @brief       计算DOP值(dilution of precision)
* @param[in]   Azim        double    Azim[NSATMAX+1], 卫星方位角(rad)
* @param[in]   Elev        double    Elev[NSATMAX+1], 卫星高度角(rad)
* @param[in]   ElevMask    double    截止高度角(rad)
* @param[in]   pSatStatus  SATSTATUS 卫星状态,PPP/RTK中计算参与解算卫星的DOPs值用,默认为NULL
* @param[out]  DOPs        double    DOP值 {GDOP,PDOP,HDOP,VDOP}
* @return      bool        true or false
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals:
*              多系统的话有好几个钟差,计算DOP值需要改正下
*/
bool ComputeDOP(const double* Azim, const double* Elev, double ElevMask, double DOPs[4], const SATSTATUS* pSatStatus, bool bIAR, bool bIARPAR)
{
    DOPs[0] = DOPs[1] = DOPs[2] = DOPs[3] = 99.9;

    bool bSatStatus = !(pSatStatus == NULL);
    if (ElevMask < 1.0 * D2R) ElevMask = 1.0 * D2R;

    bool flag = true;
    int ncount = 0;
    int pos[NSATMAX + 1] = { 0 };

    for (int i = 1; i < NSATMAX + 1; i++)
    {
        if (bSatStatus)
        {
            if (bIAR)
            {
                for (int f = 0; f < NFREQ; f++)
                {
                    // 只要有1个频率固定就可以
                    if (pSatStatus[i].AR_Status[f] >= ARSTATE_FIXED)
                    {
                        pos[ncount] = i;
                        ncount++;
                        break;
                    }
                }
            }
            else if (bIARPAR)
            {
                for (int f = 0; f < NFREQ; f++)
                {
                    // 只要有1个频率固定就可以
                    if (pSatStatus[i].AR_Status[f] >= ARSTATE_FLOFIX)
                    {
                        pos[ncount] = i;
                        ncount++;
                        break;
                    }
                }
            }
            else
            {
                if (pSatStatus[i].azel[1] >= ElevMask)
                {
                    pos[ncount] = i;
                    ncount++;
                }
            }
        }
        else
        {
            if (Elev[i] >= ElevMask)
            {
                pos[ncount] = i;
                ncount++;
            }
        }
    }

    if (ncount < 4) return false;

    double H[NSATMAX * 4] = { 0.0 };
    double az, el;

    for (int i = 0; i < ncount; i++)
    {
        if (bSatStatus)
        {
            az = pSatStatus[pos[i]].azel[0];
            el = pSatStatus[pos[i]].azel[1];
            //if (bIARPAR)printf("%d,%8.3f,%8.3f\n", i, az * R2D, el * R2D);
        }
        else
        {
            az = Azim[pos[i]];
            el = Elev[pos[i]];
        }

        H[i * 4 + 0] = cos(el) * sin(az);
        H[i * 4 + 1] = cos(el) * cos(az);
        H[i * 4 + 2] = sin(el);
        H[i * 4 + 3] = 1.0;
    }

    double Q[4 * 4] = { 0.0 };
    MatrixMultiply_BTPB(ncount, 4, H, NULL, true, Q);
    if (MatrixInvSP(4, 4, Q))
    {
        DOPs[0] = xsqrt(Q[0] + Q[5] + Q[10] + Q[15]); /* GDOP */
        DOPs[1] = xsqrt(Q[0] + Q[5] + Q[10]);         /* PDOP */
        DOPs[2] = xsqrt(Q[0] + Q[5]);                 /* HDOP */
        DOPs[3] = xsqrt(Q[10]);                       /* VDOP */
    }
    else
    {
        flag = false;
    }

    return flag;
}



/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @return      double      三频组合观测值的波长(m)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_lam(const double frq[NFREQ], const int i, const int j, const int k)
{
    return CLIGHT / (i * frq[0] + j * frq[1] + k * frq[2]);
}


/**
* @brief       获取每个系统三个频点上的频率
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   L           double    三频L观测值(cycle)
* @return      double      三频L观测值的组合(m), 0:false
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_ZD_L(const double frq[NFREQ], const int i, const int j, const int k, const double L[3])
{
    if ((i && (!L[0])) || (j && (!L[1])) || (k && (!L[2]))) return 0.0;

    double L1 = CLIGHT / frq[0] * L[0];
    double L2 = CLIGHT / frq[1] * L[1];
    double L5 = CLIGHT / frq[2] * L[2];
    return (i * frq[0] * L1 + j * frq[1] * L2 + k * frq[2] * L5) / (i * frq[0] + j * frq[1] + k * frq[2]);
}


/**
* @brief       获取单差三频L观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   Li          double    三频L观测值(cycle)
* @param[in]   Lj          double    基准的三频L观测值(cycle)
* @return      double      单差三频L观测值的组合(m), 0:false
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_SD_L(const double frq[NFREQ], const int i, const int j, const int k, const double Li[3], const double Lj[3])
{
    double L[3] = { 0.0 };
    M31_M31(Li, Lj, L);
    return TFLC_ZD_L(frq, i, j, k, L);
}


/**
* @brief       获取非差三频P观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   P           double    三频P观测值(m)
* @return      double      三频P观测值的组合(m), 0:false
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_ZD_P(const double frq[NFREQ], const int i, const int j, const int k, const double P[3])
{
    if ((i && (!P[0])) || (j && (!P[1])) || (k && (!P[2]))) return 0.0;
    return (i * frq[0] * P[0] + j * frq[1] * P[1] + k * frq[2] * P[2]) / (i * frq[0] + j * frq[1] + k * frq[2]);
}


/**
* @brief       获取单差三频P观测值的组合
* @param[in]   frq         double    三频频率
* @param[in]   i,j,k       int       多频信号组合系数
* @param[in]   Pi          double    三频P观测值(m)
* @param[in]   Pj          double    基准的三频P观测值(m)
* @return      double      单差三频P观测值的组合(m), 0:false
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_SD_P(const double frq[NFREQ], const int i, const int j, const int k, const double Pi[3], const double Pj[3])
{
    double P[3] = { 0.0 };
    M31_M31(Pi, Pj, P);
    return TFLC_ZD_P(frq, i, j, k, P);
}



/**
* @brief       获取伪距三频无电离层组合IFLC
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    伪距观测值(m)
* @return      double      IFLC组合值(m),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_IF_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ])
{
    if (P[iFrq[0]] == 0.0 || P[iFrq[1]] == 0.0) return 0.0;

    double lam02 = lam[iFrq[0]] * lam[iFrq[0]];
    double lam12 = lam[iFrq[1]] * lam[iFrq[1]];
    const double c1 = lam12 / (lam12 - lam02);
    const double c2 = lam02 / (lam12 - lam02);
    double IF_P = c1 * P[iFrq[0]] - c2 * P[iFrq[1]];
    if (IF_P == 0.0) IF_P = IPS_EPSILON; // 防止为零
    return IF_P;
}


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
double TFLC_IF_L(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ], bool bLlam)
{
    if (L[iFrq[0]] == 0.0 || L[iFrq[1]] == 0.0) return 0.0;

    double lam02 = lam[iFrq[0]] * lam[iFrq[0]];
    double lam12 = lam[iFrq[1]] * lam[iFrq[1]];
    const double c1 = lam12 / (lam12 - lam02);
    const double c2 = lam02 / (lam12 - lam02);
    double IF_L = bLlam ? (c1 * L[iFrq[0]] - c2 * L[iFrq[1]]) : (c1 * lam[iFrq[0]] * L[iFrq[0]] - c2 * lam[iFrq[1]] * L[iFrq[1]]);
    if (IF_L == 0.0) IF_L = IPS_EPSILON; // 防止为零
    return IF_L;
}


/**
* @brief       获取相位三频GF组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   L           double    相位观测值(cycle)
* @return      double      GF组合值(m),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_GF_L(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ])
{
    if (L[iFrq[0]] == 0.0 || L[iFrq[1]] == 0.0) return 0.0;
    double GF = (lam[iFrq[0]] * L[iFrq[0]] - lam[iFrq[1]] * L[iFrq[1]]);
    if (GF == 0.0) GF = IPS_EPSILON; // 防止为零
    return GF;
}



/**
* @brief       获取相位三频伪距宽巷组合
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    相位观测值(m)
* @return      double      WL组合值(m),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_WL_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ])
{
    if (P[iFrq[0]] == 0.0 || P[iFrq[1]] == 0.0) return 0.0;
    double WL = (lam[iFrq[1]] * P[iFrq[0]] - lam[iFrq[0]] * P[iFrq[1]]) / (lam[iFrq[1]] - lam[iFrq[0]]);
    if (WL == 0.0) WL = IPS_EPSILON; // 防止为零
    return WL;
}


/**
* @brief       获取相位三频伪距窄巷组合
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   P           double    相位观测值(m)
* @return      double      NL组合值(m),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_NL_P(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double P[NFREQ])
{
    if (P[iFrq[0]] == 0.0 || P[iFrq[1]] == 0.0) return 0.0;
    double NL = (lam[iFrq[1]] * P[iFrq[0]] + lam[iFrq[0]] * P[iFrq[1]]) / (lam[iFrq[1]] + lam[iFrq[0]]);
    if (NL == 0.0) NL = IPS_EPSILON; // 防止为零
    return NL;
}


/**
* @brief       获取三频MW组合
* @param[in]   lam         double    三频波长
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   L           double    相位观测值(cycle)
* @param[in]   P           double    伪距观测值(m)
* @return      double      MW组合值(cycle),0:计算不成功
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double TFLC_MW_LP(const double lam[NFREQ], const TFLCTYPE iFrq[2], const double L[NFREQ], const double P[NFREQ])
{
    if (NFREQ < 2) return 0.0;   // edit by ly-0119

    double LR[NFREQ] = { 0.0 };
    for (int f = 0; f < NFREQ; f++) LR[f] = L[f] * lam[f]; // 单位转换成m

    double L_WL = TFLC_WL_P(lam, iFrq, LR); // 相位宽巷(m)
    double P_NL = TFLC_NL_P(lam, iFrq, P);  // 伪距窄巷(m)
    if (L_WL == 0.0 || P_NL == 0.0) return 0.0;
    double lam_WL = lam[iFrq[0]] * lam[iFrq[1]] / (lam[iFrq[1]] - lam[iFrq[0]]); // 宽巷波长
    double MW = (L_WL - P_NL) / lam_WL;
    if (MW == 0.0) MW = IPS_EPSILON; // 防止为零
    return MW;
}



/**
* @brief       得到双频组合的索引号
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      int         IF索引号
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int TFLC_IFIndex1(const TFLCTYPE iFrq[2])
{
    if (iFrq[0] == TFLC_L1 && iFrq[1] == TFLC_L2) return 0;
    if (iFrq[0] == TFLC_L1 && iFrq[1] == TFLC_L3) return 1;
    if (iFrq[0] == TFLC_L2 && iFrq[1] == TFLC_L3) return 2;
    return -1;
}


/**
* @brief       得到双频组合的索引号
* @param[in]   id          IF索引号
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      bool        true or false
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool TFLC_IFIndex2(const int id, TFLCTYPE iFrq[2])
{
    if (id == 0) { iFrq[0] = TFLC_L1; iFrq[1] = TFLC_L2; return true; }
    if (id == 1) { iFrq[0] = TFLC_L1; iFrq[1] = TFLC_L3; return true; }
    if (id == 2) { iFrq[0] = TFLC_L2; iFrq[1] = TFLC_L3; return true; }
    return false;
}


/**
* @brief       由某个频点得到与其联系的双频组合的索引号
* @param[in]   iFrq        哪个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @param[in]   id          IF索引号
* @return      bool        true or false
* @note
*              L1_L2:0, L1_L3:1, L2:L3:2
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool TFLC_IFIndex3(const TFLCTYPE iFrq, int id[2])
{
    if (iFrq == TFLC_L1) { id[0] = 0; id[1] = 1; return true; }
    if (iFrq == TFLC_L2) { id[0] = 0; id[1] = 2; return true; }
    if (iFrq == TFLC_L3) { id[0] = 1; id[1] = 2; return true; }
    return false;
}


/**
* @brief       选择IF组合的两个频率
* @param[in]   bFrq        bool      哪些频率可用
* @param[in]   iFrq        TFLCTYPE  哪两个频率,[TFLC_L1,TFLC_L2,TFLC_L3]
* @return      bool        true or false
* @note
*
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool TFLC_SelectIFIndex(const bool bFrq[NFREQ], TFLCTYPE iFrq[2])
{
    int nf = 0;
    for (int f = 0; f < NFREQ; f++)
    {
        if (bFrq[f])
        {
            iFrq[nf++] = (TFLCTYPE)f;
        }

        if (nf == 2) break;
    }

    return (nf == 2);
}


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
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool ComputeMultipath(const double P[3], const double L[3], const double lam[3], const char CS_Type[3], int nMP[3], double MPMean[3], double MP[3])
{
    // 选择哪两个频率的L观测值来导出MP
    int j = 0, k = 0;
    if (L[0] != 0.0 && L[1] != 0.0)
    {
        j = 0; k = 1;
    }
    else if (L[0] != 0.0 && L[2] != 0.0)
    {
        j = 0; k = 2;
    }
    else if (L[1] != 0.0 && L[2] != 0.0)
    {
        j = 1; k = 2;
    }
    else
    {
        return false;
    }

    double m = 0, MPCurrent[3] = { 0.0 };
    for (int f = 0; f < 3; f++)
    {
        MP[f] = 0.0;
        if (P[f] == 0.0) continue;

        m = (lam[f] * lam[f] + lam[j] * lam[j]) / (lam[j] * lam[j] - lam[k] * lam[k]);
        MPCurrent[f] = P[f] + (m - 1) * lam[j] * L[j] - m * lam[k] * L[k];

        // 如果当前MP和前面MP的值超过0.5m时,认为存在周跳,重新初始化
        if (fabs(MPCurrent[f] - MPMean[f]) > 2.0 || CS_Type[f])
        {
            MPMean[f] = MPCurrent[f];
            nMP[f] = 1;
        }
        else
        {
            MPMean[f] = (MPMean[f] * nMP[f] + MPCurrent[f]) / (nMP[f] + 1);
            nMP[f] ++;
        }

        MP[f] = MPCurrent[f] - MPMean[f];
    }

    return true;
}


/**
* @brief       双天线基线向量得到航向角和俯仰角
* @param[in]   BL           double   双天线基线向量
* @param[in]   Azimuth      double   北向起算的航向角[0,2*PI]和俯仰角[-PI/2,PI/2]
* @return
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool BL2Azimuth(const double BL[3], double Azimuth[3], const double* XYZ, const double* XYZP, double* AttP)
{
    double cog = atan2(BL[1], BL[0]);
    double BL2 = BL[0] * BL[0] + BL[1] * BL[1];

    // 车辆前向是Y轴,现在需要计算车辆X轴与E向的夹角,需要以下转换
    // 两者方位指示轴不一样，车辆速度是用Y轴指示的，姿态角是用X轴指示的
    cog -= PI_12;
    if (cog < -PI) cog += PI_2;

    // Attitude2Azimuth
    if (cog < 0.0)
    {
        cog = -cog;
    }
    else
    {
        cog = PI_2 - cog;
    }

    Azimuth[0] = cog; // 北向起算的航向角
    Azimuth[1] = atan2(BL[2], sqrt(BL2));

    if (XYZ && XYZP && AttP)
    {
        double ENUP[9] = { 0.0 };
        double Ren[9] = { 0.0 };
        double LLH[3] = { 0.0 };
        XYZ2LLH(XYZ, LLH, 0);
        Rxyz2enu(LLH, Ren);
        MatrixMultiply_HPHT(3, 3, Ren, XYZP, false, ENUP);

        MatrixZero(3, 3, AttP);
        double h = 0, l = 0, f[3] = { 0.0 };

        h = BL2;
        l = BL2 + BL[2] * BL[2];

        // 航向角方差
        f[0] = -BL[1] / h;
        f[1] = BL[0] / h;
        f[2] = 0.0;
        MatrixMultiply_HPHT(1, 3, f, ENUP, false, AttP + 0);

        // 俯仰角方差
        f[2] = sqrt(h) / l;
        f[0] = -BL[0] * BL[2] / f[2];
        f[1] = -BL[1] * BL[2] / f[2];
        MatrixMultiply_HPHT(1, 3, f, ENUP, false, AttP + 4);
    }

    return true;
}


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
void InitGNSS_StateIndex(GNSS_GLSINFO* pGLSInfo, int EstTrp, int EstIon, bool bReverse)
{
    int I = 0;

    // 位置索引
    pGLSInfo->IPos = I; 
    pGLSInfo->LPos = 3; 
    I = pGLSInfo->IPos + pGLSInfo->LPos;

    // 模糊度索引
    pGLSInfo->IAmb = I;
    pGLSInfo->LAmb = MAXOBSLIM;
    pGLSInfo->EAmb = pGLSInfo->IAmb + pGLSInfo->LAmb;

    pGLSInfo->LStateConst = pGLSInfo->LPos + pGLSInfo->LAmb;

    // 其他索引
    pGLSInfo->IVel = pGLSInfo->LVel = 0;
    pGLSInfo->IAcc = pGLSInfo->LAcc = 0;
    pGLSInfo->IClkVel = pGLSInfo->LClkVel = 0;
    pGLSInfo->ITrp = pGLSInfo->LTrp = 0;
    pGLSInfo->EIon = pGLSInfo->IIon = pGLSInfo->LIon = 0;
}


/**
* @brief       GNSS方程部分信息清零
* @param[in]   n           int      卫星数
* @param[in]   pGNSSMEQ    GNSS_MEQ GNSS_MEQ
* return
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void zeroGNSSMEQ(const int n, GNSS_MEQ* pGNSSMEQ)
{
    for (int i = 0; i < n; i++)
    {
        GNSS_MEQ* pMEQ = pGNSSMEQ + i;
        for (int f = 0; f < NFREQ; f++)
        {
            pMEQ->RFactL[f] = 1.0;
            pMEQ->RFactP[f] = 1.0;
            pMEQ->RFactD[f] = 1.0;
            pMEQ->GLSPosP[f] = 0;
            pMEQ->GLSPosL[f] = 0;
        }
        pMEQ->Trp_base = 0.0;
    }
}


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
GPSTIME adjweek(GPSTIME t, GPSTIME t0)
{
    double tt = MinusGPSTIME(t, t0);
    if (tt < -302400.0) return (AddGPSTIMESec(t, 604800.0));
    if (tt > 302400.0) return (MinusGPSTIMESec(t, 604800.0));
    return t;
}


/**
* @brief       URA值转换为URA指数
* @param[in]   value       double   URA值
* @return      int         URA指数(0-14)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
int URA2Index(const double value)
{
    int i;
    for (i = 0; i < 15; i++) if (gs_ura_eph[i] >= value) break;
    return i;
}


/**
* @brief       URA指数转换为URA值
* @param[in]   index       int       URA指数(0-14)
* @return      double      URA值
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double Index2URA(const int index)
{
    return index < 0 || 15 < index ? 6144.0 : gs_ura_eph[index];
}


/**
* @brief       获取动态阈值
* @param[in]   src[]		待确定动态阈值的源数据（均大于0）
* @param[in]   nsrc		    源数据维度
* @param[in]   proportion	保留比例
* @param[in]   minThres		最小阈值
* @param[in]   maxThres		最大阈值
* @param[in]   bMedian		是否以中位数为基准
* @return      double		动态阈值
* @note
* @internals
*/
double GetDynamicThres(double* src, int nsrc, double proportion, double minThres, double maxThres, bool bMedian)
{
    if (!src || nsrc < 2) return minThres;

    double median = 0.0, thres = minThres;
    int index = 0;

    SortArr(src, nsrc, true);

    if (bMedian) median = src[nsrc / 2];

    for (int i = 0; i < nsrc; i++)
    {
        src[i] = fabs(src[i] - median);
    }

    SortArr(src, nsrc, true);

    index = (int)(proportion * nsrc); // 保留比例
    if (src[index] < minThres) thres = minThres;
    else if (src[index] > maxThres) thres = maxThres;
    else thres = src[index];

    return thres;
}

bool CheckDogPermission(GPSTIME* dataTime)
{
    GPSTIME LicenseTime;
    InitGPSTIME2(2651, 0, &LicenseTime);
    if (MinusGPSTIME(*dataTime, LicenseTime) > 0.0)return false;
    return true;
}