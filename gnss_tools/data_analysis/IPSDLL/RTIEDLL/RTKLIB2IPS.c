#include "RTKLIB2IPS.h"
#include "BaseTime.h"
#include "GNSSCmnFunc.h"


static int ConvertPrn(int sat_rtk)
{
    int prn = 0, sys = 0, sat = 0;
    sys = satsys(sat_rtk, &sat);

    // IPS和rtklib两边卫星数不一致，一定要加上卫星数判断，edit by Lying
    if (sys == SYS_GPS)
    {
        if (sat <= NSATGPS)prn = PRNGPS + sat;
    }
    else if (sys == SYS_GLO)
    {
        if (sat <= NSATGLO)prn = PRNGLO + sat;
    }
    else if (sys == SYS_GAL)
    {
        if (sat <= NSATGAL)prn = PRNGAL + sat;
    }
    else if (sys == SYS_QZS)
    {
        if (sat <= NSATQZS)prn = PRNQZS + sat;
    }
    else if (sys == SYS_CMP)
    {
        if (sat <= NSATBD2)prn = PRNBD2 + sat;
        else if (sat > 18 && sat <= 18 + NSATBD3)prn = PRNBD3 + sat - 18;
    }

    return prn;
}


///< Convert rtklib eph to IPS eph, GCCEJ
static void ConvertEph(const eph_t *src, GPSEPH *dst)
{
    ConvertTime(src->toe, &dst->toe);
    ConvertTime(src->toc, &dst->toc);
    //ConvertTime(src->ttr, dst->ttr);

    dst->prn = ConvertPrn(src->sat);
    dst->iode = src->iode;
    dst->iodc = src->iodc;
    dst->sva = src->sva;
    dst->svh = src->svh;
    dst->week = src->week;
    dst->code = src->code;
    //dst->flag = src->flag;
    dst->A = src->A;
    dst->e = src->e;
    dst->i0 = src->i0;
    dst->OMG0 = src->OMG0;
    dst->omg = src->omg;
    dst->M0 = src->M0;
    dst->deln = src->deln;
    dst->OMGd = src->OMGd;
    dst->idot = src->idot;
    dst->crc = src->crc;
    dst->crs = src->crs;
    dst->cuc = src->cuc;
    dst->cus = src->cus;
    dst->cic = src->cic;
    dst->cis = src->cis;
    dst->toes = src->toes;
    //dst->fit = src->fit;
    dst->f0 = src->f0;
    dst->f1 = src->f1;
    dst->f2 = src->f2;

    for (int i = 0; i < 4; i++)
    {
        dst->tgd[i] = src->tgd[i];
    }
}


///< Convert rtklib eph to IPS eph, GLO
static void ConvertEphg(const geph_t *src, GLOEPH *dst)
{
    ConvertTime(src->toe, &dst->toe);
    ConvertTime(src->tof, &dst->tof);

    dst->prn = ConvertPrn(src->sat);
    dst->iode = src->iode;
    dst->frq = src->frq;
    dst->svh = src->svh;
    dst->sva = src->sva;
    dst->age = src->age;
    dst->taun = src->taun;
    dst->gamn = src->gamn;
    //dst->dtaun = src->dtaun;

    for (int i = 0; i < 3; i++)
    {
        dst->pos[i] = src->pos[i];
        dst->vel[i] = src->vel[i];
        dst->acc[i] = src->acc[i];
    }
}


static int FindFrqIndex(int sys, char(*type)[5], obsd_t obs)
{
    // 根据解算频点设置情况，选取对应索引
    // 该表严格和RTCM3数据格式相对应，edit by Lying
    int index = -1;
    char s[10] = "";
    if (sys == SYSGPS)
    {
        for (int f = 0; f < (N_FREQ + NEXOBS); f++)
        {
            if (obs.code[f] == CODE_NONE)continue;
            else if (obs.code[f] == CODE_L1C)strcpy(s, "L1"); // 对应L1
            else if (obs.code[f] == CODE_L1P)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1W)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1Y)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1M)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1N)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1S)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1L)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L2C)strcpy(s, "L2"); // 对应L2
            else if (obs.code[f] == CODE_L2D)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2S)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2L)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2X)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2P)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2W)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2Y)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2M)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2N)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L5I)strcpy(s, "L5"); // 对应L5
            else if (obs.code[f] == CODE_L5Q)strcpy(s, "L5");
            else if (obs.code[f] == CODE_L5X)strcpy(s, "L5");
            if (!strcmp(*type, s)) {
                index = f; break;
            }
        }
    }
    else if (sys == SYSGLO)
    {
        for (int f = 0; f < (N_FREQ + NEXOBS); f++)
        {
            if (obs.code[f] == CODE_NONE)continue;
            else if (obs.code[f] == CODE_L1C)strcpy(s, "G1"); // 对应G1
            else if (obs.code[f] == CODE_L1P)strcpy(s, "G1");
            else if (obs.code[f] == CODE_L2C)strcpy(s, "G2"); // 对应G2
            else if (obs.code[f] == CODE_L2P)strcpy(s, "G2");
            if (!strcmp(*type, s)) {
                index = f; break;
            }
        }
    }
    else if (sys == SYSBD2 || sys == SYSBD3)
    {
        for (int f = 0; f < (N_FREQ + NEXOBS); f++)
        {
            if (obs.code[f] == CODE_NONE)continue;
            else if (obs.code[f] == CODE_L2I)strcpy(s, "B1I"); // 对应B1I信号
            else if (obs.code[f] == CODE_L2Q)strcpy(s, "B1I");
            else if (obs.code[f] == CODE_L2X)strcpy(s, "B1I");
            else if (obs.code[f] == CODE_L7I)strcpy(s, "B2I"); // 对应B2I信号，注意BD3无B2I
            else if (obs.code[f] == CODE_L7Q)strcpy(s, "B2I");
            else if (obs.code[f] == CODE_L7X)strcpy(s, "B2I");
            else if (obs.code[f] == CODE_L6I)strcpy(s, "B3I"); // 对应B3I信号
            else if (obs.code[f] == CODE_L6Q)strcpy(s, "B3I");
            else if (obs.code[f] == CODE_L6X)strcpy(s, "B3I");
            else if (obs.code[f] == CODE_L5D)strcpy(s, "B2a"); // 对应B2a信号
            else if (obs.code[f] == CODE_L5P)strcpy(s, "B2a");
            else if (obs.code[f] == CODE_L5X)strcpy(s, "B2a");
            else if (obs.code[f] == CODE_L7D)strcpy(s, "B2b"); // 对应B2b信号
            else if (obs.code[f] == CODE_L1D)strcpy(s, "B1C"); // 对应B1C信号
            else if (obs.code[f] == CODE_L1P)strcpy(s, "B1C");
            else if (obs.code[f] == CODE_L1X)strcpy(s, "B1C");
            if (!strcmp(*type, s)) {
                index = f; break;
            }
        }
    }
    else if (sys == SYSGAL)
    {
        for (int f = 0; f < (N_FREQ + NEXOBS); f++)
        {
            if (obs.code[f] == CODE_NONE)continue;
            else if (obs.code[f] == CODE_L1C)strcpy(s, "E1");  // 对应E1
            else if (obs.code[f] == CODE_L1A)strcpy(s, "E1");
            else if (obs.code[f] == CODE_L1B)strcpy(s, "E1");
            else if (obs.code[f] == CODE_L1X)strcpy(s, "E1");
            else if (obs.code[f] == CODE_L1Z)strcpy(s, "E1");
            else if (obs.code[f] == CODE_L6A)strcpy(s, "E6");  // 对应E6
            else if (obs.code[f] == CODE_L6B)strcpy(s, "E6");
            else if (obs.code[f] == CODE_L6C)strcpy(s, "E6");
            else if (obs.code[f] == CODE_L6X)strcpy(s, "E6");
            else if (obs.code[f] == CODE_L6Z)strcpy(s, "E6");
            else if (obs.code[f] == CODE_L7I)strcpy(s, "E5b"); // 对应E5b
            else if (obs.code[f] == CODE_L7Q)strcpy(s, "E5b");
            else if (obs.code[f] == CODE_L7X)strcpy(s, "E5b");
            else if (obs.code[f] == CODE_L8I)strcpy(s, "E5a_b"); // 对应E5a_b
            else if (obs.code[f] == CODE_L8Q)strcpy(s, "E5a_b");
            else if (obs.code[f] == CODE_L8X)strcpy(s, "E5a_b");
            else if (obs.code[f] == CODE_L5I)strcpy(s, "E5a"); // 对应E5a
            else if (obs.code[f] == CODE_L5Q)strcpy(s, "E5a");
            else if (obs.code[f] == CODE_L5X)strcpy(s, "E5a");
            if (!strcmp(*type, s)) {
                index = f; break;
            }
        }
    }
    else if (sys == SYSQZS)
    {
        for (int f = 0; f < (N_FREQ + NEXOBS); f++)
        {
            if (obs.code[f] == CODE_NONE)continue;
            else if (obs.code[f] == CODE_L1C)strcpy(s, "L1"); // 对应L1
            else if (obs.code[f] == CODE_L1S)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1L)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L1X)strcpy(s, "L1");
            else if (obs.code[f] == CODE_L6S)strcpy(s, "LEX"); // 对应LEX
            else if (obs.code[f] == CODE_L6L)strcpy(s, "LEX");
            else if (obs.code[f] == CODE_L6X)strcpy(s, "LEX");
            else if (obs.code[f] == CODE_L2S)strcpy(s, "L2"); // 对应L2
            else if (obs.code[f] == CODE_L2L)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L2X)strcpy(s, "L2");
            else if (obs.code[f] == CODE_L5I)strcpy(s, "L5"); // 对应L5
            else if (obs.code[f] == CODE_L5Q)strcpy(s, "L5");
            else if (obs.code[f] == CODE_L5X)strcpy(s, "L5");
            if (!strcmp(*type, s)) {
                index = f; break;
            }
        }
    }
    return index;
}


///< Sort observations by PRN
static void SortOBS(OBS_DATA* src)
{
    if (!src || src->nsat <= 1)return;

    OBS_DATA_t dst[MAXOBS];
    int i, j = 0;
    int prnlist[NSATMAX + 1] = { -1 };

    memset(prnlist, -1, (NSATMAX + 1) * sizeof(int));

    for (i = 0; i < src->nsat; i++)prnlist[src->obs[i].prn] = i;

    for (i = 1; i < NSATMAX + 1; i++)
    {
        if (prnlist[i] < 0)continue;
        dst[j++] = src->obs[prnlist[i]];
    }
    memcpy(src->obs, dst, sizeof(OBS_DATA_t) * src->nsat);
}


/**
* @brief       rtklib gtime_t转程序ips gt
* @param[in]   no          src     rtklib gtime_t
* @param[out]  no          n       ips    gt
* @note
*/
void ConvertTime(gtime_t src, GPSTIME* dst)
{
    if (!dst)return;
    int Week = 0;
    double tow = time2gpst(src, &Week);
    dst->GPSWeek = Week;
    dst->secsOfWeek = (int)(tow);
    dst->fracOfSec = tow - dst->secsOfWeek;
}


/**
* @brief       rtklib nav_t转程序ipseph和ipsgeph
* @param[in]   no          src     rtklib  nav数组
* @param[out]  no          n       ipseph  eph数组
* @param[out]  no          dst     ipsgeph geph数组
* @param[out]  no          nEph    eph和geph更新的卫星数
* @note        仅支持GCCEJ，不支持GLONASS
*/
void ConvertNav(const nav_t *src, GPSEPH *dst, GLOEPH *dst_GLO, int nEph[])
{
    nEph[0] = nEph[1] = 0;

    if (dst == NULL) return;

    for (int i = 0; i < (src->n - src->ng); i++)
    {
        int prn = ConvertPrn(src->eph[i].sat);
        if (prn < 1 || prn > NSATMAX) continue;

        GPSTIME gt;
        ConvertTime(src->eph[i].toe, &gt);
        double dt = MinusGPSTIME(gt, dst[prn - 1].toe);
        if (dt <= 0.0) continue;

        ConvertEph(&src->eph[i], &dst[prn - 1]);		// G/C2/C3/E/J按照prn-1顺序存储
        nEph[0]++;
    }
}


/**
* @brief       rtklib obsd_t转程序rt_ipsobs
* @param[in]   no          src     rtklib obs数组
* @param[in]   no          n       rtklib obs卫星数
* @param[out]  no          dst     rt_ips obs数组
* @return      int         rt_ipsobs卫星数
* @note        仅支持 GCCE
*/
void ConvertOBS(const obsd_t *src, int n, OBS_DATA* dst, int* nOBS)
{
    if (!src || !dst)return;

    int index, sys = 0;
    GPSTIME gt;
    OBS_DATA_t* iobs;
    char(*frq)[5] = NULL;
    if (nOBS)*nOBS = 0;

    ConvertTime(src[0].time, &gt);
    if (MinusGPSTIME(gt, dst->gt) == 0.0) return; // 前后观测值重复

    dst->gt = gt;
    dst->flag = 0;
    dst->nsat = 0;
    memset(dst->ngnss, 0, sizeof(int) * NSYS);
    memset(dst->obs, 0, sizeof(OBS_DATA_t) * MAXOBS);

    for (int i = 0; i < n; i++)
    {
        if (dst->nsat >= MAXOBS)break;

        iobs = dst->obs + dst->nsat;
        iobs->prn = ConvertPrn(src[i].sat);
        if (iobs->prn > NSATMAX || iobs->prn < 1) continue; // 防止rtklib中有sbas卫星，数量超限

        satprn2no(iobs->prn, &sys);
        if (sys == SYSGPS) {
            dst->ngnss[0]++; frq = gs_strGPSFrq;
        }
        else if (sys == SYSBD2) {
            dst->ngnss[2]++; frq = gs_strBD2Frq;
        }
        else if (sys == SYSBD3) {
            dst->ngnss[3]++; frq = gs_strBD3Frq;
        }
        else if (sys == SYSGAL){
            dst->ngnss[4]++; frq = gs_strGALFrq;
        }
        else continue;

        for (int f = 0; f < NFREQ; f++)
        {
            index = FindFrqIndex(sys, frq + f, src[i]);
            if (index < 0)continue;
            iobs->P[f]  = src[i].P[index];
            iobs->L[f]  = src[i].L[index];
            iobs->D[f]  = src[i].D[index];
            iobs->LLI[f] = src[i].LLI[index];
            iobs->S[f] = (float)(src[i].SNR[index] * SNR_UNIT);
        }
        dst->nsat++;
    }
    SortOBS(dst);
    if (nOBS)*nOBS = dst->nsat;
    frq = NULL;
}
