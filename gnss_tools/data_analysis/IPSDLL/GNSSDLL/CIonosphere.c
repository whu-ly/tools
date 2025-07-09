#include "CIonosphere.h"
#include "BaseTime.h"


#define HION        350000.0            /* ionosphere height (m) */
#define IP_SIZE     300                 // 300个数据
#define IP_PRE_SIZE 20                  // 20个用来预报


///< Constructor function, to be called when defining
extern void InitCIonosphere(CIonosphere* pStruct)
{
    if (!pStruct)return;

    pStruct->m_KloCoeff = NULL; // ALPHA, BETA {a0,a1,a2,a3,b0,b1,b2,b3}
    pStruct->m_azel = NULL;     // azimuth/elevation angle {az,el} (rad)

    pStruct->m_STD = 0.0;       // (m)
    pStruct->m_ZTD = 0.0;
    pStruct->m_MF = 0.0;
    pStruct->m_IonoVar = 0.0;
    pStruct->m_GPSL1Lam = CLIGHT / FREQ1_GPS;
    pStruct->m_lam = pStruct->m_GPSL1Lam;

    InitGPSTIME(&pStruct->m_gt);
}


static bool KlobucharModel(CIonosphere* pStruct)
{
    if (!pStruct)return false;

    double ion_default[] = { /* 2004/1/1 */
        0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
        0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
    };

    double tt, f, psi, phi, lam, amp, per, x;

    if (pStruct->m_KloCoeff == NULL)
        pStruct->m_KloCoeff = ion_default;

    double nor2 = 0.0;
    for (int i = 0; i < 8; i++)
    {
        nor2 += pStruct->m_KloCoeff[i] * pStruct->m_KloCoeff[i];
    }

    if (nor2 <= IPS_EPSILON)
    {
        pStruct->m_KloCoeff = ion_default;
    }

    /* earth centered angle (semi-circle) */
    psi = 0.0137 / (pStruct->m_azel[1] / PI + 0.11) - 0.022;

    /* subionospheric latitude/longitude (semi-circle) */
    phi = pStruct->m_LLH[0] / PI + psi * cos(pStruct->m_azel[0]);

    if (phi > 0.416) phi = 0.416;
    else if (phi < -0.416) phi = -0.416;

    lam = pStruct->m_LLH[1] / PI + psi * sin(pStruct->m_azel[0]) / cos(phi * PI);

    /* geomagnetic latitude (semi-circle) */
    phi += 0.064 * cos((lam - 1.617) * PI);

    /* local time (s) */
    tt = 43200.0 * lam + GetGPSTIMESow(pStruct->m_gt);

    tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

    /* slant factor */
    f = 1.0 + 16.0 * pow(0.53 - pStruct->m_azel[1] / PI, 3.0);

    /* ionospheric delay */
    amp = pStruct->m_KloCoeff[0] + phi * (pStruct->m_KloCoeff[1] + phi * (pStruct->m_KloCoeff[2] + phi * pStruct->m_KloCoeff[3]));
    per = pStruct->m_KloCoeff[4] + phi * (pStruct->m_KloCoeff[5] + phi * (pStruct->m_KloCoeff[6] + phi * pStruct->m_KloCoeff[7]));

    amp = amp < 0.0 ? 0.0 : amp;
    per = per < 72000.0 ? 72000.0 : per;
    x = 2.0 * PI * (tt - 50400.0) / per;

    pStruct->m_ZTD = CLIGHT * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
    x = pStruct->m_lam / pStruct->m_GPSL1Lam;
    pStruct->m_ZTD *= x * x; // 频率转换

    pStruct->m_MF = f;

    pStruct->m_STD = pStruct->m_MF * pStruct->m_ZTD;

    pStruct->m_IonoVar = pStruct->m_STD * pStruct->m_STD * ERR_BRDCI2;

    return true;
}


bool IonoComputer(CIonosphere* pStruct, GPSTIME gt, const double LLH[3], double* azel, double lam, IONO_TYPE ionoType, double* KloCoeff)
{
    if (!pStruct)return false;

    pStruct->m_KloCoeff = KloCoeff;
    pStruct->m_STD = 0.0;
    pStruct->m_ZTD = 0.0;
    pStruct->m_MF = 0.0;
    pStruct->m_IonoVar = 0.0;

    if (LLH[2] < -1000.0 || azel[1] <= 0)
    {
        return true;
    }

    pStruct->m_gt = gt;
    pStruct->m_LLH[0] = LLH[0]; pStruct->m_LLH[1] = LLH[1]; pStruct->m_LLH[2] = LLH[2];
    pStruct->m_azel = azel;
    pStruct->m_lam = lam;
    if (pStruct->m_lam < 0.001)
    {
        pStruct->m_lam = pStruct->m_GPSL1Lam;
    }

    if (ionoType == IONO_KLOB)
    {
        KlobucharModel(pStruct);
    }
    else
    {
        return false;
    }

    return true;
}
