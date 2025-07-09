#include "CTroposphere.h"
#include "BaseTime.h"
#include "BaseMath.h"
#include "BaseMatrix.h"


#define K2C(x) (x)-= 273.16      // convert Thermodynamic temperature(K) to Celsius(°C)
#define C2K(x) (x)+= 273.16      // convert Celsius(°C) to Thermodynamic temperature(K)

#define SPT_TK   273.16          // 0°C
#define SPT_T0K  288.16          // standard temperature on sea level
#define SPT_P0   1013.25         // standard pressure on sea level


///< Constructor function, to be called when defining
void InitCTroposphere(CTroposphere* pStruct)
{
    if (!pStruct)return;

    pStruct->m_elev = 0.0;
    pStruct->m_T = 0.0;
    pStruct->m_P = 0.0;
    pStruct->m_e = 0.0;

    pStruct->m_mfh[0] = pStruct->m_mfh[1] = 0.0;
    pStruct->m_mfw[0] = pStruct->m_mfw[1] = 0.0;

    pStruct->m_mfg = 0.0;
    SetValsD(pStruct->m_Pnm[0], 0.0, 10 * 10);

    pStruct->m_STD = 0.0;
    pStruct->m_ZHD = 0.0;
    pStruct->m_ZWD = 0.0;
    pStruct->m_ZHDMF = 0.0;
    pStruct->m_ZWDMF = 0.0;
    pStruct->m_GRDMF = 0.0;
    pStruct->m_TropVar = 0.0;
}


/**
*     Standard temperature and pressure model
*     ----------
*     ghgt: geodetic height in m
*
*     output data
*     -----------
*     T   : Temperature(K)
*     P   : Pressure(mbar)
*     e   : water vapor pressure(mbar)
*
*/
static void SPT(CTroposphere* pStruct)
{
    if (!pStruct)return;

    double ghgt = pStruct->m_LLH[2];

    if (ghgt < 0.0)
        ghgt = 0.0;

    pStruct->m_T = SPT_T0K - 0.0065 * ghgt;

    pStruct->m_P = SPT_P0 * pow(1.0 - 2.2557E-5 * ghgt, 5.2568);

    // F. Kleijer,2004
    if (pStruct->m_T < SPT_TK)
        pStruct->m_e = exp(21.3195 - 5327.1157 / (pStruct->m_T));
    else
        pStruct->m_e = exp(24.3702 - 6162.3496 / (pStruct->m_T));

    double humi = 0.7;
    pStruct->m_e = 6.108 * humi * exp((17.15 * pStruct->m_T - 4684.0) / (pStruct->m_T - 38.45));
}


/**
* @brief       Using EGM97 to compute Geoid undulation
* @param[in]   glon        double   geodetic longitude(rad)
* @param[in]   glat        double   geodetic latitude(rad)
* @return      double      Geoid undulation in m
* @note        Geoid undulation from a 9x9 EGM based model
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double EGM97_N(double glon, double glat)
{
    const double a_geoid[55] = {
        -5.6195e-001,-6.0794e-002,-2.0125e-001,-6.4180e-002,-3.6997e-002,
        +1.0098e+001,+1.6436e+001,+1.4065e+001,+1.9881e+000,+6.4414e-001,
        -4.7482e+000,-3.2290e+000,+5.0652e-001,+3.8279e-001,-2.6646e-002,
        +1.7224e+000,-2.7970e-001,+6.8177e-001,-9.6658e-002,-1.5113e-002,
        +2.9206e-003,-3.4621e+000,-3.8198e-001,+3.2306e-002,+6.9915e-003,
        -2.3068e-003,-1.3548e-003,+4.7324e-006,+2.3527e+000,+1.2985e+000,
        +2.1232e-001,+2.2571e-002,-3.7855e-003,+2.9449e-005,-1.6265e-004,
        +1.1711e-007,+1.6732e+000,+1.9858e-001,+2.3975e-002,-9.0013e-004,
        -2.2475e-003,-3.3095e-005,-1.2040e-005,+2.2010e-006,-1.0083e-006,
        +8.6297e-001,+5.8231e-001,+2.0545e-002,-7.8110e-003,-1.4085e-004,
        -8.8459e-006,+5.7256e-006,-1.5068e-006,+4.0095e-007,-2.4185e-008 };

    const double b_geoid[55] = {
        +0.0000e+000,+0.0000e+000,-6.5993e-002,+0.0000e+000,+6.5364e-002,
        -5.8320e+000,+0.0000e+000,+1.6961e+000,-1.3557e+000,+1.2694e+000,
        +0.0000e+000,-2.9310e+000,+9.4805e-001,-7.6243e-002,+4.1076e-002,
        +0.0000e+000,-5.1808e-001,-3.4583e-001,-4.3632e-002,+2.2101e-003,
        -1.0663e-002,+0.0000e+000,+1.0927e-001,-2.9463e-001,+1.4371e-003,
        -1.1452e-002,-2.8156e-003,-3.5330e-004,+0.0000e+000,+4.4049e-001,
        +5.5653e-002,-2.0396e-002,-1.7312e-003,+3.5805e-005,+7.2682e-005,
        +2.2535e-006,+0.0000e+000,+1.9502e-002,+2.7919e-002,-8.1812e-003,
        +4.4540e-004,+8.8663e-005,+5.5596e-005,+2.4826e-006,+1.0279e-006,
        +0.0000e+000,+6.0529e-002,-3.5824e-002,-5.1367e-003,+3.0119e-005,
        -2.9911e-005,+1.9844e-005,-1.2349e-006,-7.6756e-009,+5.0100e-008 };

    double t = sin(glat);
    double aP, bP;
    int i = 0;
    double undu = 0.0;

    double P[10][10] = { {0.0} };
    Pnm(9, 9, t, P[0]);

    for (int n = 0; n <= 9; n++)
        for (int m = 0; m <= n; m++)
        {
            aP = P[n][m] * cos(m * glon);
            bP = P[n][m] * sin(m * glon);
            undu += a_geoid[i] * aP + b_geoid[i] * bP;
            i++;
        }

    return undu;
}


/**
*     Global Temperature and Pressure Model
*     input data
*     ----------
*     mjd : modified julian date, if mjd < 44239.0, input doy
*     glon: geodetic longitude in radians
*     glat: geodetic latitude  in radians
*     ghgt: geodetic height in m
*
*     output data
*     -----------
*     un  : Geoid undulation in m (from a 9x9 EGM based model)
*     P   : pressure in hPa
*     T   : temperature in K
*
*/
static bool GPT(CTroposphere* pStruct)
{
    if (!pStruct)return false;

    MJD jd = GPST2MJD(pStruct->m_utc);
    double mjd = jd.mjd + jd.fracOfDay;

    double glat = pStruct->m_LLH[0];
    double glon = pStruct->m_LLH[1];
    double ghgt = pStruct->m_LLH[2];

    if (ghgt < 0.0)
        ghgt = 0.0;

    // 大地高转化为正常高
    double h;
    double un = EGM97_N(glon, glat);
    h = ghgt - un;


    const double ap_mean[55] = {
        +1.0108e+003, +8.4886e+000, +1.4799e+000, -1.3897e+001, +3.7516e-003,
        -1.4936e-001, +1.2232e+001, -7.6615e-001, -6.7699e-002, +8.1002e-003,
        -1.5874e+001, +3.6614e-001, -6.7807e-002, -3.6309e-003, +5.9966e-004,
        +4.8163e+000, -3.7363e-001, -7.2071e-002, +1.9998e-003, -6.2385e-004,
        -3.7916e-004, +4.7609e+000, -3.9534e-001, +8.6667e-003, +1.1569e-002,
        +1.1441e-003, -1.4193e-004, -8.5723e-005, +6.5008e-001, -5.0889e-001,
        -1.5754e-002, -2.8305e-003, +5.7458e-004, +3.2577e-005, -9.6052e-006,
        -2.7974e-006, +1.3530e+000, -2.7271e-001, -3.0276e-004, +3.6286e-003,
        -2.0398e-004, +1.5846e-005, -7.7787e-006, +1.1210e-006, +9.9020e-008,
        +5.5046e-001, -2.7312e-001, +3.2532e-003, -2.4277e-003, +1.1596e-004,
        +2.6421e-007, -1.3263e-006, +2.7322e-007, +1.4058e-007, +4.9414e-009 };

    const double bp_mean[55] = {
        +0.0000e+000, +0.0000e+000, -1.2878e+000, +0.0000e+000, +7.0444e-001,
        +3.3222e-001, +0.0000e+000, -2.9636e-001, +7.2248e-003, +7.9655e-003,
        +0.0000e+000, +1.0854e+000, +1.1145e-002, -3.6513e-002, +3.1527e-003,
        +0.0000e+000, -4.8434e-001, +5.2023e-002, -1.3091e-002, +1.8515e-003,
        +1.5422e-004, +0.0000e+000, +6.8298e-001, +2.5261e-003, -9.9703e-004,
        -1.0829e-003, +1.7688e-004, -3.1418e-005, +0.0000e+000, -3.7018e-001,
        +4.3234e-002, +7.2559e-003, +3.1516e-004, +2.0024e-005, -8.0581e-006,
        -2.3653e-006, +0.0000e+000, +1.0298e-001, -1.5086e-002, +5.6186e-003,
        +3.2613e-005, +4.0567e-005, -1.3925e-006, -3.6219e-007, -2.0176e-008,
        +0.0000e+000, -1.8364e-001, +1.8508e-002, +7.5016e-004, -9.6139e-005,
        -3.1995e-006, +1.3868e-007, -1.9486e-007, +3.0165e-010, -6.4376e-010 };

    const double ap_amp[55] = {
        -1.0444e-001, +1.6618e-001, -6.3974e-002, +1.0922e+000, +5.7472e-001,
        -3.0277e-001, -3.5087e+000, +7.1264e-003, -1.4030e-001, +3.7050e-002,
        +4.0208e-001, -3.0431e-001, -1.3292e-001, +4.6746e-003, -1.5902e-004,
        +2.8624e+000, -3.9315e-001, -6.4371e-002, +1.6444e-002, -2.3403e-003,
        +4.2127e-005, +1.9945e+000, -6.0907e-001, -3.5386e-002, -1.0910e-003,
        -1.2799e-004, +4.0970e-005, +2.2131e-005, -5.3292e-001, -2.9765e-001,
        -3.2877e-002, +1.7691e-003, +5.9692e-005, +3.1725e-005, +2.0741e-005,
        -3.7622e-007, +2.6372e+000, -3.1165e-001, +1.6439e-002, +2.1633e-004,
        +1.7485e-004, +2.1587e-005, +6.1064e-006, -1.3755e-008, -7.8748e-008,
        -5.9152e-001, -1.7676e-001, +8.1807e-003, +1.0445e-003, +2.3432e-004,
        +9.3421e-006, +2.8104e-006, -1.5788e-007, -3.0648e-008, +2.6421e-010 };

    const double bp_amp[55] = {
        +0.0000e+000, +0.0000e+000, +9.3340e-001, +0.0000e+000, +8.2346e-001,
        +2.2082e-001, +0.0000e+000, +9.6177e-001, -1.5650e-002, +1.2708e-003,
        +0.0000e+000, -3.9913e-001, +2.8020e-002, +2.8334e-002, +8.5980e-004,
        +0.0000e+000, +3.0545e-001, -2.1691e-002, +6.4067e-004, -3.6528e-005,
        -1.1166e-004, +0.0000e+000, -7.6974e-002, -1.8986e-002, +5.6896e-003,
        -2.4159e-004, -2.3033e-004, -9.6783e-006, +0.0000e+000, -1.0218e-001,
        -1.3916e-002, -4.1025e-003, -5.1340e-005, -7.0114e-005, -3.3152e-007,
        +1.6901e-006, +0.0000e+000, -1.2422e-002, +2.5072e-003, +1.1205e-003,
        -1.3034e-004, -2.3971e-005, -2.6622e-006, +5.7852e-007, +4.5847e-008,
        +0.0000e+000, +4.4777e-002, -3.0421e-003, +2.6062e-005, -7.2421e-005,
        +1.9119e-006, +3.9236e-007, +2.2390e-007, +2.9765e-009, -4.6452e-009 };


    const double at_mean[55] = {
        +1.6257e+001, +2.1224e+000, +9.2569e-001, -2.5974e+001, +1.4510e+000,
        +9.2468e-002, -5.3192e-001, +2.1094e-001, -6.9210e-002, -3.4060e-002,
        -4.6569e+000, +2.6385e-001, -3.6093e-002, +1.0198e-002, -1.8783e-003,
        +7.4983e-001, +1.1741e-001, +3.9940e-002, +5.1348e-003, +5.9111e-003,
        +8.6133e-006, +6.3057e-001, +1.5203e-001, +3.9702e-002, +4.6334e-003,
        +2.4406e-004, +1.5189e-004, +1.9581e-007, +5.4414e-001, +3.5722e-001,
        +5.2763e-002, +4.1147e-003, -2.7239e-004, -5.9957e-005, +1.6394e-006,
        -7.3045e-007, -2.9394e+000, +5.5579e-002, +1.8852e-002, +3.4272e-003,
        -2.3193e-005, -2.9349e-005, +3.6397e-007, +2.0490e-006, -6.4719e-008,
        -5.2225e-001, +2.0799e-001, +1.3477e-003, +3.1613e-004, -2.2285e-004,
        -1.8137e-005, -1.5177e-007, +6.1343e-007, +7.8566e-008, +1.0749e-009 };

    const double bt_mean[55] = {
        +0.0000e+000, +0.0000e+000, +1.0210e+000, +0.0000e+000, +6.0194e-001,
        +1.2292e-001, +0.0000e+000, -4.2184e-001, +1.8230e-001, +4.2329e-002,
        +0.0000e+000, +9.3312e-002, +9.5346e-002, -1.9724e-003, +5.8776e-003,
        +0.0000e+000, -2.0940e-001, +3.4199e-002, -5.7672e-003, -2.1590e-003,
        +5.6815e-004, +0.0000e+000, +2.2858e-001, +1.2283e-002, -9.3679e-003,
        -1.4233e-003, -1.5962e-004, +4.0160e-005, +0.0000e+000, +3.6353e-002,
        -9.4263e-004, -3.6762e-003, +5.8608e-005, -2.6391e-005, +3.2095e-006,
        -1.1605e-006, +0.0000e+000, +1.6306e-001, +1.3293e-002, -1.1395e-003,
        +5.1097e-005, +3.3977e-005, +7.6449e-006, -1.7602e-007, -7.6558e-008,
        +0.0000e+000, -4.5415e-002, -1.8027e-002, +3.6561e-004, -1.1274e-004,
        +1.3047e-005, +2.0001e-006, -1.5152e-007, -2.7807e-008, +7.7491e-009 };

    const double at_amp[55] = {
        -1.8654e+000, -9.0041e+000, -1.2974e-001, -3.6053e+000, +2.0284e-002,
        +2.1872e-001, -1.3015e+000, +4.0355e-001, +2.2216e-001, -4.0605e-003,
        +1.9623e+000, +4.2887e-001, +2.1437e-001, -1.0061e-002, -1.1368e-003,
        -6.9235e-002, +5.6758e-001, +1.1917e-001, -7.0765e-003, +3.0017e-004,
        +3.0601e-004, +1.6559e+000, +2.0722e-001, +6.0013e-002, +1.7023e-004,
        -9.2424e-004, +1.1269e-005, -6.9911e-006, -2.0886e+000, -6.7879e-002,
        -8.5922e-004, -1.6087e-003, -4.5549e-005, +3.3178e-005, -6.1715e-006,
        -1.4446e-006, -3.7210e-001, +1.5775e-001, -1.7827e-003, -4.4396e-004,
        +2.2844e-004, -1.1215e-005, -2.1120e-006, -9.6421e-007, -1.4170e-008,
        +7.8720e-001, -4.4238e-002, -1.5120e-003, -9.4119e-004, +4.0645e-006,
        -4.9253e-006, -1.8656e-006, -4.0736e-007, -4.9594e-008, +1.6134e-009 };

    const double bt_amp[55] = {
        +0.0000e+000, +0.0000e+000, -8.9895e-001, +0.0000e+000, -1.0790e+000,
        -1.2699e-001, +0.0000e+000, -5.9033e-001, +3.4865e-002, -3.2614e-002,
        +0.0000e+000, -2.4310e-002, +1.5607e-002, -2.9833e-002, -5.9048e-003,
        +0.0000e+000, +2.8383e-001, +4.0509e-002, -1.8834e-002, -1.2654e-003,
        -1.3794e-004, +0.0000e+000, +1.3306e-001, +3.4960e-002, -3.6799e-003,
        -3.5626e-004, +1.4814e-004, +3.7932e-006, +0.0000e+000, +2.0801e-001,
        +6.5640e-003, -3.4893e-003, -2.7395e-004, +7.4296e-005, -7.9927e-006,
        -1.0277e-006, +0.0000e+000, +3.6515e-002, -7.4319e-003, -6.2873e-004,
        -8.2461e-005, +3.1095e-005, -5.3860e-007, -1.2055e-007, -1.1517e-007,
        +0.0000e+000, +3.1404e-002, +1.5580e-002, -1.1428e-003, +3.3529e-005,
        +1.0387e-005, -1.9378e-006, -2.7327e-007, +7.5833e-009, -9.2323e-009 };

    double doy = mjd - 44239.0 + 1 - 28;
    if (mjd < 44239.0)
        doy = mjd - 28;

    double cos_doy = cos(doy / 365.25 * 2 * PI);

    int i = 0;
    double p_mean = 0.0, p_amp = 0.0;
    double T_mean = 0.0, T_amp = 0.0;
    double ap = 0.0, bp = 0.0;
    double t = sin(glat);

    Pnm(9, 9, t, pStruct->m_Pnm[0]);

    for (int n = 0; n <= 9; n++)
    {
        for (int m = 0; m <= n; m++)
        {
            ap = pStruct->m_Pnm[n][m] * cos(m * glon);
            bp = pStruct->m_Pnm[n][m] * sin(m * glon);
            p_mean += ap * ap_mean[i] + bp * bp_mean[i];
            p_amp += ap * ap_amp[i] + bp * bp_amp[i];
            T_mean += ap * at_mean[i] + bp * bt_mean[i];
            T_amp += ap * at_amp[i] + bp * bt_amp[i];
            i++;
        }
    }


    double p0 = p_mean + p_amp * cos_doy;
    pStruct->m_P = p0 * pow(1 - 0.00002260 * h, 5.225);

    double T0 = T_mean + T_amp * cos_doy;
    pStruct->m_T = T0 - 0.0065 * h;
    pStruct->m_T = pStruct->m_T + 273.16;

    // F. Kleijer,2004
    if (pStruct->m_T < SPT_TK)
        pStruct->m_e = exp(21.3195 - 5327.1157 / (pStruct->m_T));
    else
        pStruct->m_e = exp(24.3702 - 6162.3496 / (pStruct->m_T));

    return true;
}


// three order Continued Fractions Derivation
// f =( 1+a/(1+b/(1+c)) )/( sine+a/(sine+b/(sine+c)) )
static double cfd3(double a, double b, double c, double E)
{
    double topcon = a / (b / (c + 1) + 1) + 1;
    double cose = cos(E);
    double sine = sin(E);
    double tmp3 = c + sine;
    double tmp1 = sine + b / tmp3;
    double tmp2 = sine + a / tmp1;

    double df = -(topcon * (cose - (a * (cose - (b * cose) / tmp3)) / tmp1)) / tmp2;
    return df;
}


/**
*     Saastamoinen model to compute  zenith troposhere delay
*     ----------
*     glat: geodetic latitude  in radians
*     ghgt: geodetic height in m
*     T   : Temperature(K)
*     P   : Pressure(mbar)
*     e   : water vapor pressure(mbar)
*
*     output data
*     -----------
*     zenith_delay(2): zenith dry delay ande zenith wet delay
*
*/
static void Saas(CTroposphere* pStruct)
{
    if (!pStruct)return;

    double glat = pStruct->m_LLH[0];
    double ghgt = pStruct->m_LLH[2] < 0 ? 0.0 : pStruct->m_LLH[2];

    double f_BH = (1 - 0.00266 * cos(2 * glat) - 0.00028 * ghgt / 1000.0);

    pStruct->m_ZHD = 0.0022768 * pStruct->m_P / f_BH;

    pStruct->m_ZWD = 0.002277 * pStruct->m_e * (1255.0 / pStruct->m_T + 0.05);
}


static void SaasSPT(CTroposphere* pStruct)
{
    if (!pStruct)return;

    SPT(pStruct);
    Saas(pStruct);
}


static void SaasGPT(CTroposphere* pStruct)
{
    if (!pStruct)return;

    GPT(pStruct);
    Saas(pStruct);
}


void UNB3M(CTroposphere* pStruct)
{
    if (!pStruct)return;

    double glat = pStruct->m_LLH[0];
    double ghgt = pStruct->m_LLH[2] < 0 ? 0.0 : pStruct->m_LLH[2]; // UNB需要的是正高

    YDOYHMS dy = GPST2YDHMS(pStruct->m_utc);
    int doy = dy.yday;

    const double pi = 3.1415926535897932;
    //=======================================================================
    // Initialize UNB3m look-up table
    //-----------------------------------------------------------------------
    //      lat      P       T      RH    beta lambda
    const double AVG[5][6] = {
        {15.0, 1013.25, 299.65, 75.00, 6.30, 2.77},
        {30.0, 1017.25, 294.15, 80.00, 6.05, 3.15},
        {45.0, 1015.75, 283.15, 76.00, 5.58, 2.57},
        {60.0, 1011.75, 272.15, 77.50, 5.39, 1.81},
        {75.0, 1013.00, 263.65, 82.50, 4.53, 1.55}
    };

    //      lat     P      T      RH    beta lambda
    const double AMP[5][6] = {
        {15.0, 0.00, 0.00, 0.00, 0.00, 0.00},
        {30.0, -3.75, 7.00, 0.00, 0.25, 0.33},
        {45.0, -2.25, 11.00, -1.00, 0.32, 0.46},
        {60.0, -1.75, 15.00, -2.50, 0.81, 0.74},
        {75.0, -0.50, 14.50, 2.50, 0.62, 0.30}
    };

    //=======================================================================
    const double EXCEN2 = 6.6943799901413e-03;
    const double MD = 28.9644;
    const double MW = 18.0152;
    const double K1 = 77.604;
    const double K2 = 64.79;
    const double K3 = 3.776e5;
    const double R = 8314.34;
    const double C1 = 2.2768e-03;
    const double K2PRIM = K2 - K1 * (MW / MD);
    const double RD = R / MD;
    //const double DTR = 1.745329251994329e-02;
    const double DOY2RAD = (0.31415926535897935601e01) * 2 / 365.25;


    //=======================================================================
    // Transform latitude from radians to decimal degrees
    //-----------------------------------------------------------------------
    const double LATDEG = glat * (180.0 / pi);
    //=======================================================================
    //
    //
    //=======================================================================
    // Deal with southern hemisphere and yearly variation
    //-----------------------------------------------------------------------
    double TD_O_Y = doy;

    if (LATDEG < 0)
        TD_O_Y = TD_O_Y + 182.625;

    double COSPHS = cos((TD_O_Y - 28) * DOY2RAD);
    //=======================================================================
    //
    //
    //=======================================================================
    // Initialize pointers to lookup table
    //-----------------------------------------------------------------------
    double LAT = fabs(LATDEG);

    int P1 = 0;
    int P2 = 0;
    double M = 0.0;

    if (LAT >= 75)
    {
        P1 = 4;
        P2 = 4;
        M = 0;
    }
    else if (LAT <= 15)
    {
        P1 = 0;
        P2 = 0;
        M = 0;
    }
    else
    {
        P1 = (int)((LAT - 15) / 15.0);
        P2 = P1 + 1;
        M = (LAT - AVG[P1][0]) / (AVG[P2][0] - AVG[P1][0]);
    }

    //=======================================================================
    //
    //
    //=======================================================================
    // Compute average surface tropo values by interpolation
    //-----------------------------------------------------------------------
    double PAVG = M * (AVG[P2][1] - AVG[P1][1]) + AVG[P1][1];
    double TAVG = M * (AVG[P2][2] - AVG[P1][2]) + AVG[P1][2];
    double EAVG = M * (AVG[P2][3] - AVG[P1][3]) + AVG[P1][3];
    double BETAAVG = M * (AVG[P2][4] - AVG[P1][4]) + AVG[P1][4];
    double LAMBDAAVG = M * (AVG[P2][5] - AVG[P1][5]) + AVG[P1][5];
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute variation of average surface tropo values
    //-----------------------------------------------------------------------
    double PAMP = M * (AMP[P2][1] - AMP[P1][1]) + AMP[P1][1];
    double TAMP = M * (AMP[P2][2] - AMP[P1][2]) + AMP[P1][2];
    double EAMP = M * (AMP[P2][3] - AMP[P1][3]) + AMP[P1][3];
    double BETAAMP = M * (AMP[P2][4] - AMP[P1][4]) + AMP[P1][4];
    double LAMBDAAMP = M * (AMP[P2][5] - AMP[P1][5]) + AMP[P1][5];
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute surface tropo values
    //-----------------------------------------------------------------------
    double P0 = PAVG - PAMP * COSPHS;
    double T0 = TAVG - TAMP * COSPHS;
    double E0 = EAVG - EAMP * COSPHS;
    double BETA = BETAAVG - BETAAMP * COSPHS;
    BETA = BETA / 1000.0;
    double LAMBDA = LAMBDAAVG - LAMBDAAMP * COSPHS;
    //=======================================================================
    //
    //
    //=======================================================================
    // Transform from relative humidity to WVP (IERS Conventions 2003)
    //-----------------------------------------------------------------------
    double ES = 0.01 * exp(1.2378847e-5 * (T0 * T0) - 1.9121316e-2 * T0 + 3.393711047e1 - 6.3431645e3 * (1 / T0));
    double FW = 1.00062 + 3.14e-6 * P0 + 5.6e-7 * ((T0 - 273.15) * (T0 - 273.15));
    E0 = (E0 / 1.00e2) * ES * FW;
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute power value for pressure & water vapour
    //-----------------------------------------------------------------------
    double EP = 9.80665 / 287.054 / BETA;
    //=======================================================================
    //
    //
    //=======================================================================
    // Scale surface values to required height
    //-----------------------------------------------------------------------
    double T = T0 - BETA * ghgt;
    double P = P0 * pow((T / T0), EP);
    double E = E0 * pow((T / T0), (EP * (LAMBDA + 1)));
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute the acceleration at the mass center
    // of a vertical column of the atmosphere
    //-----------------------------------------------------------------------
    double GEOLAT = atan((1.0 - EXCEN2) * tan(glat));
    double DGREF = 1.0 - 2.66e-03 * cos(2.0 * GEOLAT) - 2.8e-07 * ghgt;
    double GM = 9.784 * DGREF;
    double DEN = (LAMBDA + 1.0) * GM;
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute mean temperature of the water vapor
    //-----------------------------------------------------------------------
    double TM = T * (1 - BETA * RD / DEN);
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute zenith hydrostatic delay
    //-----------------------------------------------------------------------
    pStruct->m_ZHD = C1 / DGREF * P;
    //=======================================================================
    //
    //
    //=======================================================================
    // Compute zenith wet delay
    //-----------------------------------------------------------------------
    pStruct->m_ZWD = 1.0e-6 * (K2PRIM + K3 / TM) * RD * E / DEN;
    //=======================================================================

}




/**
*     Neill Mapping Functions
*     ----------
*     mjd : modified julian date
*     glat: geodetic latitude  in radians
*     ghgt: geodetic height in m
*     e   : elevation in radians
*
*     output data
*     -----------
*     nmfh(2): hydrostatic mapping function and derivative
*     nmfw(2): wet mapping function and derivative
*
*/
static bool NMF(CTroposphere* pStruct)
{
    if (!pStruct)return false;

    YDOYHMS ydoy = GPST2YDHMS(pStruct->m_utc);

    double glat = pStruct->m_LLH[0];
    double ghgt = pStruct->m_LLH[2];


    glat *= R2D;
    // Parameters borrowed from Saastamoinen tropospheric model
    // Constants for wet mapping function
    const double NeillWetA[5] = { 0.00058021897, 0.00056794847, 0.00058118019, 0.00059727542, 0.00061641693 };
    const double NeillWetB[5] = { 0.0014275268, 0.0015138625, 0.0014572752, 0.0015007428, 0.0017599082 };
    const double NeillWetC[5] = { 0.043472961, 0.046729510, 0.043908931, 0.044626982, 0.054736038 };

    // constants for dry mapping function
    const double NeillDryA[5] = { 0.0012769934, 0.0012683230, 0.0012465397, 0.0012196049, 0.0012045996 };
    const double NeillDryB[5] = { 0.0029153695, 0.0029152299, 0.0029288445, 0.0029022565, 0.0029024912 };
    const double NeillDryC[5] = { 0.062610505, 0.062837393, 0.063721774, 0.063824265, 0.064258455 };

    const double NeillDryA1[5] = { 0.0, 0.000012709626, 0.000026523662, 0.000034000452, 0.000041202191 };
    const double NeillDryB1[5] = { 0.0, 0.000021414979, 0.000030160779, 0.000072562722, 0.00011723375 };
    const double NeillDryC1[5] = { 0.0, 0.000090128400, 0.000043497037, 0.00084795348, 0.0017037206 };


    if (pStruct->m_elev < 2.99999999 * D2R)
    {
        return false;
    }

    double dy = ydoy.yday + (ydoy.hour * 3600.0 + ydoy.min * 60.0 + ydoy.sec) / 86400.0;
    double y = (dy - 28.0) / 365.25 + (glat < 0.0 ? 0.5 : 0.0);

    double cos_doy = cos(2 * PI * y);

    glat = fabs(glat); //-geoidh(pos);/* height in m (mean sea level) */

    double a, b, c;
    if (glat < 15.0)
    {
        a = NeillDryA[0];
        b = NeillDryB[0];
        c = NeillDryC[0];
    }
    else if (glat < 75.)      // coefficients are for 15,30,45,60,75 deg
    {
        int i = (int)(glat / 15.0) - 1;
        double frac = (glat - 15.0 * (i + 1)) / 15.0;

        a = NeillDryA[i] + frac * (NeillDryA[i + 1] - NeillDryA[i]);
        b = NeillDryB[i] + frac * (NeillDryB[i + 1] - NeillDryB[i]);
        c = NeillDryC[i] + frac * (NeillDryC[i + 1] - NeillDryC[i]);

        a -= cos_doy * (NeillDryA1[i] + frac * (NeillDryA1[i + 1] - NeillDryA1[i]));
        b -= cos_doy * (NeillDryB1[i] + frac * (NeillDryB1[i + 1] - NeillDryB1[i]));
        c -= cos_doy * (NeillDryC1[i] + frac * (NeillDryC1[i + 1] - NeillDryC1[i]));
    }
    else
    {
        a = NeillDryA[4] - cos_doy * NeillDryA1[4];
        b = NeillDryB[4] - cos_doy * NeillDryB1[4];
        c = NeillDryC[4] - cos_doy * NeillDryC1[4];
    }

    double sine = sin(pStruct->m_elev);
    double cose = cos(pStruct->m_elev);

    pStruct->m_mfh[0] = (1. + a / (1. + b / (1. + c))) / (sine + a / (sine + b / (sine + c)));
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfh[1] = cfd3(a, b, c, pStruct->m_elev);

    a = 0.0000253;
    b = 0.00549;
    c = 0.00114;
    //hydro
    double hs_km = ghgt / 1000.0;
    //double dm = hs_km * (1. / sine - ((1. + a / (1. + b / (1. + c))) / (sine + a / (sine + b / (sine + c)))));
    pStruct->m_mfh[0] += hs_km * (1. / sine - ((1. + a / (1. + b / (1. + c))) / (sine + a / (sine + b / (sine + c)))));
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfh[1] += (-cose / sine / sine - cfd3(a, b, c, pStruct->m_elev)) * hs_km;


    if (glat < 15.0)
    {
        a = NeillWetA[0];
        b = NeillWetB[0];
        c = NeillWetC[0];
    }
    else if (glat < 75.)          // coefficients are for 15,30,45,60,75 deg
    {
        int i = (int)(glat / 15.0) - 1;
        double frac = (glat - 15. * (i + 1)) / 15.;
        a = NeillWetA[i] + frac * (NeillWetA[i + 1] - NeillWetA[i]);
        b = NeillWetB[i] + frac * (NeillWetB[i + 1] - NeillWetB[i]);
        c = NeillWetC[i] + frac * (NeillWetC[i + 1] - NeillWetC[i]);
    }
    else
    {
        a = NeillWetA[4];
        b = NeillWetB[4];
        c = NeillWetC[4];
    }

    //wet
    pStruct->m_mfw[0] = (1. + a / (1. + b / (1. + c))) / (sine + a / (sine + b / (sine + c)));
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfw[1] = cfd3(a, b, c, pStruct->m_elev);

    return true;
}


/**
*     Global Mapping Functions
*     ----------
*     mjd : modified julian date
*     glon: geodetic longitude in radians
*     glat: geodetic latitude  in radians
*     ghgt: geodetic height in m
*     e   : elevation in radians
*
*     output data
*     -----------
*     gmfh(2): hydrostatic mapping function and derivative
*     gmfw(2): wet mapping function and derivative
*
*/
static bool GMF(CTroposphere* pStruct)
{
    if (!pStruct)return false;

    MJD jd = GPST2MJD(pStruct->m_utc);
    double mjd = jd.mjd + jd.fracOfDay;

    double glat = pStruct->m_LLH[0];
    double glon = pStruct->m_LLH[1];
    double ghgt = pStruct->m_LLH[2];

    double doy = mjd - 44239.0 + 1 - 28;
    if (mjd < 44239.0)
        doy = mjd - 28;

    const double ah_mean[55] =
    {
        +1.2517e+02, +8.503e-01, +6.936e-02, -6.760e+00, +1.771e-01,
        +1.130e-02, +5.963e-01, +1.808e-02, +2.801e-03, -1.414e-03,
        -1.212e+00, +9.300e-02, +3.683e-03, +1.095e-03, +4.671e-05,
        +3.959e-01, -3.867e-02, +5.413e-03, -5.289e-04, +3.229e-04,
        +2.067e-05, +3.000e-01, +2.031e-02, +5.900e-03, +4.573e-04,
        -7.619e-05, +2.327e-06, +3.845e-06, +1.182e-01, +1.158e-02,
        +5.445e-03, +6.219e-05, +4.204e-06, -2.093e-06, +1.540e-07,
        -4.280e-08, -4.751e-01, -3.490e-02, +1.758e-03, +4.019e-04,
        -2.799e-06, -1.287e-06, +5.468e-07, +7.580e-08, -6.300e-09,
        -1.160e-01, +8.301e-03, +8.771e-04, +9.955e-05, -1.718e-06,
        -2.012e-06, +1.170e-08, +1.790e-08, -1.300e-09, +1.000e-10
    };

    const double bh_mean[55] =
    {
        +0.000e+00, +0.000e+00, +3.249e-02, +0.000e+00, +3.324e-02,
        +1.850e-02, +0.000e+00, -1.115e-01, +2.519e-02, +4.923e-03,
        +0.000e+00, +2.737e-02, +1.595e-02, -7.332e-04, +1.933e-04,
        +0.000e+00, -4.796e-02, +6.381e-03, -1.599e-04, -3.685e-04,
        +1.815e-05, +0.000e+00, +7.033e-02, +2.426e-03, -1.111e-03,
        -1.357e-04, -7.828e-06, +2.547e-06, +0.000e+00, +5.779e-03,
        +3.133e-03, -5.312e-04, -2.028e-05, +2.323e-07, -9.100e-08,
        -1.650e-08, +0.000e+00, +3.688e-02, -8.638e-04, -8.514e-05,
        -2.828e-05, +5.403e-07, +4.390e-07, +1.350e-08, +1.800e-09,
        +0.000e+00, -2.736e-02, -2.977e-04, +8.113e-05, +2.329e-07,
        +8.451e-07, +4.490e-08, -8.100e-09, -1.500e-09, +2.000e-10
    };

    const double ah_amp[55] =
    {
        -2.738e-01, -2.837e+00, +1.298e-02, -3.588e-01, +2.413e-02,
        +3.427e-02, -7.624e-01, +7.272e-02, +2.160e-02, -3.385e-03,
        +4.424e-01, +3.722e-02, +2.195e-02, -1.503e-03, +2.426e-04,
        +3.013e-01, +5.762e-02, +1.019e-02, -4.476e-04, +6.790e-05,
        +3.227e-05, +3.123e-01, -3.535e-02, +4.840e-03, +3.025e-06,
        -4.363e-05, +2.854e-07, -1.286e-06, -6.725e-01, -3.730e-02,
        +8.964e-04, +1.399e-04, -3.990e-06, +7.431e-06, -2.796e-07,
        -1.601e-07, +4.068e-02, -1.352e-02, +7.282e-04, +9.594e-05,
        +2.070e-06, -9.620e-08, -2.742e-07, -6.370e-08, -6.300e-09,
        +8.625e-02, -5.971e-03, +4.705e-04, +2.335e-05, +4.226e-06,
        +2.475e-07, -8.850e-08, -3.600e-08, -2.900e-09, +0.000e+00
    };

    const double bh_amp[55] =
    {
        +0.000e+00, +0.000e+00, -1.136e-01, +0.000e+00, -1.868e-01,
        -1.399e-02, +0.000e+00, -1.043e-01, +1.175e-02, -2.240e-03,
        +0.000e+00, -3.222e-02, +1.333e-02, -2.647e-03, -2.316e-05,
        +0.000e+00, +5.339e-02, +1.107e-02, -3.116e-03, -1.079e-04,
        -1.299e-05, +0.000e+00, +4.861e-03, +8.891e-03, -6.448e-04,
        -1.279e-05, +6.358e-06, -1.417e-07, +0.000e+00, +3.041e-02,
        +1.150e-03, -8.743e-04, -2.781e-05, +6.367e-07, -1.140e-08,
        -4.200e-08, +0.000e+00, -2.982e-02, -3.000e-03, +1.394e-05,
        -3.290e-05, -1.705e-07, +7.440e-08, +2.720e-08, -6.600e-09,
        +0.000e+00, +1.236e-02, -9.981e-04, -3.792e-05, -1.355e-05,
        +1.162e-06, -1.789e-07, +1.470e-08, -2.400e-09, -4.000e-10
    };

    const double aw_mean[55] =
    {
        +5.640e+01, +1.555e+00, -1.011e+00, -3.975e+00, +3.171e-02,
        +1.065e-01, +6.175e-01, +1.376e-01, +4.229e-02, +3.028e-03,
        +1.688e+00, -1.692e-01, +5.478e-02, +2.473e-02, +6.059e-04,
        +2.278e+00, +6.614e-03, -3.505e-04, -6.697e-03, +8.402e-04,
        +7.033e-04, -3.236e+00, +2.184e-01, -4.611e-02, -1.613e-02,
        -1.604e-03, +5.420e-05, +7.922e-05, -2.711e-01, -4.406e-01,
        -3.376e-02, -2.801e-03, -4.090e-04, -2.056e-05, +6.894e-06,
        +2.317e-06, +1.941e+00, -2.562e-01, +1.598e-02, +5.449e-03,
        +3.544e-04, +1.148e-05, +7.503e-06, -5.667e-07, -3.660e-08,
        +8.683e-01, -5.931e-02, -1.864e-03, -1.277e-04, +2.029e-04,
        +1.269e-05, +1.629e-06, +9.660e-08, -1.015e-07, -5.000e-10
    };

    const double bw_mean[55] =
    {
        +0.000e+00, +0.000e+00, +2.592e-01, +0.000e+00, +2.974e-02,
        -5.471e-01, +0.000e+00, -5.926e-01, -1.030e-01, -1.567e-02,
        +0.000e+00, +1.710e-01, +9.025e-02, +2.689e-02, +2.243e-03,
        +0.000e+00, +3.439e-01, +2.402e-02, +5.410e-03, +1.601e-03,
        +9.669e-05, +0.000e+00, +9.502e-02, -3.063e-02, -1.055e-03,
        -1.067e-04, -1.130e-04, +2.124e-05, +0.000e+00, -3.129e-01,
        +8.463e-03, +2.253e-04, +7.413e-05, -9.376e-05, -1.606e-06,
        +2.060e-06, +0.000e+00, +2.739e-01, +1.167e-03, -2.246e-05,
        -1.287e-04, -2.438e-05, -7.561e-07, +1.158e-06, +4.950e-08,
        +0.000e+00, -1.344e-01, +5.342e-03, +3.775e-04, -6.756e-05,
        -1.686e-06, -1.184e-06, +2.768e-07, +2.730e-08, +5.700e-09
    };

    const double aw_amp[55] =
    {
        +1.023e-01, -2.695e+00, +3.417e-01, -1.405e-01, +3.175e-01,
        +2.116e-01, +3.536e+00, -1.505e-01, -1.660e-02, +2.967e-02,
        +3.819e-01, -1.695e-01, -7.444e-02, +7.409e-03, -6.262e-03,
        -1.836e+00, -1.759e-02, -6.256e-02, -2.371e-03, +7.947e-04,
        +1.501e-04, -8.603e-01, -1.360e-01, -3.629e-02, -3.706e-03,
        -2.976e-04, +1.857e-05, +3.021e-05, +2.248e+00, -1.178e-01,
        +1.255e-02, +1.134e-03, -2.161e-04, -5.817e-06, +8.836e-07,
        -1.769e-07, +7.313e-01, -1.188e-01, +1.145e-02, +1.011e-03,
        +1.083e-04, +2.570e-06, -2.140e-06, -5.710e-08, +2.000e-08,
        -1.632e+00, -6.948e-03, -3.893e-03, +8.592e-04, +7.577e-05,
        +4.539e-06, -3.852e-07, -2.213e-07, -1.370e-08, +5.800e-09
    };

    const double bw_amp[55] =
    {
        +0.000e+00, +0.000e+00, -8.865e-02, +0.000e+00, -4.309e-01,
        +6.340e-02, +0.000e+00, +1.162e-01, +6.176e-02, -4.234e-03,
        +0.000e+00, +2.530e-01, +4.017e-02, -6.204e-03, +4.977e-03,
        +0.000e+00, -1.737e-01, -5.638e-03, +1.488e-04, +4.857e-04,
        -1.809e-04, +0.000e+00, -1.514e-01, -1.685e-02, +5.333e-03,
        -7.611e-05, +2.394e-05, +8.195e-06, +0.000e+00, +9.326e-02,
        -1.275e-02, -3.071e-04, +5.374e-05, -3.391e-05, -7.436e-06,
        +6.747e-07, +0.000e+00, -8.637e-02, -3.807e-03, -6.833e-04,
        -3.861e-05, -2.268e-05, +1.454e-06, +3.860e-07, -1.068e-07,
        +0.000e+00, -2.658e-02, -1.947e-03, +7.131e-04, -3.506e-05,
        +1.885e-07, +5.792e-07, +3.990e-08, +2.000e-08, -5.700e-09
    };

    const double t = sin(glat);

    int n = 9;

    double ahm = 0.0;
    double aha = 0.0;
    double awm = 0.0;
    double awa = 0.0;

    Pnm(9, 9, t, pStruct->m_Pnm[0]);

    int k = 0;
    for (int i = 0; i <= n; i++)
        for (int j = 0; j <= i; j++)
        {
            double cosml = cos(j * glon);
            double sinml = sin(j * glon);
            double p = pStruct->m_Pnm[i][j];
            ahm += p * (ah_mean[k] * cosml + bh_mean[k] * sinml);
            aha += p * (ah_amp[k] * cosml + bh_amp[k] * sinml);
            awm += p * (aw_mean[k] * cosml + bw_mean[k] * sinml);
            awa += p * (aw_amp[k] * cosml + bw_amp[k] * sinml);
            k++;
        }


    ahm *= 1E-5;
    aha *= 1E-5;
    awm *= 1E-5;
    awa *= 1E-5;

    // hydrostatic

    double bh = 0.0029;
    double c0h = 0.062;

    double c11h = 0.005;
    double c10h = 0.001;
    if (glat < 0)
    {
        c11h = 0.007;
        c10h = 0.002;
    }

    double cos_doy = cos(doy / 365.25 * 2 * PI);
    double ch = c0h + ((cos_doy + 1.0) * c11h / 2.0 + c10h) * (1 - cos(glat));

    double ah = ahm + aha * cos_doy;

    double cose = cos(pStruct->m_elev);
    double sine = sin(pStruct->m_elev);
    double beta = bh / (sine + ch);
    double gamma = ah / (sine + beta);
    double topcon = (1.0 + ah / (1.0 + bh / (1.0 + ch)));
    pStruct->m_mfh[0] = topcon / (sine + gamma);
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfh[1] = cfd3(ah, bh, ch, pStruct->m_elev);

    double a_ht = 2.53E-5;
    double b_ht = 5.49E-3;
    double c_ht = 1.14E-3;
    double hs_km = ghgt / 1000.0;

    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = (1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht)));
    double ht_corr = (1 / sine - topcon / (sine + gamma)) * hs_km;
    pStruct->m_mfh[0] += ht_corr;

    if (pStruct->m_GRDMF == TROP_BER)  pStruct->m_mfh[1] += (-cose / sine / sine - cfd3(a_ht, b_ht, c_ht, pStruct->m_elev)) * hs_km;

    // wet
    double bw = 0.00146;
    double cw = 0.04391;

    double aw = awm + awa * cos_doy;
    beta = bw / (sine + cw);
    gamma = aw / (sine + beta);
    topcon = (1.0 + aw / (1.0 + bw / (1.0 + cw)));
    pStruct->m_mfw[0] = topcon / (sine + gamma);
    if (pStruct->m_GRDMF == TROP_BER)  pStruct->m_mfw[1] = cfd3(aw, bw, cw, pStruct->m_elev);

    return true;
}


/**
*     Vienna Mapping Functions
*     ----------
*     ah  : hydrostatic coefficient a
*     aw  : wet coefficient a
*     mjd : modified julian date
*     glat: geodetic latitude  in radians
*     ghgt: geodetic height in m
*     e   : elevation in radians
*
*     output data
*     -----------
*     vmf1h(2): hydrostatic mapping function and derivative
*     vmf1w(2): wet mapping function and derivative
*
*/
static bool VMF1(CTroposphere* pStruct, double ah, double aw)
{
    if (!pStruct)return false;

    MJD jd = GPST2MJD(pStruct->m_utc);
    double mjd = jd.mjd + jd.fracOfDay;

    double glat = pStruct->m_LLH[0];
    //double glon = pStruct->m_LLH[1];
    double ghgt = pStruct->m_LLH[2];


    double doy = mjd - 44239.0 + 1 - 28;
    if (mjd < 44239.0)
        doy = mjd - 28;

    double bh = 0.0029;
    double c0h = 0.062;
    double phh = 0.0;
    double c11h = 0.005;
    double c10h = 0.001;
    if (glat < 0)
    {
        phh = PI;
        c11h = 0.007;
        c10h = 0.002;
    }

    double ch = c0h + ((cos(2 * PI * doy / 365.25 + phh) + 1.0) * c11h / 2 + c10h) * (1.0 - cos(glat));
    double sine = sin(pStruct->m_elev);
    double cose = cos(pStruct->m_elev);
    double beta = bh / (sine + ch);
    double gamma = ah / (sine + beta);
    double topcon = (1.0 + ah / (1.0 + bh / (1.0 + ch)));
    pStruct->m_mfh[0] = topcon / (sine + gamma);
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfh[1] = cfd3(ah, bh, ch, pStruct->m_elev);

    double a_ht = 2.53E-5;
    double b_ht = 5.49E-3;
    double c_ht = 1.14E-3;
    double hs_km = ghgt / 1000.0;

    beta = b_ht / (sine + c_ht);
    gamma = a_ht / (sine + beta);
    topcon = (1.0 + a_ht / (1.0 + b_ht / (1.0 + c_ht)));
    double ht_corr = (1 / sine - topcon / (sine + gamma)) * hs_km;
    pStruct->m_mfh[0] += ht_corr;

    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfh[1] += (-cose / sine / sine - cfd3(a_ht, b_ht, c_ht, pStruct->m_elev)) * hs_km;


    // wet
    double bw = 0.00146;
    double cw = 0.04391;
    beta = bw / (sine + cw);
    gamma = aw / (sine + beta);
    topcon = (1.0 + aw / (1.0 + bw / (1.0 + cw)));
    pStruct->m_mfw[0] = topcon / (sine + gamma);
    if (pStruct->m_GRDMF == TROP_BER) pStruct->m_mfw[1] = cfd3(aw, bw, cw, pStruct->m_elev);

    return true;
}


static bool GradMF(CTroposphere* pStruct)
{
    if (!pStruct)return false;

    if (pStruct->m_MFGType == TROP_NONE)
    {
        ;
    }
    else if (pStruct->m_MFGType == TROP_MIT) // MIT
    {
        pStruct->m_mfg = 1.0 / (sin(pStruct->m_elev) * tan(pStruct->m_elev) + 0.0032);
    }
    else if (pStruct->m_MFGType == TROP_JPL) // JPL
    {
        pStruct->m_mfg = pStruct->m_mfw[0] / tan(pStruct->m_elev);
    }
    else if (pStruct->m_MFGType == TROP_BER) // BERN
    {
        pStruct->m_mfg = -pStruct->m_mfw[1];
    }
    else
    {
        return false;
    }

    return true;
}


bool TropComputer(CTroposphere* pStruct, const GPSTIME gt, const double LLH[3], const double elev, bool bSPP, TROP_TYPE ZTDType, TROP_TYPE MFType, TROP_TYPE MFGType)
{
    if (!pStruct)return false;

    // !!!! 对于飞机战斗机数据,高程会在几万米高空,此时对流层计算还得修改

    pStruct->m_mfh[0] = pStruct->m_mfh[1] = 0.0;
    pStruct->m_mfw[0] = pStruct->m_mfw[1] = 0.0;
    pStruct->m_mfg = 0.0;
    pStruct->m_STD = 0.0;
    pStruct->m_ZHD = 0.0;
    pStruct->m_ZWD = 0.0;
    pStruct->m_ZHDMF = 0.0;
    pStruct->m_ZWDMF = 0.0;
    pStruct->m_GRDMF = 0.0;
    pStruct->m_TropVar = 0.0;

    pStruct->m_LLH[0] = LLH[0]; pStruct->m_LLH[1] = LLH[1]; pStruct->m_LLH[2] = LLH[2];
    pStruct->m_elev = elev;
    pStruct->m_utc = gt; // 不进行utc改正？？

    pStruct->m_ZTDType = ZTDType;
    pStruct->m_MFType = MFType;
    pStruct->m_MFGType = MFGType;

    if (gt.GPSWeek <= 0)
    {
        return false;
    }


    // 大地高满足要求时计算对流层延迟
    if (1E4 >= LLH[2] && LLH[2] >= -100.0)
    {
        if (pStruct->m_ZTDType == TROP_NONE)
        {
            ;
        }
        else if (pStruct->m_ZTDType == TROP_SS)
        {
            SaasSPT(pStruct);
            double a = ERR_SAAS / (sin(elev) + 0.1);
            pStruct->m_TropVar = a * a;
        }
        else if (pStruct->m_ZTDType == TROP_GS)
        {
            SaasGPT(pStruct);
            double a = ERR_SAAG / (sin(elev) + 0.1);
            pStruct->m_TropVar = a * a;
        }
        else if (pStruct->m_ZTDType == TROP_UM)
        {
            UNB3M(pStruct);
            double a = ERR_UNBM / (sin(elev) + 0.1);
            pStruct->m_TropVar = a * a;
        }
    }


    // 高度角满足要求计算投影函数
    if (pStruct->m_elev > 0.0)
    {
        if (pStruct->m_MFType == TROP_NONE)
        {
            pStruct->m_mfh[0] = pStruct->m_mfh[1] = 0.0;
            pStruct->m_mfw[0] = pStruct->m_mfw[1] = 0.0;
        }
        else if (pStruct->m_MFType == TROP_SMF)
        {
            double z = PI / 2.0 - elev;
            pStruct->m_mfh[0] = pStruct->m_mfw[0] = 1.0 / cos(z);
        }
        else if (pStruct->m_MFType == TROP_NMF)
        {
            NMF(pStruct);
        }
        else if (pStruct->m_MFType == TROP_GMF)
        {
            GMF(pStruct);
        }
        else if (pStruct->m_MFType == TROP_VMF1)
        {
            VMF1(pStruct, 0, 0);
        }

        GradMF(pStruct);
    }

    pStruct->m_ZHDMF = pStruct->m_mfh[0];
    pStruct->m_ZWDMF = pStruct->m_mfw[0];
    pStruct->m_GRDMF = pStruct->m_mfg;

    pStruct->m_STD = pStruct->m_ZHDMF * pStruct->m_ZHD + pStruct->m_ZWDMF * pStruct->m_ZWD;

    return true;
}



