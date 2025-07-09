#include "CEphemerisComputer.h"
#include "GNSSCmnFunc.h"

#define MAXDTOE_GPS 7200.0        /* max time difference to ephem Toe (s) for GPS,BDS,GAL */
#define MAXDTOE_GLO 1800.0        /* max time difference to GLONASS Toe (s) */
#define MAXDTOE_BD2 21600.0       /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_BD3 21600.0       /* max time difference to BeiDou Toe (s) */
#define MAXDTOE_GAL 10800.0       /* max time difference to Galileo Toe (s) */
#define MAXDTOE_QZS 7200.0        /* max time difference to QZSS Toe (s) */

#define OMGE_GPS 7.2921151467E-5  /* earth angular velocity (IS-GPS) (rad/s) */
#define OMGE_GLO 7.292115E-5      /* earth angular velocity (rad/s) ref [2] */
#define OMGE_BDS 7.292115E-5      /* earth angular velocity (rad/s) ref [9] */
#define OMGE_GAL 7.2921151467E-5  /* earth angular velocity (rad/s) ref [7] */
#define OMGE_QZS OMGE_GPS

#define RE_GPS   6378137.0        /* earth semimajor axis (WGS84) (m) */
#define RE_GLO   6378136.0        /* radius of earth (m)            ref [2] */
#define RE_BDS   RE_GPS           /* earth semimajor axis (WGS84) (m) */
#define RE_GAL   RE_GPS           /* earth semimajor axis (WGS84) (m) */
#define RE_QZS   RE_GPS           /* earth semimajor axis (WGS84) (m) */

#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_BDS   3.986004418E14   /* earth gravitational constant   ref [9] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_QZS   MU_GPS

#define J2_GLO   1.0826257E-3     /* 2nd zonal harmonic of geopot   ref [2] */

#define F_const -4.442807633E-10  /* relativity const */

#define TSTEP    60.0             /* integration step glonass ephemeris (s) */
#define RTOL_KEPLER 1E-14         /* relative tolerance for Kepler equation */
#define MAX_ITER_KEPLER 30        /* max number of iteration of Kelpler */
#define NMAX        10            /* order of polynomial interpolation */
#define MAXDTE      900.0         /* max time difference to ephem time (s) */
#define EXTERR_CLK  1E-3          /* extrapolation error for clock (m/s) */
#define EXTERR_EPH  5E-7          /* extrapolation error for ephem (m/s^2) */

#define SIN_5 -0.0871557427476582 /* sin(-5.0 deg) */
#define COS_5  0.9961946980917456 /* cos(-5.0 deg) */


///< Select GPS ephemeris
static bool SelectGPSEph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATGPS || pStruct->m_GPSEphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_GPS + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_GPSEphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_GPSEphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_GPSEph = pStruct->m_GPSEphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


///< Select GLO ephemeris
static bool SelectGLOEph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATGLO || pStruct->m_GLOEphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_GLO + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_GLOEphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_GLOEphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_GLOEph = pStruct->m_GLOEphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


///< Select BD2 ephemeris
static bool SelectBD2Eph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATBD2 || pStruct->m_BD2EphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_BD2 + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_BD2EphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_BD2EphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_BD2Eph = pStruct->m_BD2EphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


///< Select BD3 ephemeris
static bool SelectBD3Eph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATBD3 || pStruct->m_BD3EphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_BD3 + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_BD3EphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_BD3EphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_BD3Eph = pStruct->m_BD3EphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


///< Select GAL ephemeris
static bool SelectGALEph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATGAL || pStruct->m_GALEphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_GAL + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_GALEphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_GALEphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_GALEph = pStruct->m_GALEphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


///< Select QZS ephemeris
static bool SelectQZSEph(CEphemerisComputer* pStruct)
{
    if (!pStruct)return false;

    int sat = satprn2no(pStruct->m_prn, NULL);
    if (sat <= 0 || sat > NSATQZS || pStruct->m_QZSEphDatas[sat - 1][0].toe.GPSWeek <= 10) return false;

    double dt = 0.0;
    double tmax = MAXDTOE_QZS + 1.0;
    double tmin = tmax + 1.0;
    bool flag = false;

    for (int i = 0; i < MAXEPHNUM; i++)
    {
        if (pStruct->m_QZSEphDatas[sat - 1][i].prn != pStruct->m_prn) continue;
        dt = fabs(MinusGPSTIME(pStruct->m_QZSEphDatas[sat - 1][i].toe, pStruct->m_gt));
        if (dt > tmax) continue;
        if (dt <= tmin)
        {
            tmin = dt;
            pStruct->m_QZSEph = pStruct->m_QZSEphDatas[sat - 1][i];
            flag = true;
        }
    }

    return flag;
}


static double CalculateGPSEphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_GPSEph.toc);

    //循环两次之后的钟差改正值差异很小
    for (int i = 0; i < 2; i++)
    {
        dt -= pStruct->m_GPSEph.f0 + pStruct->m_GPSEph.f1 * dt + pStruct->m_GPSEph.f2 * dt * dt;
    }

    return pStruct->m_GPSEph.f0 + pStruct->m_GPSEph.f1 * dt + pStruct->m_GPSEph.f2 * dt * dt;
}


static double CalculateGLOEphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_GLOEph.toe);

    for (int i = 0; i < 2; i++)
    {
        dt -= -pStruct->m_GLOEph.taun + pStruct->m_GLOEph.gamn * dt;
    }
    return -pStruct->m_GLOEph.taun + pStruct->m_GLOEph.gamn * dt;

}


static double CalculateBD2EphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_BD2Eph.toc);

    //循环两次之后的钟差改正值差异很小
    for (int i = 0; i < 2; i++)
    {
        dt -= pStruct->m_BD2Eph.f0 + pStruct->m_BD2Eph.f1 * dt + pStruct->m_BD2Eph.f2 * dt * dt;
    }

    return pStruct->m_BD2Eph.f0 + pStruct->m_BD2Eph.f1 * dt + pStruct->m_BD2Eph.f2 * dt * dt;
}


static double CalculateBD3EphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_BD3Eph.toc);

    //循环两次之后的钟差改正值差异很小
    for (int i = 0; i < 2; i++)
    {
        dt -= pStruct->m_BD3Eph.f0 + pStruct->m_BD3Eph.f1 * dt + pStruct->m_BD3Eph.f2 * dt * dt;
    }

    return pStruct->m_BD3Eph.f0 + pStruct->m_BD3Eph.f1 * dt + pStruct->m_BD3Eph.f2 * dt * dt;
}


static double CalculateGALEphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_GALEph.toc);

    //循环两次之后的钟差改正值差异很小
    for (int i = 0; i < 2; i++)
    {
        dt -= pStruct->m_GALEph.f0 + pStruct->m_GALEph.f1 * dt + pStruct->m_GALEph.f2 * dt * dt;
    }

    return pStruct->m_GALEph.f0 + pStruct->m_GALEph.f1 * dt + pStruct->m_GALEph.f2 * dt * dt;
}


static double CalculateQZSEphClk(CEphemerisComputer* pStruct)
{
    if (!pStruct)return 0.0;

    double dt = MinusGPSTIME(pStruct->m_gt, pStruct->m_QZSEph.toc);

    //循环两次之后的钟差改正值差异很小
    for (int i = 0; i < 2; i++)
    {
        dt -= pStruct->m_QZSEph.f0 + pStruct->m_QZSEph.f1 * dt + pStruct->m_QZSEph.f2 * dt * dt;
    }

    return pStruct->m_QZSEph.f0 + pStruct->m_QZSEph.f1 * dt + pStruct->m_QZSEph.f2 * dt * dt;
}


static bool CalculateGPSEphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    double tk = 0.0, M = 0.0, E = 0.0, Ek = 0.0, sinE = 0.0, cosE = 0.0, u = 0.0, r = 0.0, i = 0.0, O = 0.0, sin2u = 0.0, cos2u = 0.0, x = 0.0, y = 0.0, sinO = 0.0, cosO = 0.0, cosi = 0.0, mu = 0.0, omge = 0.0;

    if (pStruct->m_GPSEph.A <= 0.0)
    {
        rs[0] = rs[1] = rs[2] = 0.0;
        *dts = 0.0;
        return false;
    }

    tk = MinusGPSTIME(gt, pStruct->m_GPSEph.toe);

    mu = MU_GPS;
    omge = OMGE_GPS;

    M = pStruct->m_GPSEph.M0 + (sqrt(mu / (pStruct->m_GPSEph.A * pStruct->m_GPSEph.A * pStruct->m_GPSEph.A)) + pStruct->m_GPSEph.deln) * tk;

    int it = 0;
    // 使用牛顿迭代法
    for (E = M, sinE = Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && it < MAX_ITER_KEPLER; it++) {
        Ek = E; E -= (E - pStruct->m_GPSEph.e * sin(E) - M) / (1.0 - pStruct->m_GPSEph.e * cos(E));
    }

    if (it > MAX_ITER_KEPLER)
    {
        return false;
    }

    cosE = cos(E); sinE = sin(E);
    u = atan2(sqrt(1.0 - pStruct->m_GPSEph.e * pStruct->m_GPSEph.e) * sinE, cosE - pStruct->m_GPSEph.e) + pStruct->m_GPSEph.omg;
    r = pStruct->m_GPSEph.A * (1.0 - pStruct->m_GPSEph.e * cosE);
    i = pStruct->m_GPSEph.i0 + pStruct->m_GPSEph.idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += pStruct->m_GPSEph.cus * sin2u + pStruct->m_GPSEph.cuc * cos2u;
    r += pStruct->m_GPSEph.crs * sin2u + pStruct->m_GPSEph.crc * cos2u;
    i += pStruct->m_GPSEph.cis * sin2u + pStruct->m_GPSEph.cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    O = pStruct->m_GPSEph.OMG0 + (pStruct->m_GPSEph.OMGd - omge) * tk - omge * pStruct->m_GPSEph.toes;
    sinO = sin(O); cosO = cos(O);
    rs[0] = x * cosO - y * cosi * sinO;
    rs[1] = x * sinO + y * cosi * cosO;
    rs[2] = y * sin(i);

    tk = MinusGPSTIME(gt, pStruct->m_GPSEph.toc);
    *dts = pStruct->m_GPSEph.f0 + pStruct->m_GPSEph.f1 * tk + pStruct->m_GPSEph.f2 * tk * tk;

    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * pStruct->m_GPSEph.A) * pStruct->m_GPSEph.e * sinE / (CLIGHT * CLIGHT);

    pStruct->m_SatVar = pStruct->m_GPSEph.sva * pStruct->m_GPSEph.sva;

    return true;
}


/* glonass orbit differential equations --------------------------------------*/
void deq(const double* x, double* xdot, const double* acc)
{
    double r2 = x[0] * x[0] + x[1] * x[1] + x[2] * x[2];

    double a, b, c, r3 = r2 * sqrt(r2), omg2 = OMGE_GLO * OMGE_GLO;

    if (r2 <= 0.0) {
        xdot[0] = xdot[1] = xdot[2] = xdot[3] = xdot[4] = xdot[5] = 0.0;
        return;
    }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a = 1.5 * J2_GLO * MU_GLO * RE_GLO * RE_GLO / r2 / r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b = 5.0 * x[2] * x[2] / r2;                    /* 5*z^2/r^2 */
    c = -MU_GLO / r3 - a * (1.0 - b);                /* -mu/r^3-a(1-b) */
    xdot[0] = x[3]; xdot[1] = x[4]; xdot[2] = x[5];
    xdot[3] = (c + omg2) * x[0] + 2.0 * OMGE_GLO * x[4] + acc[0];
    xdot[4] = (c + omg2) * x[1] - 2.0 * OMGE_GLO * x[3] + acc[1];
    xdot[5] = (c - 2.0 * a) * x[2] + acc[2];
}


/* glonass position and velocity by numerical integration --------------------*/
static void glorbit(double t, double* x, const double* acc)
{
    double k1[6], k2[6], k3[6], k4[6], w[6];
    int i;

    deq(x, k1, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k1[i] * t / 2.0;
    deq(w, k2, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k2[i] * t / 2.0;
    deq(w, k3, acc); for (i = 0; i < 6; i++) w[i] = x[i] + k3[i] * t;
    deq(w, k4, acc);
    for (i = 0; i < 6; i++) x[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * t / 6.0;
}


static bool CalculateGLOEphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    double dt, tt, x[6];
    int i;

    dt = MinusGPSTIME(gt, pStruct->m_GLOEph.toe);

    *dts = -pStruct->m_GLOEph.taun + pStruct->m_GLOEph.gamn * dt;

    for (i = 0; i < 3; i++)
    {
        x[i] = pStruct->m_GLOEph.pos[i];
        x[i + 3] = pStruct->m_GLOEph.vel[i];
    }

    for (tt = dt < 0.0 ? -TSTEP : TSTEP; fabs(dt) > 1E-9; dt -= tt)
    {
        if (fabs(dt) < TSTEP) tt = dt;

        glorbit(tt, x, pStruct->m_GLOEph.acc);
    }

    for (i = 0; i < 3; i++)
    {
        rs[i] = x[i];
    }

    pStruct->m_SatVar = ERREPH_GLO * ERREPH_GLO;

    return true;
}


static bool CalculateBD2EphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    int BDSType = BDSSatelliteType(pStruct->m_prn);

    double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;

    if (pStruct->m_BD2Eph.A <= 0.0)
    {
        rs[0] = rs[1] = rs[2] = 0.0;
        *dts = 0.0;
        return false;
    }

    // toe存储时已转换成gps时了
    tk = MinusGPSTIME(gt, pStruct->m_BD2Eph.toe);

    mu = MU_BDS;
    omge = OMGE_BDS;

    M = pStruct->m_BD2Eph.M0 + (sqrt(mu / (pStruct->m_BD2Eph.A * pStruct->m_BD2Eph.A * pStruct->m_BD2Eph.A)) + pStruct->m_BD2Eph.deln) * tk;

    int it = 0;
    // 使用牛顿迭代法
    for (E = M, sinE = Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && it < MAX_ITER_KEPLER; it++) {
        Ek = E; E -= (E - pStruct->m_BD2Eph.e * sin(E) - M) / (1.0 - pStruct->m_BD2Eph.e * cos(E));
    }

    if (it > MAX_ITER_KEPLER)
    {
        return false;
    }

    cosE = cos(E); sinE = sin(E);

    u = atan2(sqrt(1.0 - pStruct->m_BD2Eph.e * pStruct->m_BD2Eph.e) * sinE, cosE - pStruct->m_BD2Eph.e) + pStruct->m_BD2Eph.omg;
    r = pStruct->m_BD2Eph.A * (1.0 - pStruct->m_BD2Eph.e * cosE);
    i = pStruct->m_BD2Eph.i0 + pStruct->m_BD2Eph.idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += pStruct->m_BD2Eph.cus * sin2u + pStruct->m_BD2Eph.cuc * cos2u;
    r += pStruct->m_BD2Eph.crs * sin2u + pStruct->m_BD2Eph.crc * cos2u;
    i += pStruct->m_BD2Eph.cis * sin2u + pStruct->m_BD2Eph.cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    // GEO卫星
    if (BDSType == 1)
    {
        O = pStruct->m_BD2Eph.OMG0 + pStruct->m_BD2Eph.OMGd * tk - omge * pStruct->m_BD2Eph.toes;
        sinO = sin(O); cosO = cos(O);
        double xg = x * cosO - y * cosi * sinO;
        double yg = x * sinO + y * cosi * cosO;
        double zg = y * sin(i);
        double sino = sin(omge * tk);
        double coso = cos(omge * tk);
        rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
        rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
        rs[2] = -yg * SIN_5 + zg * COS_5;
    }
    else
    {
        O = pStruct->m_BD2Eph.OMG0 + (pStruct->m_BD2Eph.OMGd - omge) * tk - omge * pStruct->m_BD2Eph.toes;
        sinO = sin(O); cosO = cos(O);
        rs[0] = x * cosO - y * cosi * sinO;
        rs[1] = x * sinO + y * cosi * cosO;
        rs[2] = y * sin(i);
    }

    tk = MinusGPSTIME(gt, pStruct->m_BD2Eph.toc);
    *dts = pStruct->m_BD2Eph.f0 + pStruct->m_BD2Eph.f1 * tk + pStruct->m_BD2Eph.f2 * tk * tk;

    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * pStruct->m_BD2Eph.A) * pStruct->m_BD2Eph.e * sinE / (CLIGHT * CLIGHT);

    pStruct->m_SatVar = pStruct->m_BD2Eph.sva * pStruct->m_BD2Eph.sva;

    return true;
}


static bool CalculateBD3EphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    int BDSType = BDSSatelliteType(pStruct->m_prn);

    double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;

    if (pStruct->m_BD3Eph.A <= 0.0)
    {
        rs[0] = rs[1] = rs[2] = 0.0;
        *dts = 0.0;
        return false;
    }

    // toe存储时已转换成gps时了
    tk = MinusGPSTIME(gt, pStruct->m_BD3Eph.toe);

    mu = MU_BDS;
    omge = OMGE_BDS;

    M = pStruct->m_BD3Eph.M0 + (sqrt(mu / (pStruct->m_BD3Eph.A * pStruct->m_BD3Eph.A * pStruct->m_BD3Eph.A)) + pStruct->m_BD3Eph.deln) * tk;

    int it = 0;
    // 使用牛顿迭代法
    for (E = M, sinE = Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && it < MAX_ITER_KEPLER; it++) {
        Ek = E; E -= (E - pStruct->m_BD3Eph.e * sin(E) - M) / (1.0 - pStruct->m_BD3Eph.e * cos(E));
    }

    if (it > MAX_ITER_KEPLER)
    {
        return false;
    }

    cosE = cos(E); sinE = sin(E);

    u = atan2(sqrt(1.0 - pStruct->m_BD3Eph.e * pStruct->m_BD3Eph.e) * sinE, cosE - pStruct->m_BD3Eph.e) + pStruct->m_BD3Eph.omg;
    r = pStruct->m_BD3Eph.A * (1.0 - pStruct->m_BD3Eph.e * cosE);
    i = pStruct->m_BD3Eph.i0 + pStruct->m_BD3Eph.idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += pStruct->m_BD3Eph.cus * sin2u + pStruct->m_BD3Eph.cuc * cos2u;
    r += pStruct->m_BD3Eph.crs * sin2u + pStruct->m_BD3Eph.crc * cos2u;
    i += pStruct->m_BD3Eph.cis * sin2u + pStruct->m_BD3Eph.cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    // GEO卫星
    if (BDSType == 1)
    {
        O = pStruct->m_BD3Eph.OMG0 + pStruct->m_BD3Eph.OMGd * tk - omge * pStruct->m_BD3Eph.toes;
        sinO = sin(O); cosO = cos(O);
        double xg = x * cosO - y * cosi * sinO;
        double yg = x * sinO + y * cosi * cosO;
        double zg = y * sin(i);
        double sino = sin(omge * tk);
        double coso = cos(omge * tk);
        rs[0] = xg * coso + yg * sino * COS_5 + zg * sino * SIN_5;
        rs[1] = -xg * sino + yg * coso * COS_5 + zg * coso * SIN_5;
        rs[2] = -yg * SIN_5 + zg * COS_5;
    }
    else
    {
        O = pStruct->m_BD3Eph.OMG0 + (pStruct->m_BD3Eph.OMGd - omge) * tk - omge * pStruct->m_BD3Eph.toes;
        sinO = sin(O); cosO = cos(O);
        rs[0] = x * cosO - y * cosi * sinO;
        rs[1] = x * sinO + y * cosi * cosO;
        rs[2] = y * sin(i);
    }

    tk = MinusGPSTIME(gt, pStruct->m_BD3Eph.toc);
    *dts = pStruct->m_BD3Eph.f0 + pStruct->m_BD3Eph.f1 * tk + pStruct->m_BD3Eph.f2 * tk * tk;

    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * pStruct->m_BD3Eph.A) * pStruct->m_BD3Eph.e * sinE / (CLIGHT * CLIGHT);

    pStruct->m_SatVar = pStruct->m_BD3Eph.sva * pStruct->m_BD3Eph.sva;

    return true;
}


static bool CalculateGALEphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;

    if (pStruct->m_GALEph.A <= 0.0)
    {
        rs[0] = rs[1] = rs[2] = 0.0;
        *dts = 0.0;
        return false;
    }

    tk = MinusGPSTIME(gt, pStruct->m_GALEph.toe);

    mu = MU_GAL;
    omge = OMGE_GAL;

    M = pStruct->m_GALEph.M0 + (sqrt(mu / (pStruct->m_GALEph.A * pStruct->m_GALEph.A * pStruct->m_GALEph.A)) + pStruct->m_GALEph.deln) * tk;

    int it = 0;
    // 使用牛顿迭代法
    for (E = M, sinE = Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && it < MAX_ITER_KEPLER; it++) {
        Ek = E; E -= (E - pStruct->m_GALEph.e * sin(E) - M) / (1.0 - pStruct->m_GALEph.e * cos(E));
    }

    if (it > MAX_ITER_KEPLER)
    {
        return false;
    }

    cosE = cos(E); sinE = sin(E);

    u = atan2(sqrt(1.0 - pStruct->m_GALEph.e * pStruct->m_GALEph.e) * sinE, cosE - pStruct->m_GALEph.e) + pStruct->m_GALEph.omg;
    r = pStruct->m_GALEph.A * (1.0 - pStruct->m_GALEph.e * cosE);
    i = pStruct->m_GALEph.i0 + pStruct->m_GALEph.idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += pStruct->m_GALEph.cus * sin2u + pStruct->m_GALEph.cuc * cos2u;
    r += pStruct->m_GALEph.crs * sin2u + pStruct->m_GALEph.crc * cos2u;
    i += pStruct->m_GALEph.cis * sin2u + pStruct->m_GALEph.cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    O = pStruct->m_GALEph.OMG0 + (pStruct->m_GALEph.OMGd - omge) * tk - omge * pStruct->m_GALEph.toes;
    sinO = sin(O); cosO = cos(O);
    rs[0] = x * cosO - y * cosi * sinO;
    rs[1] = x * sinO + y * cosi * cosO;
    rs[2] = y * sin(i);


    tk = MinusGPSTIME(gt, pStruct->m_GALEph.toc);
    *dts = pStruct->m_GALEph.f0 + pStruct->m_GALEph.f1 * tk + pStruct->m_GALEph.f2 * tk * tk;

    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * pStruct->m_GALEph.A) * pStruct->m_GALEph.e * sinE / (CLIGHT * CLIGHT);

    pStruct->m_SatVar = pStruct->m_GALEph.sva * pStruct->m_GALEph.sva;

    return true;
}


static bool CalculateQZSEphPos(CEphemerisComputer* pStruct, GPSTIME gt, double rs[3], double* dts)
{
    if (!pStruct)return false;

    double tk, M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;

    if (pStruct->m_QZSEph.A <= 0.0)
    {
        rs[0] = rs[1] = rs[2] = 0.0;
        *dts = 0.0;
        return false;
    }

    tk = MinusGPSTIME(gt, pStruct->m_QZSEph.toe);

    mu = MU_QZS;
    omge = OMGE_QZS;

    M = pStruct->m_QZSEph.M0 + (sqrt(mu / (pStruct->m_QZSEph.A * pStruct->m_QZSEph.A * pStruct->m_QZSEph.A)) + pStruct->m_QZSEph.deln) * tk;

    int it = 0;
    // 使用牛顿迭代法
    for (E = M, sinE = Ek = 0.0; fabs(E - Ek) > RTOL_KEPLER && it < MAX_ITER_KEPLER; it++) {
        Ek = E; E -= (E - pStruct->m_QZSEph.e * sin(E) - M) / (1.0 - pStruct->m_QZSEph.e * cos(E));
    }

    if (it > MAX_ITER_KEPLER)
    {
        //            string msg("kepler iteration overflow");
        //            DumpException(msg, BUGT_FEW, DUMP_LOCATION);
        return false;
    }

    cosE = cos(E); sinE = sin(E);

    u = atan2(sqrt(1.0 - pStruct->m_QZSEph.e * pStruct->m_QZSEph.e) * sinE, cosE - pStruct->m_QZSEph.e) + pStruct->m_QZSEph.omg;
    r = pStruct->m_QZSEph.A * (1.0 - pStruct->m_QZSEph.e * cosE);
    i = pStruct->m_QZSEph.i0 + pStruct->m_QZSEph.idot * tk;
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += pStruct->m_QZSEph.cus * sin2u + pStruct->m_QZSEph.cuc * cos2u;
    r += pStruct->m_QZSEph.crs * sin2u + pStruct->m_QZSEph.crc * cos2u;
    i += pStruct->m_QZSEph.cis * sin2u + pStruct->m_QZSEph.cic * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    O = pStruct->m_QZSEph.OMG0 + (pStruct->m_QZSEph.OMGd - omge) * tk - omge * pStruct->m_QZSEph.toes;
    sinO = sin(O); cosO = cos(O);
    rs[0] = x * cosO - y * cosi * sinO;
    rs[1] = x * sinO + y * cosi * cosO;
    rs[2] = y * sin(i);


    tk = MinusGPSTIME(gt, pStruct->m_QZSEph.toc);
    *dts = pStruct->m_QZSEph.f0 + pStruct->m_QZSEph.f1 * tk + pStruct->m_QZSEph.f2 * tk * tk;

    /* relativity correction */
    *dts -= 2.0 * sqrt(mu * pStruct->m_QZSEph.A) * pStruct->m_QZSEph.e * sinE / (CLIGHT * CLIGHT);

    pStruct->m_SatVar = pStruct->m_QZSEph.sva * pStruct->m_QZSEph.sva;

    return true;
}


///< Calculate GPS ephemeris
static bool GPSEphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSGPS] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectGPSEph(pStruct))
    {
        pStruct->m_svh = pStruct->m_GPSEph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateGPSEphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateGPSEphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateGPSEphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->pStruct->m_SatVel = (rs2-rs1)/dt;
        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Calculate GLO ephemeris
static bool GLOEphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSGLO] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectGLOEph(pStruct))
    {
        pStruct->m_svh = pStruct->m_GLOEph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateGLOEphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateGLOEphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateGLOEphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->m_SatVel = (rs2-rs1)/dt;

        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Calculate BD2 ephemeris
static bool BD2EphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSBD2] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectBD2Eph(pStruct))
    {
        pStruct->m_svh = pStruct->m_BD2Eph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateBD2EphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateBD2EphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateBD2EphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->m_SatVel = (rs2-rs1)/dt;
        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Calculate BD3 ephemeris
static bool BD3EphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSBD3] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectBD3Eph(pStruct))
    {
        pStruct->m_svh = pStruct->m_BD3Eph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateBD3EphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateBD3EphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateBD3EphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->m_SatVel = (rs2-rs1)/dt;
        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Calculate GAL ephemeris
static bool GALEphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSGAL] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectGALEph(pStruct))
    {
        pStruct->m_svh = pStruct->m_GALEph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateGALEphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateGALEphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateGALEphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->m_SatVel = (rs2-rs1)/dt;
        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Calculate QZS ephemeris
static bool QZSEphComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct || pStruct->m_bNavEph[ISYSQZS] == false)
    {
        return false;
    }

    bool flag = false;

    double dt = 1E-3;

    GPSTIME gt1 = pStruct->m_gt;
    double  rs1[3];
    double  dts1 = 0;

    GPSTIME gt2 = AddGPSTIMESec(gt1, dt);
    double  rs2[3];
    double  dts2 = 0;

    double clk = 0.0;

    if (SelectQZSEph(pStruct))
    {
        pStruct->m_svh = pStruct->m_QZSEph.svh;

        // 先由卫星发射时刻计算钟差
        clk = CalculateQZSEphClk(pStruct);

        // 用钟差进一步改正卫星信号发射时刻，修正到正确的GPS时
        CalculateQZSEphPos(pStruct, MinusGPSTIMESec(gt2, clk), rs2, &dts2);
        CalculateQZSEphPos(pStruct, MinusGPSTIMESec(gt1, clk), rs1, &dts1);

        M31EQU(rs1, pStruct->m_SatPos);
        M31_M31(rs2, rs1, pStruct->m_SatVel); M31Scale(1.0 / dt, pStruct->m_SatVel); // pStruct->m_SatVel = (rs2-rs1)/dt;
        pStruct->m_SatClk = dts1;
        pStruct->m_SatClkVel = (dts2 - dts1) / dt;

        flag = true;
    }

    return flag;
}


///< Constructor function, to be called when defining
void InitCEphemerisComputer(CEphemerisComputer* pStruct)
{
    if (!pStruct)return;

    for (int i = 0; i < NSYS; i++)
        pStruct->m_bNavEph[i] = false;

    if (NSATGPS > 0)
    {
        for (int i = 0; i < NSATGPS; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGPSEPH(&pStruct->m_GPSEphDatas[i][j]);
    }
    if (NSATGLO > 0)
    {
        for (int i = 0; i < NSATGLO; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGLOEPH(&pStruct->m_GLOEphDatas[i][j]);
    }
    if (NSATBD2 > 0)
    {
        for (int i = 0; i < NSATBD2; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGPSEPH(&pStruct->m_BD2EphDatas[i][j]);
    }
    if (NSATBD3 > 0)
    {
        for (int i = 0; i < NSATBD3; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGPSEPH(&pStruct->m_BD3EphDatas[i][j]);
    }
    if (NSATGAL > 0)
    {
        for (int i = 0; i < NSATGAL; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGPSEPH(&pStruct->m_GALEphDatas[i][j]);
    }
    if (NSATQZS > 0)
    {
        for (int i = 0; i < NSATQZS; i++)
            for (int j = 0; j < MAXEPHNUM; j++)
                InitGPSEPH(&pStruct->m_QZSEphDatas[i][j]);
    }

    for (int i = 0; i < 8; i++)
        pStruct->m_GPSION[i] = 0;

    InitGPSTIME(&pStruct->m_gt);
    pStruct->m_prn = 0;
    pStruct->m_bSimulateRTDS = false;
    for (int i = 0; i < 3; i++) pStruct->m_SatPos[i] = pStruct->m_SatVel[i] = 0.0;
    pStruct->m_SatClk = 0.0;
    pStruct->m_SatClkVel = 0.0;
    pStruct->m_SatVar = 0.0;
    pStruct->m_svh = -1;
}


///< Calculate satellite position
bool EphComputer(CEphemerisComputer* pStruct, const GPSTIME gt, const int prn, const EPH_TYPE ephType)
{
    if (!pStruct)return false;

    bool bSuc = false;
    pStruct->m_gt = gt;
    pStruct->m_prn = prn;

    for (int i = 0; i < 3; i++) pStruct->m_SatPos[i] = pStruct->m_SatVel[i] = 0.0;
    pStruct->m_SatClk = 0.0;
    pStruct->m_SatClkVel = 0.0;
    pStruct->m_SatVar = 0.0;
    pStruct->m_svh = -1;

    if (ephType == EPH_BRDC)
    {
        int sys = SYSNON;
        satprn2no(prn, &sys);

        if (sys == SYSGPS)
        {
            bSuc = GPSEphComputer(pStruct);
        }
        else if (sys == SYSGLO)
        {
            bSuc = GLOEphComputer(pStruct);
        }
        else if (sys == SYSBD2)
        {
            bSuc = BD2EphComputer(pStruct);
        }
        else if (sys == SYSBD3)
        {
            bSuc = BD3EphComputer(pStruct);
        }
        else if (sys == SYSGAL)
        {
            bSuc = GALEphComputer(pStruct);
        }
        else if (sys == SYSQZS)
        {
            bSuc = QZSEphComputer(pStruct);
        }
    }
    else
    {
        bSuc = false;
    }

    return bSuc;
}


