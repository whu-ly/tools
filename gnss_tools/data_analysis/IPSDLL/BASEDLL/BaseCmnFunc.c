#include "BaseCmnFunc.h"
#include "BaseSDC.h"


/**
* @brief       ECEF转向ENU系的旋转矩阵
* @param[in]   LLH       double     纬度,经度,大地高
* @param[out]  R_E2L     double     ECEF->ENU的旋转矩阵
* @return
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void Rxyz2enu(const double LLH[3], double R_E2L[9])
{
    double sinp = sin(LLH[0]), cosp = cos(LLH[0]), sinl = sin(LLH[1]), cosl = cos(LLH[1]);
    R_E2L[0] = -sinl;        R_E2L[1] = cosl;         R_E2L[2] = 0.0;
    R_E2L[3] = -sinp * cosl; R_E2L[4] = -sinp * sinl; R_E2L[5] = cosp;
    R_E2L[6] = cosp * cosl;  R_E2L[7] = cosp * sinl;  R_E2L[8] = sinp;
}


/**
* @brief       ENU转向ECEF系的旋转矩阵
* @param[in]   LLH       double     纬度,经度,大地高
* @param[out]  R_L2E     double     ENU->ECEF的旋转矩阵
* @return
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void Renu2xyz(const double LLH[3], double R_L2E[9])
{

    double sinp = sin(LLH[0]), cosp = cos(LLH[0]), sinl = sin(LLH[1]), cosl = cos(LLH[1]);
    R_L2E[0] = -sinl; R_L2E[1] = -sinp * cosl;  R_L2E[2] = cosp * cosl;
    R_L2E[3] = cosl;  R_L2E[4] = -sinp * sinl;  R_L2E[5] = cosp * sinl;
    R_L2E[6] = 0.0;   R_L2E[7] = cosp;          R_L2E[8] = sinp;
}


/**
* @brief       ECEF的向量转到ENU系下
* @param[in]   LLH       double     纬度,经度,大地高
* @param[in]   v_xyz     double     ECEF系下向量
* @param[out]  v_enu     double     ENU系下向量
* @return
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void Vxyz2enu(const double LLH[3], const double v_xyz[3], double v_enu[3])
{
    double R[9] = { 0.0 };
    Rxyz2enu(LLH, R);
    v_enu[0] = R[0] * v_xyz[0] + R[1] * v_xyz[1] + R[2] * v_xyz[2];
    v_enu[1] = R[3] * v_xyz[0] + R[4] * v_xyz[1] + R[5] * v_xyz[2];
    v_enu[2] = R[6] * v_xyz[0] + R[7] * v_xyz[1] + R[8] * v_xyz[2];
}


/**
* @brief       WGS84坐标系: XYZ坐标转LLH坐标
* @param[in]   XYZ          double   XYZ(m) 坐标
* @param[in]   CoorSys      int      坐标系统: 0:WGS84, 1:CGCS2000
* @param[out]  LLH          double   Lat[-90,90],Lon[-180,180],Hgt (deg,m)
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void XYZ2LLH(const double XYZ[3], double LLH[3], int CoorSys)
{
    double a = gs_WGS84_a;
    double e2 = gs_WGS84_e2;

    if (CoorSys == 1)
    {
        a = gs_CGCS2000_a;
        e2 = gs_CGCS2000_e2;
    }

    double X = XYZ[0];
    double Y = XYZ[1];
    double Z = XYZ[2];

    double r2 = X * X + Y * Y;
    double z = 0.0;
    double zk = 0.0;
    double v = a;
    double sinp = 0.0;

    for (z = Z, zk = 0.0; fabs(z - zk) >= 1E-4;)
    {
        zk = z;
        sinp = z / sqrt(r2 + z * z);
        v = a / sqrt(1.0 - e2 * sinp * sinp);
        z = Z + v * e2 * sinp;
    }
    LLH[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (Z > 0.0 ? PI / 2.0 : -PI / 2.0);
    LLH[1] = r2 > 1E-12 ? atan2(Y, X) : 0.0;
    LLH[2] = sqrt(r2 + z * z) - v;
}


/**
* @brief       WGS84坐标系: LLH坐标转XYZ坐标
* @param[in]   LLH          double   Lat[-90,90],Lon[-180,180],Hgt (deg,m)
* @param[in]   CoorSys      int      坐标系统: 0:WGS84, 1:CGCS2000
* @param[out]  XYZ          double   XYZ(m) 坐标
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void LLH2XYZ(const double LLH[3], double XYZ[3], int CoorSys)
{
    double a = gs_WGS84_a;
    double e2 = gs_WGS84_e2;

    if (CoorSys == 1)
    {
        a = gs_CGCS2000_a;
        e2 = gs_CGCS2000_e2;
    }

    double sinp = sin(LLH[0]), cosp = cos(LLH[0]), sinl = sin(LLH[1]), cosl = cos(LLH[1]);
    double v = a / sqrt(1.0 - e2 * sinp * sinp);

    XYZ[0] = (v + LLH[2]) * cosp * cosl;
    XYZ[1] = (v + LLH[2]) * cosp * sinl;
    XYZ[2] = (v * (1.0 - e2) + LLH[2]) * sinp;
}


/**
* @brief       WGS84坐标转换到CGCS2000坐标,坐标对应的是框架历元
* @param[in]   XYZ_WGS84         double  WGS84坐标(m)
* @param[out]  XYZ_CGCS2000      double  CGCS2000坐标(m)
* @return      void
* @note        WGS84对准到ITRF2000框架,CGCS2000对准到ITRF1997框架,无站速坐标框架转换
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void WGS84_CGCS2000(const double XYZ_WGS84[3], double XYZ_CGCS2000[3])
{
    double T[3] = { 0.0 };
    T[0] = 0.0067;
    T[1] = 0.0061;
    T[2] = -0.0185;

    const double D = 1.55 * 1E-9;

    // 采用简化写法
    for (int i = 0; i < 3; i++)
    {
        XYZ_CGCS2000[i] = (1.0 + D) * XYZ_WGS84[i] + T[i];
    }
}


/**
* @brief       CGCS2000坐标转换到WGS84坐标,坐标对应的是框架历元
* @param[in]   XYZ_CGCS2000      double  CGCS2000坐标(m)
* @param[out]  XYZ_WGS84         double  WGS84坐标(m)
* @return      void
* @note        WGS84对准到ITRF2000框架,CGCS2000对准到ITRF1997框架,无站速坐标框架转换
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void CGCS2000_WGS84(const double XYZ_CGCS2000[3], double XYZ_WGS84[3])
{
    double T[3] = { 0.0 };
    T[0] = 0.0067;
    T[1] = 0.0061;
    T[2] = -0.0185;

    double D = 1.55 * 1E-9;

    // 采用简化写法
    for (int i = 0; i < 3; i++)
    {
        XYZ_WGS84[i] = (XYZ_CGCS2000[i] - T[i]) / (1.0 + D);
    }
}


/**
* @brief       读取复制字符串
* @param[in]   src          char     源字符串,如果末位有'\n',自动除去
* @param[in]   nPos         int      读取的初始位置(从0开始)
* @param[in]   nCount       int      读取的字符串数
* @param[out]  dst          char     目标字符串, 末尾已用\0填充
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void xstrmid(const char* src, const int nPos, const int nCount, char* dst)
{
    int		i;
    const char* str;
    char	c;

    str = src + nPos;

    for (i = 0; i < nCount; i++)
    {
        c = *(str + i);
        if (c)
        {
            *(dst + i) = c;
        }
        else
        {
            // 除去末尾的'\n'
            if (dst[i - 1] == '\n') dst[i - 1] = '\0';
            *(dst + i) = '\0';
            break;
        }
    }

    *(dst + nCount) = '\0';
}


/**
* @brief       字符串转换成数字
* @param[in]   src        char     源字符串
* @param[in]   nPos       int      读取的初始位置(从0开始)
* @param[in]   nCount     int      读取的字符串数
* @return      double     转换得到的数字
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double str2num(const char* src, int nPos, int nCount)
{
    double value = 0;
    char str[1024], * p = str;
    if (nPos < 0 || (int)strlen(src) < nPos || (int)sizeof(str) - 1 < nCount) return 0.0;
    for (src += nPos; *src && --nCount >= 0; src++) *p++ = *src == 'd' || *src == 'D' ? 'E' : *src; *p = '\0';
    return sscanf(str, "%lf", &value) == 1 ? value : 0.0;
}