#include "BaseTime.h"


const static long    JAN61980 = 44244;    // gps time reference
const static long    JAN11901 = 15385;
const static double  SECPERDAY = 86400.0;

const static long    month_day[2][13] = {
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365},
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335, 366}
};

const static double leaps[][7]={ /* leap seconds {y,m,d,h,m,s,utc-gpst,...} */
    {2017,1,1,0,0,0,-18},
    {2015,7,1,0,0,0,-17},
    {2012,7,1,0,0,0,-16},
    {2009,1,1,0,0,0,-15},
    {2006,1,1,0,0,0,-14},
    {1999,1,1,0,0,0,-13},
    {1997,7,1,0,0,0,-12},
    {1996,1,1,0,0,0,-11},
    {1994,7,1,0,0,0,-10},
    {1993,7,1,0,0,0, -9},
    {1992,7,1,0,0,0, -8},
    {1991,1,1,0,0,0, -7},
    {1990,1,1,0,0,0, -6},
    {1988,1,1,0,0,0, -5},
    {1985,7,1,0,0,0, -4},
    {1983,7,1,0,0,0, -3},
    {1982,7,1,0,0,0, -2},
    {1981,7,1,0,0,0, -1}
};


void InitGPSTIME(GPSTIME* ot)
{
    if (!ot)return;
    ot->GPSWeek = -1;
    ot->secsOfWeek = 0;
    ot->fracOfSec = 0.0;
}


void InitGPSTIME2(int week, double sec, GPSTIME* ot)
{
    if (!ot)return;
    ot->GPSWeek = week;
    ot->secsOfWeek = (int)(sec);
    ot->fracOfSec = sec - ot->secsOfWeek;
    if (ot->fracOfSec < IPS_EPSILON)
    {
        ot->fracOfSec = 0.0;
    }
}


void InitMJD(MJD* ot)
{
    if (!ot)return;
    ot->mjd = 0;
    ot->fracOfDay = 0.0;
}


void InitYDOYHMS(YDOYHMS* ot)
{
    if (!ot)return;
    ot->year = ot->yday = ot->hour = ot->min = 0;
    ot->sec = 0.0;
}


void InitYMDHMS(YMDHMS* ot)
{
    if (!ot)return;
    ot->year = 2000;
    ot->month = ot->day = 1;
    ot->hour = ot->min = 0;
    ot->sec = 0.0;
}


///< GPS时减去GPS时, sec=gt1-gt2
double MinusGPSTIME(GPSTIME gt1, GPSTIME gt2)
{
    double lsec = (gt1.GPSWeek - gt2.GPSWeek) * 604800.0 + gt1.secsOfWeek - gt2.secsOfWeek;
    double t = lsec + gt1.fracOfSec - gt2.fracOfSec;
    return t;
}


///< GPS时减去秒数, gt2=gt1-sec
GPSTIME MinusGPSTIMESec(GPSTIME gt1, double sec)
{
    GPSTIME gt2;
    InitGPSTIME(&gt2);

    double t = gt1.secsOfWeek + gt1.fracOfSec - sec;

    long   lt = (int)t;

    double dt = (gt1.secsOfWeek - lt) + gt1.fracOfSec - sec;

    if (t >= 0 && t < 604800)
    {
        gt2.GPSWeek = gt1.GPSWeek;
        gt2.secsOfWeek = lt;
        gt2.fracOfSec = dt;
    }
    else if (t < 0)
    {
        int i = (int)(-lt / 604800) + 1;
        gt2.GPSWeek = gt1.GPSWeek - i;
        if (dt == 0)
        {
            gt2.secsOfWeek = i * 604800 + lt;
            gt2.fracOfSec = dt;
        }
        else
        {
            gt2.secsOfWeek = i * 604800 + lt - 1;
            gt2.fracOfSec = dt + 1;
        }
    }
    else if (t >= 604800)
    {
        int i = (int)(lt / 604800);
        gt2.GPSWeek = gt1.GPSWeek + i;
        gt2.secsOfWeek = lt - i * 604800;
        gt2.fracOfSec = dt;
    }

    if (gt2.fracOfSec < IPS_EPSILON)
    {
        gt2.fracOfSec = 0.0;
    }

    return gt2;
}


///< GPS时加上秒数, gt2=gt1+sec
GPSTIME AddGPSTIMESec(GPSTIME gt1, double sec)
{
    double dtmp = 0 - sec;

    return MinusGPSTIMESec(gt1, dtmp);
}


///< Get seconds of the week
double GetGPSTIMESow(GPSTIME gt)
{
    return (gt.secsOfWeek + gt.fracOfSec);
}


bool IsGPSTIMEEqual(GPSTIME gt1, GPSTIME gt2)
{
    double dt = MinusGPSTIME(gt1, gt2);

    return (dt == 0.0) ? true : false;
}


YMDHMS InitYMDHMS1(const double* ep)
{
    YMDHMS ot;
    ot.year = (int)ep[0];
    ot.month = (int)ep[1];
    ot.day = (int)ep[2];
    ot.hour = (int)ep[3];
    ot.min = (int)ep[4];
    ot.sec = ep[5];
    return ot;
}


/*
* @brief       GPS时间转换为MJD
* @param[in]   t           GPSTIME  GPS时
* @return      MJD         约化儒略日
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
MJD GPST2MJD(GPSTIME t)
{
    double sec_of_week = t.secsOfWeek + t.fracOfSec;
    long   mjd = (long)(t.GPSWeek * 7 + sec_of_week / SECPERDAY + JAN61980);
    double fmjd = fmod(sec_of_week, SECPERDAY) / SECPERDAY;

    MJD ot;
    ot.mjd = mjd;
    ot.fracOfDay = fmjd;

    return ot;
}


/*
* @brief       GPS时间转换为YDOYHMS
* @param[in]   t           GPSTIME  GPS时
* @return      YDOYHMS     年/年积日/HMS
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
YDOYHMS GPST2YDHMS(GPSTIME t)
{
    int    gps_week = t.GPSWeek;
    double sec_of_week = t.secsOfWeek + t.fracOfSec;
    long   mjd = (long)(gps_week * 7 + sec_of_week / SECPERDAY + JAN61980);
    double fmjd = fmod(sec_of_week, SECPERDAY) / SECPERDAY;
    long   days_fr_jan1_1901 = mjd - JAN11901;
    long   num_four_yrs = days_fr_jan1_1901 / 1461;
    long   years_so_far = 1901 + 4 * num_four_yrs;
    long   days_left = days_fr_jan1_1901 - 1461 * num_four_yrs;
    long   delta_yrs = days_left / 365 - days_left / 1460;
    int    year = years_so_far + delta_yrs;
    int    yday = days_left - 365*delta_yrs + 1;
    int    hour = (int)(fmjd*24.0);
    int    minute = (int)(fmjd*1440.0 - hour * 60.0);
    double second = fmjd * 86400.0 - hour * 3600.0 - minute * 60.0;

    YDOYHMS ot;
    ot.year = year;
    ot.yday = yday;
    ot.hour = hour;
    ot.min  = minute;
    ot.sec  = second;

    return ot;
}


/*
* @brief       GPS时间转换为YMDHMS
* @param[in]   t           GPSTIME  GPS时
* @return      YMDHMS      年/月/日/HMS
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
YMDHMS  GPST2YMDHMS(GPSTIME t)
{
    int    gps_week = t.GPSWeek;
    double sec_of_week0 = t.secsOfWeek + t.fracOfSec;
    double sec_of_week = (sec_of_week0 * 10 - (int)(sec_of_week0 * 10)) > 0.5 ? (int)(sec_of_week0 * 10 + 1) : (int)(sec_of_week0 * 10);//10HZ
    sec_of_week /= 10.0;//10HZ
    long   mjd = (long)(gps_week * 7 + sec_of_week / SECPERDAY + JAN61980);
    long   days_fr_jan1_1901 = mjd - JAN11901;
    long   num_four_yrs = days_fr_jan1_1901 / 1461;
    long   years_so_far = 1901 + 4 * num_four_yrs;
    long   days_left = days_fr_jan1_1901 - 1461 * num_four_yrs;
    long   delta_yrs = days_left / 365 - days_left / 1460;
    long   year = years_so_far + delta_yrs;
    long   yday = days_left - 365 * delta_yrs + 1;
    double fmjd = fmod(sec_of_week, SECPERDAY);
    int    hour = (int)(fmjd / 3600.0);
    int    minute = (int)(fmjd / 60.0 - hour * 60.0);
    double second = fmod(sec_of_week, SECPERDAY) - hour * 3600.0 - minute * 60.;
    long   leap = (year % 4 == 0);
    long   guess = (long)(yday*0.032);
    long   more = ((yday - month_day[leap][guess + 1]) > 0);
    int    month = guess + more + 1;
    int    mday = yday - month_day[leap][guess + more];

    YMDHMS ot;
    ot.year = year;
    ot.month = month;
    ot.day = mday;
    ot.hour = hour;
    ot.min = minute;
    ot.sec = second;

    return ot;
}


/*
* @brief       MJD转换为GPS时间
* @param[in]   t           MJD      约化儒略日
* @return      GPSTIME     GPS时
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME MJD2GPST(MJD t)
{
    long   mjd = t.mjd;
    double fmjd = t.fracOfDay;
    int    gps_week = (mjd - JAN61980) / 7;
    double sec_of_week = ((mjd - JAN61980) - gps_week * 7 + fmjd)*SECPERDAY;

    GPSTIME ot;
    InitGPSTIME2(gps_week, sec_of_week, &ot);
    return ot;
}


/*
* @brief       YDOYHMS转换为GPS时间
* @param[in]   t           YDOYHMS  年/年积日/HMS
* @return      GPSTIME     GPS时
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME YDHMS2GPST(YDOYHMS t)
{
    int year = t.year;
    int yday = t.yday;
    int hour = t.hour;
    int minute = t.min;
    double second = t.sec;

    long   mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 + JAN11901;
    int    gps_week = (mjd - JAN61980) / 7;
    double fmjd = second + minute * 60 + hour * 3600;
    double sec_of_week = ((mjd - JAN61980) - gps_week * 7)*SECPERDAY + fmjd;

    GPSTIME ot;
    InitGPSTIME2(gps_week, sec_of_week, &ot);
    return ot;
}


/*
* @brief       YMDHMS转换为GPS时间
* @param[in]   t           YMDHMS  年/月/日/HMS
* @return      GPSTIME     GPS时
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME YMDHMS2GPST(YMDHMS t)
{
    int year = t.year;
    int month = t.month;
    int mday = t.day;
    int hour = t.hour;
    int minute = t.min;
    double second = t.sec;

    long   leap = (year % 4 == 0);
    long   yday = month_day[leap][month - 1] + mday;
    long   mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 + JAN11901;
    int    gps_week = (mjd - JAN61980) / 7;
    double fmjd = second + minute * 60 + hour * 3600;
    double sec_of_week = ((mjd - JAN61980) - gps_week * 7)*SECPERDAY + fmjd;

    GPSTIME ot;
    InitGPSTIME2(gps_week, sec_of_week, &ot);
    return ot;
}


/*
* @brief       utc 时间转换到GPS时
* @param[in]   utc         GPSTIME  utc时间
* @return      GPSTIME     GPS时间
* @note        ignore slight time offset under 100 ns
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME UTC2GPST(GPSTIME utc)
{
    double dt = 0.0;
    for (int i = 0; i < sizeof(leaps) / sizeof(*leaps); i++)
    {
        dt = MinusGPSTIME(utc, YMDHMS2GPST(InitYMDHMS1(leaps[i])));
        if (dt >= 0.0) return MinusGPSTIMESec(utc, leaps[i][6]);
    }
    return utc;
}


/*
* @brief       GPS 时间转换到UTC时
* @param[in]   gt          GPSTIME  GPS时间
* @return      GPSTIME     utc时间
* @note        ignore slight time offset under 100 ns
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME GPST2UTC(GPSTIME gt)
{
    GPSTIME utc;
    InitGPSTIME(&utc);
    const int n = sizeof(leaps) / sizeof(*leaps);
    double dt = 0.0;
    for (int i = 0; i < n; i++)
    {
        utc = AddGPSTIMESec(gt, leaps[i][6]);
        dt = MinusGPSTIME(utc, YMDHMS2GPST(InitYMDHMS1(leaps[i])));
        if (dt >= 0.0)return utc;
    }

    return gt;
}


/*
* @brief       GPS时转BDS时
* @param[in]   gt          GPSTIME  GPS时间
* @param[in]   bSecCorr    bool     true:只进行14s改正, false:BDS周改正
* @return      GPSTIME     BDS时间
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME GPST2BDST(GPSTIME gpst, bool bSecCorr)
{
    if (bSecCorr == false) gpst.GPSWeek -= 1356;
    return MinusGPSTIMESec(gpst, 14.0);
}


/*
* @brief       BDS时转GPS时
* @param[in]   gt          GPSTIME  BDS时间
* @param[in]   bSecCorr    bool     true:只进行14s改正, false:BDS周改正
* @return      GPSTIME     GPS时间
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME BDST2GPST(GPSTIME bdst, bool bSecCorr)
{
    if(bSecCorr == false) bdst.GPSWeek += 1356;
    return AddGPSTIMESec(bdst, 14);
}


/*
* @brief       string 转时间
* @param[in]   s           char     字符串 string ("... yyyy mm dd hh mm ss ...")
* @param[in]   iPos        int      起始位置
* @param[in]   nCount      int      长度
* @return      GPSTIME     时间
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
GPSTIME str2time(const char* s, int iPos, int nCount)
{
    GPSTIME gt;
    InitGPSTIME(&gt);

    double ep[6] = { 0.0 };
    char str[256], * p = str;

    if (iPos < 0 || (int)strlen(s) < iPos || sizeof(str) - 1 < iPos) return gt;

    for (s += iPos; *s && --nCount >= 0;) *p++ = *s++; *p = '\0';

    if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6) return gt;

    if (ep[0] < 100.0) ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
    
    gt = YMDHMS2GPST(InitYMDHMS1(ep));
    if (gt.GPSWeek > 9999) gt.GPSWeek = -1;
    return gt;
}