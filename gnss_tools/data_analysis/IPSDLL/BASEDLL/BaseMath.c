#include "BaseMath.h"


/**
* @brief        搜索 vector 最小值的位置
* @param[in]    vec     double[]    input vector
* @param[in]    n       int         vector 维数
* @param[out]   pos     int*        最小值的位置, 从0开始
* @param[in]    bAbs    bool        是否取绝对值
* @return       double  最小值
* @note         无
* @par History:
*               2024/10/06, Ying Liu, new \n
* @internals    无
*/
double getMinPos(const double vec[], int n, int* pos, bool bAbs)
{
    double vel = bAbs ? fabs(vec[0]) : vec[0], tmp = 0.0;
    if (pos)*pos = 0;
    for (int i = 1; i < n; i++)
    {
        tmp = bAbs ? fabs(vec[i]) : vec[i];
        if (tmp < vel)
        {
            vel = tmp;
            if (pos)*pos = i;
        }
    }
    return vel;
}


/**
* @brief        搜索 vector 最大值的位置
* @param[in]    vec     double[]    input vector
* @param[in]    n       int         vector 维数
* @param[out]   pos     int*        最大值的位置, 从0开始
* @param[in]    bAbs    bool        是否取绝对值
* @return       double  最大值
* @note         无
* @par History:
*               2024/10/06, Ying Liu, new \n
* @internals    无
*/
double getMaxPos(const double vec[], int n, int* pos, bool bAbs)
{
    double vel = bAbs ? fabs(vec[0]) : vec[0], tmp = 0.0;
    if (pos)*pos = 0;
    for (int i = 1; i < n; i++)
    {
        tmp = bAbs ? fabs(vec[i]) : vec[i];
        if (tmp > vel)
        {
            vel = tmp;
            if (pos)*pos = i;
        }
    }
    return vel;
}


/**
* @brief       开方运算, 解决输入小于0的时候, sqrt报错
* @param[in]   val         const double   输入值
* @return      double 返回开方值, 若输入小于0,返回0值
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
static unsigned long gs_xsqrt_count = 0;
double xsqrt(const double val)
{
    if (val < 0.0)
    {
        if (gs_xsqrt_count > 1E7) gs_xsqrt_count = 100; // 防止溢出
        gs_xsqrt_count++;
        return 0.0;
    }
    else
    {
        return sqrt(val);
    }
}


/**
* @brief       初始化状态向量
* @param[in]   StateX      Mat     状态向量
* @param[in]   StateP      Mat     状态方差阵
* @param[in]   i           int     第i个初始化
* @param[in]   x           double  状态值
* @param[in]   var         double  状态方差
* @return      void
* @note        StateX[i] = x，StateP[i][i] = var, i行和i列其它元素置0
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void InitStateX(double StateX[], double StateP[], int row, int i, double x, double var)
{
    int n = row;
    StateX[i] = x;

    for (int k = 0; k < n; k++)
    {
        StateP[i * row + k] = 0.0;
        StateP[k * row + i] = 0.0;
    }

    StateP[i * row + i] = var;
}


/**
* @brief       计算序列均值的递推公式
* @param[in]   avg		double*     输入i-1时刻的均值,输出i时刻的均值
* @param[in]   i		int         当前为i时刻,i=1开始
* @param[in]   data		double      i时刻新进入的数据
* @return      void
* @note        先调用var, 再调用avg
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void SequenceAverage(double *avg, int i, double data)
{
    if (avg)*avg += (data - *avg) / (double)i;
}


/**
* @brief       计算序列均值、方差和均方根误差
* @param[in]   data		double*		数据序列
* @param[in]   n		int			数据序列维数
* @param[out]  mean		double*		序列均值
* @param[out]  rms		double*     序列均方根误差
* @param[out]  var		double*     序列方差,默认为NULL
* @param[in]   bessel	bool        方差计算是否用白塞尔公式，默认为false
* @return      void
* @note
*/
void Mean_RMS_Var(const double* data, const int n, double* mean, double* rms, double* var, bool bessel)
{
    double meantmp = 0.0;
    double rmstmp = 0.0;
    double vartmp = 0.0;

    if (!data)return;

    for (int i = 0; i < n; i++)
    {
        meantmp += data[i];
        rmstmp += data[i] * data[i];
    }

    meantmp = meantmp / n;
    rmstmp = xsqrt(rmstmp / n);

    for (int i = 0; i < n; i++)
    {
        vartmp += (data[i] - meantmp) * (data[i] - meantmp);
    }

    if (bessel && n > 1)
    {
        vartmp = vartmp / (n - 1);
    }
    else
    {
        vartmp = vartmp / n;
    }

    if (mean) *mean = meantmp;
    if (rms)  *rms = rmstmp;
    if (var)  *var = vartmp;
}


/**
* @brief       勒让德球谐函数 Legendre Spherical harmonics function
* @param[in]   n       int     度(degree)
* @param[in]   m       int     阶(order)
* @param[in]   x       double  value  x -> [0,1], x = sin(lat)
* @param[out]  P       double* P[n+1][m+1], i>=j 有值
* @return      void
* @note        推荐使用 Pnm(9,9,x,P[0]);
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void Pnm(int n, int m, double x, double* P)
{
    if (n > 9)return;

    int i = 0, j = 0, k = 0, a;
    double ir = 0, sum = 0;

    /* determine n!  (faktorielle)  moved by 1 */
    double dfac[20] = { 0.0 }; // IPS_malloc(2 * n + 2, sizeof(double));
    dfac[0] = 1;
    for (i = 1; i <= 2 * n + 1; i++) dfac[i] = dfac[i - 1] * (i);

    /* determine Legendre functions (Heiskanen and Moritz, Physical Geodesy, 1967, eq. 1-62) */
    for (i = 0; i <= n; i++)
    {
        a = (i < m) ? i : m;
        for (j = 0; j <= a; j++) {
            ir = (int)((i - j) / 2);
            sum = 0;
            for (k = 0; k <= ir; k++) {
                sum += ((int)pow((double)(-1.0), k)) * dfac[2 * i - 2 * k] / dfac[k] / dfac[i - k] / dfac[i - j - 2 * k] * pow(x, i - j - 2 * k);
            }
            /*  Legendre functions moved by 1 */
            P[i * (m + 1) + j] = 1.0 / pow((double)(2.0), i) * sqrt(pow(1 - x * x, j)) * sum;
        }
    }
}


///< round x
int RoundNum(double a)
{
    return (int)floor((a)+0.5);                     
}


void SortArr(double* vec, int n, bool bMin2Max)
{
    if (!vec || n <= 1)return;
    if (bMin2Max)
    {
        for (int i = 0; i < n - 1; ++i) {
            for (int j = 0; j < n - i - 1; ++j) {
                // 如果当前元素大于下一个元素，交换它们
                if (vec[j] > vec[j + 1]) {
                    double temp = vec[j];
                    vec[j] = vec[j + 1];
                    vec[j + 1] = temp;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < n - 1; ++i) {
            for (int j = 0; j < n - i - 1; ++j) {
                // 如果当前元素大于下一个元素，交换它们
                if (vec[j] < vec[j + 1]) {
                    double temp = vec[j];
                    vec[j] = vec[j + 1];
                    vec[j + 1] = temp;
                }
            }
        }
    }
}


bool IsEqual(double a, double b)
{
    return (fabs(a - b) < IPS_EPSILON ? true : false);
}


void SwapF(float *a, float *b)
{
    if (!a || !b)return;

    float c = *a;
    *a = *b; 
    *b = c;
}


void SwapI(int *a, int *b)
{
    if (!a || !b)return;

    int c = *a;
    *a = *b;
    *b = c;
}


double GetFrac(double a)
{
    return (a - (int)a);
}