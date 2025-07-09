#include "BaseMatrix.h"


void SetValsD(double* dst, double val, int n)
{
    if (!dst)return;
    for (int i = 0; i < n; i++) dst[i] = val;
}


void SetValsI(int* dst, int val, int n)
{
    if (!dst)return;
    for (int i = 0; i < n; i++) dst[i] = val;
}


void SetValsB(bool* dst, bool val, int n)
{
    if (!dst)return;
    for (int i = 0; i < n; i++) dst[i] = val;
}


void SetValsC(char* dst, char val, int n)
{
    if (!dst)return;
    for (int i = 0; i < n; i++) dst[i] = val;
}


/** @brief  Assigns matrix : M2 = M */
void M31EQU(const double M[3], double M2[3])
{
    for (int i = 0; i < 3; i++) M2[i] = M[i];
}


/** @brief  Assigns matrix : M2 = -M */
void M31EQU_1(const double M[3], double M2[3])
{
    for (int i = 0; i < 3; i++) M2[i] = -M[i];
}


/** @brief  Assigns matrix : M2 = M */
void M41EQU(const double M[4], double M2[4])
{
    for (int i = 0; i < 4; i++) M2[i] = M[i];
}


/** @brief Assigns matrix : M2 = M */
void M33EQU(const double M[9], double M2[9])
{
    for (int i = 0; i < 9; i++) M2[i] = M[i];
}


/** @brief Assigns Vector : M2 = M */
void VecEQU1(const int n, const double V[], double V2[])
{
    for (int i = 0; i < n; i++) V2[i] = V[i];
}


/** @brief Assigns Vector : M2 = M */
void VecEQU2(const int n, const int V[], int V2[])
{
    for (int i = 0; i < n; i++) V2[i] = V[i];
}


/** @brief Assigns Vector : M2 = M */
void VecEQU(const int n, const bool V[], bool V2[])
{
    for (int i = 0; i < n; i++) V2[i] = V[i];
}


/** @brief Assigns matrix : M2 = M */
void M31Zero(double M[3])
{
    for (int i = 0; i < 3; i++) M[i] = 0.0;
}


/** @brief Assigns matrix : M2 = M */
void M33Zero(double M[9])
{
    for (int i = 0; i < 9; i++) M[i] = 0.0;
}


/** @brief  Scale matrix : M2 = a*M */
void M31Scale(double a, double M[3])
{
    for (int i = 0; i < 3; i++) M[i] *= a;
}


/** @brief  Scale matrix : M2 = a*M */
void M33Scale(double a, double M[9])
{
    for (int i = 0; i < 9; i++) M[i] *= a;
}


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
void M31M311(const double M1[3], const double M2[3], double M3[3], double s1, double s2)
{
    for (int i = 0; i < 3; i++) M3[i] = s1 * M1[i] + s2 * M2[i];
}


/** @brief  Adds two matrices : M2 = s1*M1 + s2*M2 */
void M31M312(const double M1[3], double M2[3], double s1, double s2)
{
    for (int i = 0; i < 3; i++) M2[i] = s1 * M1[i] + s2 * M2[i];
}


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
void M33M331(const double M1[9], const double M2[9], double M3[9], double s1, double s2)
{
    for (int i = 0; i < 9; i++) M3[i] = s1 * M1[i] + s2 * M2[i];
}


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
void M33M332(const double M1[9], double M2[9], double s1, double s2)
{
    for (int i = 0; i < 9; i++) M2[i] = s1 * M1[i] + s2 * M2[i];
}


/** @brief  Subtracts two matrices : M3 = M1 - M2 */
void M31_M31(const double M1[3], const double M2[3], double M3[3])
{
    for (int i = 0; i < 3; i++) M3[i] = M1[i] - M2[i];
}


/** @brief  Multiplys two matrices : M31_ = M33 * M31 */
void M33XM31(const double M33[9], const double M31[3], double M31_[3])
{
    M31_[0] = M33[0] * M31[0] + M33[1] * M31[1] + M33[2] * M31[2];
    M31_[1] = M33[3] * M31[0] + M33[4] * M31[1] + M33[5] * M31[2];
    M31_[2] = M33[6] * M31[0] + M33[7] * M31[1] + M33[8] * M31[2];
}


/** @brief  Multiplys two matrices : M33_3 = M33_1 * M33_2 */
void M33XM33(const double M33_1[9], const double M33_2[9], double M33_3[9])
{
    int i, j, k;
    double Sum;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
        Sum = 0.0; for (k = 0; k < 3; k++) Sum = Sum + M33_1[i * 3 + k] * M33_2[k * 3 + j]; M33_3[i * 3 + j] = Sum;
    }
}


/** @brief  Multiplys two matrices : M33_2 = M33_1 * M33_2 */
void M33XM33_R(const double M33_1[9], double M33_2[9])
{
    int i;
    double M33_3[9] = { 0.0 };
    M33XM33(M33_1, M33_2, M33_3);
    for (i = 0; i < 9; i++) M33_2[i] = M33_3[i];
}


/** @brief  Multiplys two matrices : M33_2 = M33_2 * M33_1 */
void M33XM33_L(const double M33_1[9], double M33_2[9])
{
    int i;
    double M33_3[9] = { 0.0 };
    M33XM33(M33_2, M33_1, M33_3);
    for (i = 0; i < 9; i++) M33_2[i] = M33_3[i];
}


/** @brief  Cross two vectors : v3 = v1 X v2 */
void CrossM3(const double v1[3], const double v2[3], double v3[3])
{
    v3[0] = v1[1] * v2[2] - v1[2] * v2[1];
    v3[1] = v1[2] * v2[0] - v1[0] * v2[2];
    v3[2] = v1[0] * v2[1] - v1[1] * v2[0];
}


/** @brief  dot product of two vectors */
double MatrixDot1(int n, const double v1[], const double v2[])
{
    double c = 0.0;
    while (--n >= 0) c += v1[n] * v2[n];
    return c;
}


/*
* @brief       Get Skew-Symmetric Matrix from vector
* @param[in]   v           double   Vector
* @param[out]  M           double   Skew-Symmetric Matrix, 3 x 3
* @return      void
* @note
*/
void MatrixSkewSymmetric(const double v[3], double M[9])
{
    M[0] = 0.0;  M[1] = -v[2]; M[2] = v[1];
    M[3] = v[2]; M[4] = 0.0;  M[5] = -v[0];
    M[6] = -v[1]; M[7] = v[0]; M[8] = 0.0;
}


/*
* @brief       Get vector from Skew-Symmetric Matrix
* @param[in]   M           double   Skew-Symmetric Matrix, 3 x 3
* @param[out]  v           double   Vector
* @return      void
* @note
*/
void MatrixSkewSymmetric_T(const double M[9], double v[3])
{
    v[0] = M[7]; v[1] = M[2]; v[2] = M[3];
}


/** @brief     Matrix Symmetrization */
void MatrixSymmetrization(int r, int c, double M[])
{
    int i, j;
    for (i = 0; i < r; i++) for (j = 0; j < i; j++) M[i * c + j] = M[j * r + i] = 0.5 * (M[i * c + j] + M[j * r + i]);
}


/**
* @brief       Copy Matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   src         double   Src matrix
* @param[out]  dst         double   Dest matrix
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixCopy(int r, int c, const double* src, double* dst)
{
    int n = r*c, i;

    for (i = 0; i < n; i++) dst[i] = src[i];
}


/**
* @brief       Copy a sub-matrix of the matrix
* @param[in]   s1           int      (s1[0],s2[0]) is the starting point of M1
* @param[in]   s2           int      (s1[1],s2[1]) is the starting point of M2
* @param[in]   n            int      (n[0],n[1]) is the number of rows and cols that is copied
* @param[in]   r            int      The number of rows in the matrix M1 or M2
* @param[in]   c            int      The number of cols in the matrix M1 or M2
* @param[in]   M1           double   Src matrix
* @param[out]  M2           double   Dest matrix
* @return      void
* @note        The new matrix is given by M2[(s2[0] + i)*c2 + s2[1] + j] = M1[(s1[0] + i)*c1 + s1[1] + j];
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixCopySub(const int s1[2], const int s2[2], int nRow, int nCol, int r1, int c1, const double* M1, int r2, int c2, double* M2, bool bMinus)
{
    if (s1[0] + nRow > r1) return;
    if (s1[1] + nCol > c1) return;
    if (s2[0] + nRow > r2) return;
    if (s2[1] + nCol > c2) return;

    int i, j;
		
    if (bMinus)
    {
        for (i = 0; i < nRow; i++)
        {
            for (j = 0; j < nCol; j++)
            {
                M2[(s2[0] + i)*c2 + s2[1] + j] = -M1[(s1[0] + i)*c1 + s1[1] + j];
            }
        }
    }
    else
    {
        for (i = 0; i < nRow; i++)
        {
            for (j = 0; j < nCol; j++)
            {
                M2[(s2[0] + i)*c2 + s2[1] + j] = M1[(s1[0] + i)*c1 + s1[1] + j];
            }
        }
    }
		
}


/**
* @brief       Swaps two matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   src         double   Src matrix
* @param[out]  dst         double   Dest matrix
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixSwap(int r, int c, double* src, double* dst)
{
    int n = r*c, i;
    double tmp;

    for (i = 0; i < n; i++)
    {
        tmp = dst[i];
        dst[i] = src[i];
        src[i] = tmp;
    }
}


/**
* @brief       Sets all elements of a matrix equal to zero
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      void
* @note        its main diagonal elements will be equal to 1.0 and the rest zero
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixZero(int r, int c, double M[])
{
    if (M && r > 0 && c > 0)
    {
        memset(M, 0, sizeof(double)*r*c);
    }
}

void MatrixZero_diag(int r, int c, double M[])
{
    for (int i = 0; i < r; i++)
    {
        M[(r + 1)*i] = 0.0;
    }
}


/**
* @brief       Sets the matrix as a unit matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixUnit(int r, int c, double M[])
{
    if (M && r > 0 && c > 0 && r == c)
    {
        memset(M, 0, sizeof(double)*r*c);
        for (int i = 0; i < r; i++) M[i*r + i] = 1.0;
    }
}


/**
* @brief       Matrix Trace
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      double      Returns the sum of diagonal elements of a matrix
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double MatrixTrace(int r, int c, const double M[])
{
    int i;
    double b = 0;
    for (i = 0; i < r; i++) b += M[i*r + i];
    return b;
}


/**
* @brief       Matrix Norm2
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      double      Returns the two norm of a matrix
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double MatrixNorm2(int r, int c, const double M[])
{
    int i;
    double val = 0;

    int n = (r == 1 ? c : (c == 1 ? r : 0));

    for (i = 0; i < n; i++)
    {
        val += M[i] * M[i];
    }

    return sqrt(val);
}

	
/**
* @brief       Matrix Dot
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M1          double   Matrix
* @param[in]   M2          double   Matrix
* @return      double      Returns the scalar product of two matrice
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
double MatrixDot2(int r, int c, const double M1[], const double M2[])
{
    int i;
    double val = 0;

    int n = (r == 1 ? c : (c == 1 ? r : 0));

    for (i = 0; i < n; i++)
    {
        val += M1[i] * M2[i];
    }

    return val;
}


/**
* @brief       Matrix is Symmetric?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: symmetric, false: non-symmetric
* @note        Here M must be a square matrix, i.e. r == c
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixIsSymmetric(int r, int c, const double M[])
{
    if (r != c) return false;
    int i, j;
    for (i = 0; i < r; i++)
    for (j = 0; j < c; j++)
    {
        if (fabs(M[i*c + j] - M[j*r + i]) > 1e-8)
        {
            //z = fabs(M[i*c + j] - M[j*r + i]);
            return false;
        }
    }
    return true;
}


/**
* @brief       Matrix is SkewSymmetric?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: SkewSymmetric, false: non-SkewSymmetric
* @note        Here M must be a square matrix, i.e. r == c
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixIsSkewSymmetric(int r, int c, const double M[])
{
    if (r != c) return false;
    int i, j;
    for (i = 0; i < r; i++)
    for (j = 0; j < c; j++)
    {
        if (fabs(M[i*c + j] + M[j*r + i]) > MATRIX_EPSILON) return false;
    }
    return true;
}


/**
* @brief       Matrix is Singular?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: Singular, false: non-Singular
* @note        Here M must be a square matrix, i.e. r == c
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixIsSingular(int r, int c, const double M[])
{
    return true;
}


/**
* @brief       Adds two matrix : M3 = s1*M1 + s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixAddition1(int r, int c, const double M1[], const double M2[], double M3[], double s1, double s2)
{
    int i, j;

    for (i = 0; i<r; i++)
    {
        for (j = 0; j<c; j++)
        {
            M3[i*c + j] = s1*M1[i*c + j] + s2*M2[i*c + j];
        }
    }
}


/**
* @brief       Adds two matrices : M2 = s1*M1 + s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 or M2
* @param[in]   c            int      The number of cols in the matrix M1 or M2
* @param[in]   M1           double   Src matrix
* @param[out]  M2           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixAddition2(int r, int c, const double M1[], double M2[], double s1, double s2)
{
    int i, j;

    for (i = 0; i<r; i++)
    {
        for (j = 0; j<c; j++)
        {
            M2[i*c + j] = s1*M1[i*c + j] + s2*M2[i*c + j];
        }
    }
}

	
/**
* @brief       Subtracts two matrices : M3 = s1*M1 - s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixSubtraction1(int r, int c, const double M1[], const double M2[], double M3[], double s1, double s2)
{
    int i, j;

    for (i = 0; i<r; i++)
    {
        for (j = 0; j<c; j++)
        {
            M3[i*c + j] = s1*M1[i*c + j] - s2*M2[i*c + j];
        }
    }
}

	
/**
* @brief       Subtracts two matrix : left:M2 = M1 - M2, right: M2 = s2*M2 - s1*M1
* @param[in]   r            int      The number of rows in the matrix M1 or M2
* @param[in]   c            int      The number of cols in the matrix M1 or M2
* @param[in]   bleft        bool     left minus or right minus
* @param[in]   M1           double   Src matrix
* @param[out]  M2           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
*              left:M2 = M1 - M2, right: M2 = M2 - M1
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixSubtraction2(int r, int c, bool bleft, const double M1[], double M2[], double s1, double s2)
{
    int i, j;

    if (bleft)
    {
        for (i = 0; i<r; i++)
        {
            for (j = 0; j<c; j++)
            {
                M2[i*c + j] = s1*M1[i*c + j] - s2*M2[i*c + j];
            }
        }
    }
    else
    {
        for (i = 0; i<r; i++)
        {
            for (j = 0; j<c; j++)
            {
                M2[i*c + j] = s2*M2[i*c + j] - s1*M1[i*c + j];
            }
        }
    }
}

	
/**
* @brief       Multiplys two matrices : M3 = scale* (M1 * M2)
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[in]   scale        double   M3 = scale*M3
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        The number of M1 cols and M2 rows must be same
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixMultiply(int r1, int c1, const double M1[], int r2, int c2, const double M2[], double M3[], double scale)
{
    int i, j, k;
    double Sum;

    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            Sum = 0.0;

            for (k = 0; k < c1; k++)
            {
                Sum = Sum + *(M1 + i*c1 + k) * *(M2 + k*c2 + j);
            }

            *(M3 + i*c2 + j) = Sum*scale;
        }
    }
}

	
/**
* @brief       convolution of two Matrices
* @param[in]   r1,r2,r3     int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        r3 = r1 + r2
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixConv(int r1, const double M1[], int r2, const double M2[], int r3, double M3[])
{
    const double* signal;
    const double* filter;

    int sigLength = 0, filLength = 0;

    if (r1 > r2)
    {
        sigLength = r1;
        filLength = r2;
        signal = M1;
        filter = M2;
    }
    else
    {
        sigLength = r2;
        filLength = r1;
        signal = M2;
        filter = M1;
    }

    int length = sigLength + filLength - 1;

    for (int i = 1; i <= length; ++i)
    {
        M3[i] = 0;
        if (i < filLength)
            for (int j = 1; j <= i; ++j)
                M3[i] += filter[j] * signal[i - j + 1];
        else if (i <= sigLength)
            for (int j = 1; j <= filLength; ++j)
                M3[i] += filter[j] * signal[i - j + 1];
        else
            for (int j = i - sigLength + 1; j <= filLength; ++j)
                M3[i] += filter[j] * signal[i - j + 1];
    }
}


/**
* @brief       Multiplys two matrices : M3 = a*M1*M2 + b*M3
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        The number of M1 cols and M2 rows must be same
*              NN: M1(r,m) * M2(m,c) = M3(r,c)
*              NT: M1(r,m) * M2(c,m) = M3(r,c)
*              TN: M1(m,r) * M2(m,c) = M3(r,c)
*              TT: M1(m,r) * M2(c,m) = M3(r,c)
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixMultiplyEx(const char *tr, int r, int c, int m, const double M1[], const double M2[], double M3[], double a, double b)
{
    double d;
    int i, j, k;
    int f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < r; i++) for (j = 0; j < c; j++) {
        d = 0.0;
        switch (f) {
        case 1: for (k = 0; k < m; k++) d += M1[i*m + k] * M2[k*c + j]; break;
        case 2: for (k = 0; k < m; k++) d += M1[i*m + k] * M2[j*m + k]; break;
        case 3: for (k = 0; k < m; k++) d += M1[k*r + i] * M2[k*c + j]; break;
        case 4: for (k = 0; k < m; k++) d += M1[k*r + i] * M2[j*m + k]; break;
        }
        if (b == 0.0) M3[i*c + j] = a*d; else M3[i*c + j] = a*d + b*M3[i*c + j];
    }
}


/**
* @brief       Multiplys three matrices : BTPB = BT * P * B, P==NULL:BTPB = BT * B
* @param[in]   r            int      The number of rows
* @param[in]   c            int      The number of cols
* @param[in]   B            double   Src matrix, r x c
* @param[in]   P            double   Src matrix, r x r, if there is no P, P == NULL!!
* @param[in]   bPDiagonal   bool     The P is diagonal?
* @param[out]  BTPB         double   Dest matrix, = BT*P*B, c x c
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixMultiply_BTPB(int r, int c, const double B[], const double* P, bool bPDiagonal, double BTPB[])
{
    int i, j, k;
    double Sum;

    if (!P)
    {
        for (i = 0; i < c; i++)
        {
            for (j = 0; j < c; j++)
            {
                Sum = 0.0;

                for (k = 0; k < r; k++)
                {
                    Sum += B[k*c + i] * B[k*c + j];
                }

                BTPB[i*c + j] = Sum;
            }
        }
    }
    else if (bPDiagonal)
    {
        for (i = 0; i < c; i++)
        {
            for (j = 0; j < c; j++)
            {
                Sum = 0.0;

                for (k = 0; k < r; k++)
                {
                    Sum += B[k*c + i] * B[k*c + j] * P[k*r + k];
                }

                BTPB[i*c + j] = Sum;
            }
        }
    }
    else
    {
        //double* vec = (double*)IPS_malloc(r, sizeof(double));
        double vec[MAXOBSLIM] = { 0.0 };

        for (i = 0; i < c; i++)
        {
            for (j = 0; j < r; j++)
            {
                Sum = 0.0;

                for (k = 0; k < r; k++)
                {
                    Sum = Sum + B[i + k * c] * P[k*r + j];
                }

                vec[j] = Sum;
            }

            for (j = 0; j < c; j++)
            {
                Sum = 0.0;

                for (k = 0; k < r; k++)
                {
                    Sum = Sum + vec[k] * B[j + k * c];
                }

                BTPB[i*c + j] = Sum;
            }
        }
    }
}


/**
* @brief       Multiplys two matrices : BTPL = BT * P * L, P==NULL:BTPL = BT * L
* @param[in]   r            int      The number of rows
* @param[in]   c            int      The number of cols
* @param[in]   B            double   Src matrix, r x c
* @param[in]   P            double   Src matrix, r x r, if there is no P, P == NULL!!
* @param[in]   L            double   Src matrix, r x 1
* @param[out]  BTPL         double   Dest matrix, = BT*P*L, c x 1
* @return      void
* @note        The number of B rows and L rows must be same, \n
*              the size of BT*P*L is c x 1
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixMultiply_BTPL(int r, int c, const double B[], const double* P, const double L[], bool bPDiagonal, double BTPL[])
{
    int i, k, j;
    double Sum;

    if (!P)
    {
        for (i = 0; i < c; i++)
        {
            Sum = 0;

            for (k = 0; k < r; k++)
            {
                Sum += B[k*c + i] * L[k];
            }

            BTPL[i] = Sum;
        }
    }
    else if (bPDiagonal)
    {
        for (i = 0; i < c; i++)
        {
            Sum = 0;

            for (k = 0; k < r; k++)
            {
                Sum += B[k*c + i] * L[k] * P[k*r + k];
            }

            BTPL[i] = Sum;
        }
    }
    else
    {
        //double* vec = (double*)IPS_malloc(r, sizeof(double));
        double vec[MAXOBSLIM] = { 0.0 };

        for (i = 0; i < c; i++)
        {
            for (j = 0; j < r; j++)
            {
                Sum = 0.0;

                for (k = 0; k < r; k++)
                {
                    Sum = Sum + B[i + k*c] * P[k*r + j];
                }

                vec[j] = Sum;
            }

            Sum = 0.0;

            for (k = 0; k < r; k++)
            {
                Sum = Sum + vec[k] * L[k];
            }

            BTPL[i] = Sum;
        }
    }
}


/**
* @brief       Multiplys three matrices : HPHT = H * P * HT, P==NULL:HPHT = H * HT
* @param[in]   r            int      The number of rows
* @param[in]   c            int      The number of cols
* @param[in]   H            double   Src matrix, r x c
* @param[in]   P            double   Src matrix, c x c, if there is no P, P == NULL!!
* @param[in]   bPDiagonal   bool     The P is diagonal?
* @param[out]  HPHT         double   Dest matrix, = H*P*HT r x r
* @return      void, bool            true: HPHT > R, false HPHT <= R
* @note
*              1. HPHTR  = H * P * HT + R
*              2. R_HPHT = R - H * P * HT
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixMultiply_HPHT(int r, int c, const double H[], const double* P, bool bPDiagonal, double HPHT[])
{
    int i, j, k;
    double Sum;

    if (!P)
    {
        for (i = 0; i < r; i++)
        {
            for (j = 0; j < r; j++)
            {
                Sum = 0.0;

                for (k = 0; k < c; k++)
                {
                    Sum += H[i*c + k] * H[j*c + k];
                }

                HPHT[i*r + j] = Sum;
            }
        }
    }
    else if (bPDiagonal || (!P))
    {
        for (i = 0; i < r; i++)
        {
            for (j = 0; j < r; j++)
            {
                Sum = 0.0;

                for (k = 0; k < c; k++)
                {
                    Sum += H[i*c + k] * H[j*c + k] * P[k*c + k];
                }

                HPHT[i*r + j] = Sum;
            }
        }
    }
    else
    {
        //double* vec = (double*)IPS_malloc(c, sizeof(double));
        double vec[MAXRTKNX] = { 0.0 };

        for (i = 0; i < r; i++)
        {
            for (j = 0; j < c; j++)
            {
                Sum = 0.0;

                for (k = 0; k < c; k++)
                {
                    Sum = Sum + H[i*c + k] * P[k*c + j];
                }

                vec[j] = Sum;
            }

            for (j = 0; j < r; j++)
            {
                Sum = 0.0;

                for (k = 0; k < c; k++)
                {
                    Sum = Sum + vec[k] * H[j*c + k];
                }

                HPHT[i*r + j] = Sum;
            }
        }
    }
}
	

/**
* @brief       Scale Multiplys two matrices : M3[i][j] = M1[i][j] * M2[i][j]
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        The size of M1 and M2 must be same
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixScaleMultiply(double scale, int r1, int c1, double M[])
{
    int i, j;

    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c1; j++)
        {
            M[i*c1 + j] *= scale;
        }
    }
}


/**
* @brief       Transposes matrix
* @param[in]   r            int      The number of rows in the matrix M1 or MT
* @param[in]   c            int      The number of cols in the matrix M1 or MT
* @param[in]   M1           double   Src matrix
* @param[out]  MT           double   Transpose of matrix M1
* @return      void
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixTranspose1(int r, int c, const double M[], double MT[])
{
    int i, j;

    for (i = 0; i<r; i++)
    {
        for (j = 0; j<c; j++)
        {
            MT[j*r + i] = M[i*c + j];
        }
    }
}


/**
* @brief       Transpose of matrix
* @param[in]   r            int      The number of rows in the matrix M
* @param[in]   c            int      The number of cols in the matrix M
* @param[out]  M            double   Transpose of matrix M
* @return      void
* @note        The original matrix M is replaced by the Transpose of matrix
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void MatrixTranspose2(int r, int c, double M[])
{
    //double* MT = (double*)IPS_malloc(r * c, sizeof(double));
    double MT[MAXOBSLIM * MAXOBSLIM];
    MatrixTranspose1(r, c, M, MT);
    MatrixCopy(r, c, MT, M);
}


/**
* @brief       Inverse of matrix
* @param[in]   r            int      The number of rows in the matrix M or M_Inv
* @param[in]   c            int      The number of cols in the matrix M or M_Inv
* @param[in]   M            double   Src matrix
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixInv1(int r, int c, double M[], double M_Inv[])
{
    //int* is = (int*)IPS_malloc(r, sizeof(int));
    //int* js = (int*)IPS_malloc(r, sizeof(int));
    int is[MAXOBSLIM] = {0};
    int js[MAXOBSLIM] = {0};
    int i, j, k, l, u, v;
    double d, p;

BEGIN:
    for (i = 0; i < r; i++)
    {
        for (j = 0; j < r; j++)
        {
            M_Inv[i*r + j] = M[i*r + j];
        }
    }

    for (k = 0; k < r; k++)
    {
        d = 0.0;
        for (i = k; i < r; i++)
        {
            for (j = k; j<r; j++)
            {
                l = r*i + j;
                p = fabs(M_Inv[l]);
                if (p>d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < MATRIX_EPSILON)
        {
            if (d < 1e-30) {
                return false; }

            for (i = 0; i < r; i++)   M[i*(r + 1)] += 1E-15; // M矩阵接近奇异时,主对角线加上一个小量
            goto BEGIN;
        }

        if (is[k] != k)
        {
            for (j = 0; j < r; j++)
            {
                u = k*r + j;
                v = is[k] * r + j;
                p = M_Inv[u];
                M_Inv[u] = M_Inv[v];
                M_Inv[v] = p;
            }
        }

        if (js[k] != k)
        {
            for (i = 0; i < r; i++)
            {
                u = i*r + k;
                v = i*r + js[k];
                p = M_Inv[u];
                M_Inv[u] = M_Inv[v];
                M_Inv[v] = p;
            }
        }

        l = k*r + k;
        M_Inv[l] = 1.0 / M_Inv[l];
        for (j = 0; j < r; j++)
        {
            if (j != k)
            {
                u = k*r + j;
                M_Inv[u] = M_Inv[u] * M_Inv[l];
            }
        }
        for (i = 0; i < r; i++)
        {
            if (i != k)
            {
                for (j = 0; j < r; j++)
                {
                    if (j != k)
                    {
                        u = i*r + j;
                        M_Inv[u] = M_Inv[u] - M_Inv[i*r + k] * M_Inv[k*r + j];
                    }
                }
            }
        }
        for (i = 0; i < r; i++)
        {
            if (i != k)
            {
                u = i*r + k;
                M_Inv[u] = -M_Inv[u] * M_Inv[l];
            }
        }
    }

    for (k = r - 1; k >= 0; k--)
    {
        if (js[k] != k)
        {
            for (j = 0; j < r; j++)
            {
                u = k*r + j;
                v = js[k] * r + j;
                p = M_Inv[u];
                M_Inv[u] = M_Inv[v];
                M_Inv[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < r; i++)
            {
                u = i*r + k;
                v = is[k] + i*r;
                p = M_Inv[u];
                M_Inv[u] = M_Inv[v];
                M_Inv[v] = p;
            }
        }
    }

    return true;
}


/**
* @brief       Inverse of matrix
* @param[in]   r            int      The number of rows in the matrix M
* @param[in]   c            int      The number of cols in the matrix M
* @param[in]   M            double   Src matrix
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note        The original matrix M is replaced by the Inverse of matrix
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixInv2(int r, int c, double M[])
{
    int i;
    double b[MAXOBSLIM * MAXOBSLIM];
    bool flag = MatrixInv1(r, c, M, b);

    r = r * r;
    for (i = 0; i < r; i++) M[i] = b[i];

    return flag;
}


/**
* @brief       Inverse of the real symmetric positive definite matrix
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note        The real symmetric positive definite matrix has the form: M = ATPA. \n
*              This function for inverse of SP matrix is efficient.
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
bool MatrixInvSP(int r, int c, double M[])
{
    int j, k;

    for (k = 0; k < r; k++)
    {
        if (M[k*r + k] <= 0.0) return false;
        if (M[k*r + k] < MATRIX_EPSILON) M[k*r + k] += 1e-15; // M矩阵接近奇异时,主对角线加上一个小量

        for (int i = 0; i < r; i++)
        {
            if (i != k)   M[i*r+k] = -M[i*r+k] / M[k*r+k];
        }
        M[k*r+k] = 1.0 / M[k*r+k];

        for (int i = 0; i < r; i++)
        {
            if (i != k)
            {
                for (j = 0; j < r; j++)
                {
                    if (j != k)  M[i*r+j] += M[k*r+j] * M[i*r+k];
                }
            }
        }
        for (j = 0; j < r; j++)
        {
            if (j != k)  M[k*r+j] *= M[k*r+k];
        }
    }

    return true;
}


// 仅对向量或者方阵操作
void Equ(int* index, int nindex, const double* src, double* dst, int col)
{
    if (!index || !src || !dst || nindex < 1) return;

    int i, j;
    if (col == 1)
    {
        for (i = 0; i < nindex; i++) 
            dst[index[i]] = src[index[i]];
    }
    else
    {
        for (i = 0; i < nindex; i++) for (j = 0; j < nindex; j++)
            dst[index[i] * col + index[j]] = src[index[i] * col + index[j]];
    }
}



/**
* @brief       状态向量与状态方差复制
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void EQU_StateXP(const double *X_Src, const double *P_Src, double *X_Dst, double *P_Dst, int row)
{
    if (!X_Src || !P_Src || !X_Dst || !P_Dst)return;

    int StateIndex[MAXRTKNX], nStateIndex = 0;
    for (int i = 0; i < row; i++)
    {
        if (P_Src[i * row + i] > 0.0) StateIndex[nStateIndex++] = i;
    }
    Equ(StateIndex, nStateIndex, X_Src, X_Dst, row);
    Equ(StateIndex, nStateIndex, P_Src, P_Dst, row);
}


/**
* @brief       状态方差复制
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
void EQU_StateP(const double* P_Src, double* P_Dst, int row)
{
    if (!P_Src || !P_Dst)return;

    int StateIndex[MAXRTKNX], nStateIndex = 0;
    for (int i = 0; i < row; i++)
    {
        if (P_Src[i * row + i] > 0.0) StateIndex[nStateIndex++] = i;
    }
    Equ(StateIndex, nStateIndex, P_Src, P_Dst, row);
}


/**
* @brief            删除向量或者矩阵的第dr行和第dc列
* @param[in]        r           int      The number of rows in the matrix M
* @param[in]        c           int      The number of cols in the matrix M
* @param[in]        dr          int      Delete row number, starting from 1
* @param[in]        dc          int      Delete col number, starting from 1
* @param[in/out]    M           double   Data
* @note
* @internals
*/
void MatrixDelRC(int r, int c, int dr, int dc, double M[])
{
    if (dr > r || dc > c || dr <= 0 || dc <= 0) return;

    int i, j;
    double MCopy[MAXRTKNX * MAXRTKNX];
    MatrixCopy(r, c, M, MCopy);
    MatrixZero(r, c, M);

    if (r == 1)
    {
        for (i = 0; i < dc - 1; i++)        M[i] = MCopy[i];
        for (i = dc - 1; i < c - 1; i++)    M[i] = MCopy[i + 1];
    }
    else if (c == 1)
    {
        for (i = 0; i < dr - 1; i++)        M[i] = MCopy[i];
        for (i = dr - 1; i < r - 1; i++)    M[i] = MCopy[i + 1];
    }
    else
    {
        for (i = 0; i < dr - 1; i++)
        {
            for (j = 0; j < dc - 1; j++)    M[i * c + j] = MCopy[i * c + j];
        }
        for (i = 0; i < dr - 1; i++)
        {
            for (j = dc - 1; j < c - 1; j++)M[i * c + j] = MCopy[i * c + j + 1];
        }
        for (i = dr - 1; i < r - 1; i++)
        {
            for (j = 0; j < dc - 1; j++)    M[i * c + j] = MCopy[(i + 1) * c + j];
        }
        for (i = dr - 1; i < r - 1; i++)
        {
            for (j = dc - 1; j < c - 1; j++)M[i * c + j] = MCopy[(i + 1) * c + j + 1];
        }
    }
}