/**
* @file
* @brief    Implementation of matrix
* @details  This file is for performing matrix algebra calculations in C programs \n
*           Note that all memory allocation is static, so it is important to pay attention to the memory dimension of MAXOBSLIM
* @author   Feng Zhu. Email: fzhu@whu.edu.cn
* @date     2018/01/17
* @version  2.1
* @par      Copyright(c) 2012-2020 School of Geodesy and Geomatics, University of Wuhan. All Rights Reserved.
*           POSMind Software Kernel, named IPS (Inertial Plus Sensing), this file is a part of IPS.
* @par      History:
*           2018/01/17,Feng Zhu, new \n
*           2024/01/22,Ying Liu, new \n
*/


#ifndef BASETK_BASEMATRIX_H
#define BASETK_BASEMATRIX_H

#include "BaseSDC.h"


#define MATRIX_EPSILON (2.2204460492503131e-016)	///< == DBL_EPSILON


extern void SetValsD(double* dst, double val, int n);


extern void SetValsI(int* dst, int val, int n);


extern void SetValsB(bool* dst, bool val, int n);


extern void SetValsC(char* dst, char val, int n);


/** @brief  Assigns matrix : M2 = M */
extern void M31EQU(const double M[3], double M2[3]);


/** @brief  Assigns matrix : M2 = -M */
extern void M31EQU_1(const double M[3], double M2[3]);


/** @brief  Assigns matrix : M2 = M */
extern void M41EQU(const double M[4], double M2[4]);


/** @brief Assigns matrix : M2 = M */
extern void M33EQU(const double M[9], double M2[9]);


/** @brief Assigns Vector : M2 = M */
extern void VecEQU1(const int n, const double V[], double V2[]);


/** @brief Assigns Vector : M2 = M */
extern void VecEQU2(const int n, const int V[], int V2[]);


/** @brief Assigns Vector : M2 = M */
extern void VecEQU(const int n, const bool V[], bool V2[]);


/** @brief Assigns matrix : M2 = M */
extern void M31Zero(double M[3]);


/** @brief Assigns matrix : M2 = M */
extern void M33Zero(double M[9]);


/** @brief  Scale matrix : M2 = a*M */
extern void M31Scale(double a, double M[3]);


/** @brief  Scale matrix : M2 = a*M */
extern void M33Scale(double a, double M[9]);


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
//extern void M31M31(const double M1[3], const double M2[3], double M3[3], double s1 = 1.0, double s2 = 1.0);
extern void M31M311(const double M1[3], const double M2[3], double M3[3], double s1, double s2);


/** @brief  Adds two matrices : M2 = s1*M1 + s2*M2 */
//extern void M31M31(const double M1[3], double M2[3], double s1 = 1.0, double s2 = 1.0);
extern void M31M312(const double M1[3], double M2[3], double s1, double s2);


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
//extern void M33M33(const double M1[9], const double M2[9], double M3[9], double s1 = 1.0, double s2 = 1.0);
extern void M33M331(const double M1[9], const double M2[9], double M3[9], double s1, double s2);


/** @brief  Adds two matrices : M3 = s1*M1 + s2*M2 */
//extern void M33M33(const double M1[9], double M2[9], double s1 = 1.0, double s2 = 1.0);
extern void M33M332(const double M1[9], double M2[9], double s1, double s2);


/** @brief  Subtracts two matrices : M3 = M1 - M2 */
extern void M31_M31(const double M1[3], const double M2[3], double M3[3]);


/** @brief  Multiplys two matrices : M31_ = M33 * M31 */
extern void M33XM31(const double M33[9], const double M31[3], double M31_[3]);


/** @brief  Multiplys two matrices : M33_3 = M33_1 * M33_2 */
extern void M33XM33(const double M33_1[9], const double M33_2[9], double M33_3[9]);


/** @brief  Multiplys two matrices : M33_2 = M33_1 * M33_2 */
extern void M33XM33_R(const double M33_1[9], double M33_2[9]);


/** @brief  Multiplys two matrices : M33_2 = M33_2 * M33_1 */
extern void M33XM33_L(const double M33_1[9], double M33_2[9]);


/** @brief  Cross two vectors : v3 = v1 X v2 */
extern void CrossM3(const double v1[3], const double v2[3], double v3[3]);


/** @brief  dot product of two vectors */
extern double MatrixDot1(int n, const double v1[], const double v2[]);


/*
* @brief       Get Skew-Symmetric Matrix from vector
* @param[in]   v           double   Vector
* @param[out]  M           double   Skew-Symmetric Matrix, 3 x 3
* @return      extern void
* @note
*/
extern void MatrixSkewSymmetric(const double v[3], double M[9]);


/*
* @brief       Get vector from Skew-Symmetric Matrix
* @param[in]   M           double   Skew-Symmetric Matrix, 3 x 3
* @param[out]  v           double   Vector
* @return      void
* @note
*/
extern void MatrixSkewSymmetric_T(const double M[9], double v[3]);


/** @brief     Matrix Symmetrization */
extern void MatrixSymmetrization(int r, int c, double M[]);


/**
* @brief       Copy Matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   src         double   Src matrix
* @param[out]  dst         double   Dest matrix
* @return      void
* @note
*/
extern void MatrixCopy(int r, int c, const double* src, double* dst);


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
*/
extern void MatrixCopySub(const int s1[2], const int s2[2], int nRow, int nCol, int r1, int c1, const double* M1, int r2, int c2, double* M2, bool bMinus);


/**
* @brief       Swaps two matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   src         double   Src matrix
* @param[out]  dst         double   Dest matrix
* @return      void
* @note
*/
extern void MatrixSwap(int r, int c, double* src, double* dst);


/**
* @brief       Sets all elements of a matrix equal to zero
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      void
* @note        its main diagonal elements will be equal to 1.0 and the rest zero
*/
extern void MatrixZero(int r, int c, double M[]);
extern void MatrixZero_diag(int r, int c, double M[]);


/**
* @brief       Sets the matrix as a unit matrix
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      void
* @note
*/
extern void MatrixUnit(int r, int c, double M[]);


/**
* @brief       Matrix Trace
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      double      Returns the sum of diagonal elements of a matrix
* @note
*/
extern double MatrixTrace(int r, int c, const double M[]);


/**
* @brief       Matrix Norm2
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   Matrix
* @return      double      Returns the two norm of a matrix
* @note
*/
extern double MatrixNorm2(int r, int c, const double M[]);


/**
* @brief       Matrix Dot
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M1          double   Matrix
* @param[in]   M2          double   Matrix
* @return      double      Returns the scalar product of two matrice
* @note
*/
double MatrixDot2(int r, int c, const double M1[], const double M2[]);


/**
* @brief       Matrix is Symmetric?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: symmetric, false: non-symmetric
* @note        Here M must be a square matrix, i.e. r == c
*/
extern bool MatrixIsSymmetric(int r, int c, const double M[]);


/**
* @brief       Matrix is SkewSymmetric?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: SkewSymmetric, false: non-SkewSymmetric
* @note        Here M must be a square matrix, i.e. r == c
*/
extern bool MatrixIsSkewSymmetric(int r, int c, const double M[]);


/**
* @brief       Matrix is Singular?
* @param[in]   r           int      The number of rows in the matrix M
* @param[in]   c           int      The number of cols in the matrix M
* @param[in]   M           double   M is self multiplied n times
* @return      bool        true: Singular, false: non-Singular
* @note        Here M must be a square matrix, i.e. r == c
*/
extern bool MatrixIsSingular(int r, int c, const double M[]);


/**
* @brief       Adds two matrix : M3 = s1*M1 + s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
*/
//extern void MatrixAddition1(int r, int c, const double M1[], const double M2[], double M3[], double s1 = 1.0, double s2 = 1.0);
extern void MatrixAddition1(int r, int c, const double M1[], const double M2[], double M3[], double s1, double s2);


/**
* @brief       Adds two matrices : M2 = s1*M1 + s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 or M2
* @param[in]   c            int      The number of cols in the matrix M1 or M2
* @param[in]   M1           double   Src matrix
* @param[out]  M2           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
*/
//extern void MatrixAddition2(int r, int c, const double M1[], double M2[], double s1 = 1.0, double s2 = 1.0);
extern void MatrixAddition2(int r, int c, const double M1[], double M2[], double s1, double s2);


/**
* @brief       Subtracts two matrices : M3 = s1*M1 - s2*M2
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        These matrices must have the same dimensions
*/
//extern void MatrixSubtraction1(int r, int c, const double M1[], const double M2[], double M3[], double s1 = 1.0, double s2 = 1.0);
extern void MatrixSubtraction1(int r, int c, const double M1[], const double M2[], double M3[], double s1, double s2);
	

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
*/
//extern void MatrixSubtraction2(int r, int c, bool bleft, const double M1[], double M2[], double s1 = 1.0, double s2 = 1.0);
extern void MatrixSubtraction2(int r, int c, bool bleft, const double M1[], double M2[], double s1, double s2);

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
*/
//extern void MatrixMultiply(int r1, int c1, const double M1[], int r2, int c2, const double M2[], double M3[], double scale = 1.0);
extern void MatrixMultiply(int r1, int c1, const double M1[], int r2, int c2, const double M2[], double M3[], double scale);


/**
* @brief       convolution of two Matrices
* @param[in]   r1,r2,r3     int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        r3 = r1 + r2
*/
extern void MatrixConv(int r1, const double M1[], int r2, const double M2[], int r3, double M3[]);


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
*/
//extern void MatrixMultiplyEx(const char* tr, int r, int c, int m, const double M1[], const double M2[], double M3[], double a = 1.0, double b = 0.0);
extern void MatrixMultiplyEx(const char *tr, int r, int c, int m, const double M1[], const double M2[], double M3[], double a, double b);


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
*/
extern void MatrixMultiply_BTPB(int r, int c, const double B[], const double* P, bool bPDiagonal, double BTPB[]);


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
*/
extern void MatrixMultiply_BTPL(int r, int c, const double B[], const double* P, const double L[], bool bPDiagonal, double BTPL[]);


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
*/
extern void MatrixMultiply_HPHT(int r, int c, const double H[], const double* P, bool bPDiagonal, double HPHT[]);


/**
* @brief       Scale Multiplys two matrices : M3[i][j] = M1[i][j] * M2[i][j]
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[in]   M1           double   Src matrix
* @param[in]   M2           double   Src matrix
* @param[out]  M3           double   Dest matrix
* @return      void
* @note        The size of M1 and M2 must be same
*/
extern void MatrixScaleMultiply(double scale, int r1, int c1, double M3[]);
	

/**
* @brief       Transposes matrix
* @param[in]   r            int      The number of rows in the matrix M1 or MT
* @param[in]   c            int      The number of cols in the matrix M1 or MT
* @param[in]   M1           double   Src matrix
* @param[out]  MT           double   Transpose of matrix M1
* @return      void
* @note
*/
extern void MatrixTranspose1(int r, int c, const double M1[], double MT[]);


/**
* @brief       Transpose of matrix
* @param[in]   r            int      The number of rows in the matrix M
* @param[in]   c            int      The number of cols in the matrix M
* @param[out]  M            double   Transpose of matrix M
* @return      void
* @note        The original matrix M is replaced by the Transpose of matrix
*/
extern void MatrixTranspose2(int r, int c, double M[]);


/**
* @brief       Inverse of matrix
* @param[in]   r            int      The number of rows in the matrix M or M_Inv
* @param[in]   c            int      The number of cols in the matrix M or M_Inv
* @param[in]   M            double   Src matrix
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note
*/
extern bool MatrixInv1(int r, int c, double M[], double M_Inv[]);


/**
* @brief       Inverse of matrix
* @param[in]   r            int      The number of rows in the matrix M
* @param[in]   c            int      The number of cols in the matrix M
* @param[in]   M            double   Src matrix
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note        The original matrix M is replaced by the Inverse of matrix
*/
extern bool MatrixInv2(int r, int c, double M[]);


/**
* @brief       Inverse of the real symmetric positive definite matrix
* @param[in]   r            int      The number of rows in the matrix M1 , M2 or M3
* @param[in]   c            int      The number of cols in the matrix M1 , M2 or M3
* @param[out]  M            double   Inverse of matrix M
* @return      bool         success or failure
* @note        The real symmetric positive definite matrix has the form: M = ATPA. \n
*              This function for inverse of SP matrix is efficient.
*/
extern bool MatrixInvSP(int r, int c, double M[]);


// 仅对向量或者方阵操作
extern void Equ(int* index, int nindex, const double* src, double* dst, int col);


/**
* @brief       状态向量与状态方差复制
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void EQU_StateXP(const double* X_Src, const double* P_Src, double* X_Dst, double* P_Dst, int row);


/**
* @brief       状态向量/方差复制
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
extern void EQU_StateP(const double* P_Src, double* P_Dst, int row);


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
extern void MatrixDelRC(int r, int c, int dr, int dc, double M[]);


#endif
