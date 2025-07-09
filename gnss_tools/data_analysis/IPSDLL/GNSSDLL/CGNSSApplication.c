#include <time.h>
#include <stdlib.h>
#include "CGNSSApplication.h"
#include "BaseTime.h"
#include "BaseCmnFunc.h"
#include "BaseMatrix.h"


/*
* @brief       计时器(Timer)
* @return      double      秒长(ms)
* @note
* @par History:
*              2018/01/17,Feng Zhu, new \n
* @internals
*/
static clock_t ls_start = 0;
static clock_t ls_end = 0;
void StartTimer()
{
    ls_start = clock();
}

double EndTimer()
{
    ls_end = clock();
    return ((double)(ls_end - ls_start) / CLOCKS_PER_SEC);
}


// 为计算定位误差做准备
static bool GetRoverInfo(FILE* fp, GPSTIME* gt, double* BLH, int* status, int* satNum, bool* bInit)// 得到待评估序列的数据
{
	static YMDHMS time;
	char oneline[1024] = { '\0' };
	int i = 0;
	bool flag = false;
	char delimiter[2] = ",";

	if (!gt || !BLH)return false;

	BLH[0] = BLH[1] = BLH[2] = 0.0;
	if (status)*status = 0;
	if (satNum)*satNum = 0;

	// 读取时间，主要是获取周
	if (*bInit)
	{
		while (!feof(fp) && *bInit)
		{
			memset(oneline, '\0', 1024);
			fgets(oneline, 1024, fp);
			if (strncmp(oneline, "$ACCUR", 6) == 0)
			{
				char* p = oneline;

				for (i = 0, p = strtok(p, delimiter); p; p = strtok(NULL, delimiter), i++)
				{
					if (i == 4)
					{
						int ymd = atoi(p);
						time.hour = 1; // 赋一个值，防止闰秒
						time.day = ymd % 100;
						time.month = (ymd % 10000) / 100;
						time.year = ymd / 10000;
						if (time.year > 2000 && time.year < 2100)*bInit = false;
					}
				}
			}
		}
	}
	if (*bInit)return false;

	while (!feof(fp))
	{
		memset(oneline, '\0', 1024);
		fgets(oneline, 1024, fp);
		if (strncmp(oneline, "$GNGGA", 6) == 0)
		{
			char* p = oneline;
			YMDHMS time1 = time;

			for (i = 0, p = strtok(p, delimiter); p; p = strtok(NULL, delimiter), i++)
			{
				if (i == 1) // 时间
				{
					sscanf(p, "%2d%2d%lf", &time1.hour, &time1.min, &time1.sec);
					*gt = UTC2GPST(YMDHMS2GPST(time1));
					// 这里后续需要加容错，以及切周
				}
				else if (i == 2) // 纬度
				{
					double B = atof(p);
					int deg = (int)(B * 0.01);
					double min = B - deg * 100;
					BLH[0] = (deg + min / 60.0) * PI / 180.0;
				}
				else if (i == 4) // 经度
				{
					double L = atof(p);
					int deg = (int)(L * 0.01);
					double min = L - deg * 100;
					BLH[1] = (deg + min / 60.0) * PI / 180.0;
				}
				else if (i == 6) // 解标识
				{
					if (status)*status = atoi(p);
				}
				else if (i == 7) // 解标识
				{
					if (satNum)*satNum = atoi(p);
				}
				else if (i == 9) // 高程
				{
					BLH[2] = atof(p);
				}
				else if (i == 11) // 高程异常
				{
					BLH[2] += atof(p);
				}
			}
			if (gt->GPSWeek > 1000 && gt->GPSWeek < 5000 && BLH[0] != 0.0 && BLH[1] != 0.0 && BLH[2] != 0.0) {
				flag = true;
				break;
			}
		}

	}

	return flag;
}


static bool GetReferInfo(FILE* fp, GPSTIME* gt, double* XYZ) // 得到参考真值的数据
{
	static bool bInit = true;
	char oneline[1024] = { '\0' };
	int i = 0;
	bool flag = false;
	char delimiter[2] = " ";

	if (!gt || !XYZ)return false;

	XYZ[0] = XYZ[1] = XYZ[2] = 0.0;

	if (bInit)
	{
		while (!feof(fp) && bInit)
		{
			memset(oneline, '\0', 1024);
			fgets(oneline, 1024, fp);
			if (strncmp(oneline, "week", 4) == 0)bInit = false;
		}
	}
	if (bInit)return false;

	while (!feof(fp))
	{
		memset(oneline, '\0', 1024);
		fgets(oneline, 1024, fp);

		char* p = oneline;

		for (i = 0, p = strtok(p, delimiter); p; p = strtok(NULL, delimiter), i++)
		{
			if (i == 0) // GPS周
			{
				gt->GPSWeek = atoi(p);
				// 这里后续需要加容错，以及切周
			}
			else if (i == 1) // GPS周内秒
			{
				double secs = atof(p);
				gt->secsOfWeek = (int)secs;
				gt->fracOfSec = secs - gt->secsOfWeek;
			}
			else if (i == 16) // X坐标
			{
				XYZ[0] = atof(p);
			}
			else if (i == 17) // Y坐标
			{
				XYZ[1] = atof(p);
			}
			else if (i == 18) // Z坐标
			{
				XYZ[2] = atof(p);
			}
		}
		if (gt->GPSWeek > 1000 && gt->GPSWeek < 5000 && XYZ[0] != 0.0 && XYZ[1] != 0.0 && XYZ[2] != 0.0) {
			flag = true;
			break;
		}
	}

	return flag;
}


/**
* @brief		按照默认配置比较真值文件(HW_TRJ格式)和结果文件(NMEA格式)
* @param[in]	refPoss		double		真值坐标（静态数据需赋值）
* @param[in]	truFile		char[]		真值文件名（动态数据需赋值）
* @param[in]	errFile		char[]		结果文件名
* @param[in]	outFile		char[]		输出文件名
* return		void
* @note         refPoss或者truFile均无效，则取固定解平均值作为真值
* @internals
*/
void GetPosErrs(double *refPoss, char truFile[], char errFile[], char outFile[])
{
	GPSTIME errTime, truTime;
	double errBLH[3] = { 0.0 }, errXYZ[3] = { 0.0 }, truBLH[3] = { 0.0 }, truXYZ[3] = { 0.0 }, enu[3] = { 0.0 }, dXYZ[3];
	int errStatus = 0, errNSat = 0, refMode = 2; // 0 = errFile文件固定解均值作为真值，1 = refPoss提供坐标作为真值，2 = truFile提供坐标序列作为真值
	bool bErr = true, bTru = true, referType = 0, bInit[2] = { true,true }; // 0=ref,1=nmea; 0=refer,1=rover

	// 打开文件
	FILE* trufp = fopen(truFile, "r");
	if (!trufp)
	{
		if (refPoss && refPoss[0] != 0.0)refMode = 1;
		else refMode = 0;
	}

	FILE* errfp = fopen(errFile, "r");
	if (!errfp)
	{
		printf("open errFile fail !");
		return;
	}

	FILE* outfp = fopen(outFile, "w");
	if (!outfp)
	{
		printf("open outfp fail !");
		return;
	}

	// 获取真值
	if (refMode == 0)
	{
		int fixedNum = 0;
		while (!feof(errfp))
		{
			if (!GetRoverInfo(errfp, &errTime, errBLH, &errStatus, &errNSat, bInit + 1))continue;
			LLH2XYZ(errBLH, errXYZ, 0);
			if (errStatus == 4) // 固定解
			{
				truXYZ[0] = truXYZ[0] + (errXYZ[0] - truXYZ[0]) / (fixedNum + 1);
				truXYZ[1] = truXYZ[1] + (errXYZ[1] - truXYZ[1]) / (fixedNum + 1);
				truXYZ[2] = truXYZ[2] + (errXYZ[2] - truXYZ[2]) / (fixedNum + 1);
				fixedNum++;
			}
		}
		rewind(errfp); // 将文件指针指向开头
	}
	else if (refMode == 1)
	{
		M31EQU(refPoss, truXYZ);
	}
	else if (refMode == 2)
	{
		while (!feof(trufp))
		{
			if (!GetRoverInfo(trufp, &truTime, truBLH, NULL, NULL, bInit + 0))continue;
			if (bInit[0] == false)break;
		}
		if (bInit[0] == false)referType = 1; // 能够初始化，说明是NMEA格式真值
		bInit[0] = true; // 重新置为true
		rewind(trufp); // 将文件指针指向开头
	}

	// 计算误差序列
	if ((refMode == 0 || refMode == 1) && truXYZ[0] != 0.0)
	{
		while (!feof(errfp))
		{
			if (!GetRoverInfo(errfp, &errTime, errBLH, &errStatus, &errNSat, bInit + 1))continue;
			LLH2XYZ(errBLH, errXYZ, 0);
			XYZ2LLH(truXYZ, truBLH, 0);
			M31_M31(errXYZ, truXYZ, dXYZ);
			Vxyz2enu(truBLH, dXYZ, enu);
			fprintf(outfp, "%4d,", errTime.GPSWeek);
			fprintf(outfp, "%10.1f,", GetGPSTIMESow(errTime));
			fprintf(outfp, "%4d,", errNSat);
			fprintf(outfp, "%4d,", errStatus);
			fprintf(outfp, "%18.10f,", errBLH[0] * 180.0 / PI);
			fprintf(outfp, "%18.10f,", errBLH[1] * 180.0 / PI);
			fprintf(outfp, "%14.3f,", errBLH[2]);
			fprintf(outfp, "%8.3f,", enu[0]);
			fprintf(outfp, "%8.3f,", enu[1]);
			fprintf(outfp, "%8.3f", enu[2]);
			fprintf(outfp, "\n");
		}
	}
	else if (refMode == 2)
	{
		while (!feof(errfp) && !feof(trufp))
		{
			if (bTru)
			{
				if (referType == 0)if (!GetReferInfo(trufp, &truTime, truXYZ))continue;
				if (referType == 1)if (!GetRoverInfo(trufp, &truTime, truBLH, NULL, NULL, bInit + 0))continue;
			}
			if (bErr)
			{
				if (!GetRoverInfo(errfp, &errTime, errBLH, &errStatus, &errNSat, bInit + 1))continue;
			}

			double dt = MinusGPSTIME(truTime, errTime);

			if (fabs(dt) < 0.1)
			{
				LLH2XYZ(errBLH, errXYZ, 0);
				if (referType == 1)LLH2XYZ(truBLH, truXYZ, 0);
				double dXYZ[3];
				M31_M31(errXYZ, truXYZ, dXYZ);
				if (referType == 0)XYZ2LLH(truXYZ, truBLH, 0);
				Vxyz2enu(truBLH, dXYZ, enu);

				fprintf(outfp, "%4d,", errTime.GPSWeek);
				fprintf(outfp, "%10.1f,", GetGPSTIMESow(errTime));
				fprintf(outfp, "%4d,", errNSat);
				fprintf(outfp, "%4d,", errStatus);
				fprintf(outfp, "%18.10f,", errBLH[0] * 180.0 / PI);
				fprintf(outfp, "%18.10f,", errBLH[1] * 180.0 / PI);
				fprintf(outfp, "%14.3f,", errBLH[2]);
				fprintf(outfp, "%8.3f,", enu[0]);
				fprintf(outfp, "%8.3f,", enu[1]);
				fprintf(outfp, "%8.3f", enu[2]);
				fprintf(outfp, "\n");
			}
			else if (dt > 0.0)
			{
				bTru = false;
				bErr = true;
			}
			else if (dt < 0.0)
			{
				bTru = true;
				bErr = false;
			}
		}
	}
	else return;

	if (errfp)fclose(errfp);
	if (trufp)fclose(trufp);
	if (outfp)fclose(outfp);
}