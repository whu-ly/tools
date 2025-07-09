#include <time.h>
#include <stdlib.h>
#include "CGNSSApplication.h"
#include "BaseTime.h"
#include "BaseCmnFunc.h"
#include "BaseMatrix.h"


/*
* @brief       ��ʱ��(Timer)
* @return      double      �볤(ms)
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


// Ϊ���㶨λ�����׼��
static bool GetRoverInfo(FILE* fp, GPSTIME* gt, double* BLH, int* status, int* satNum, bool* bInit)// �õ����������е�����
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

	// ��ȡʱ�䣬��Ҫ�ǻ�ȡ��
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
						time.hour = 1; // ��һ��ֵ����ֹ����
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
				if (i == 1) // ʱ��
				{
					sscanf(p, "%2d%2d%lf", &time1.hour, &time1.min, &time1.sec);
					*gt = UTC2GPST(YMDHMS2GPST(time1));
					// ���������Ҫ���ݴ��Լ�����
				}
				else if (i == 2) // γ��
				{
					double B = atof(p);
					int deg = (int)(B * 0.01);
					double min = B - deg * 100;
					BLH[0] = (deg + min / 60.0) * PI / 180.0;
				}
				else if (i == 4) // ����
				{
					double L = atof(p);
					int deg = (int)(L * 0.01);
					double min = L - deg * 100;
					BLH[1] = (deg + min / 60.0) * PI / 180.0;
				}
				else if (i == 6) // ���ʶ
				{
					if (status)*status = atoi(p);
				}
				else if (i == 7) // ���ʶ
				{
					if (satNum)*satNum = atoi(p);
				}
				else if (i == 9) // �߳�
				{
					BLH[2] = atof(p);
				}
				else if (i == 11) // �߳��쳣
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


static bool GetReferInfo(FILE* fp, GPSTIME* gt, double* XYZ) // �õ��ο���ֵ������
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
			if (i == 0) // GPS��
			{
				gt->GPSWeek = atoi(p);
				// ���������Ҫ���ݴ��Լ�����
			}
			else if (i == 1) // GPS������
			{
				double secs = atof(p);
				gt->secsOfWeek = (int)secs;
				gt->fracOfSec = secs - gt->secsOfWeek;
			}
			else if (i == 16) // X����
			{
				XYZ[0] = atof(p);
			}
			else if (i == 17) // Y����
			{
				XYZ[1] = atof(p);
			}
			else if (i == 18) // Z����
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
* @brief		����Ĭ�����ñȽ���ֵ�ļ�(HW_TRJ��ʽ)�ͽ���ļ�(NMEA��ʽ)
* @param[in]	refPoss		double		��ֵ���꣨��̬�����踳ֵ��
* @param[in]	truFile		char[]		��ֵ�ļ�������̬�����踳ֵ��
* @param[in]	errFile		char[]		����ļ���
* @param[in]	outFile		char[]		����ļ���
* return		void
* @note         refPoss����truFile����Ч����ȡ�̶���ƽ��ֵ��Ϊ��ֵ
* @internals
*/
void GetPosErrs(double *refPoss, char truFile[], char errFile[], char outFile[])
{
	GPSTIME errTime, truTime;
	double errBLH[3] = { 0.0 }, errXYZ[3] = { 0.0 }, truBLH[3] = { 0.0 }, truXYZ[3] = { 0.0 }, enu[3] = { 0.0 }, dXYZ[3];
	int errStatus = 0, errNSat = 0, refMode = 2; // 0 = errFile�ļ��̶����ֵ��Ϊ��ֵ��1 = refPoss�ṩ������Ϊ��ֵ��2 = truFile�ṩ����������Ϊ��ֵ
	bool bErr = true, bTru = true, referType = 0, bInit[2] = { true,true }; // 0=ref,1=nmea; 0=refer,1=rover

	// ���ļ�
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

	// ��ȡ��ֵ
	if (refMode == 0)
	{
		int fixedNum = 0;
		while (!feof(errfp))
		{
			if (!GetRoverInfo(errfp, &errTime, errBLH, &errStatus, &errNSat, bInit + 1))continue;
			LLH2XYZ(errBLH, errXYZ, 0);
			if (errStatus == 4) // �̶���
			{
				truXYZ[0] = truXYZ[0] + (errXYZ[0] - truXYZ[0]) / (fixedNum + 1);
				truXYZ[1] = truXYZ[1] + (errXYZ[1] - truXYZ[1]) / (fixedNum + 1);
				truXYZ[2] = truXYZ[2] + (errXYZ[2] - truXYZ[2]) / (fixedNum + 1);
				fixedNum++;
			}
		}
		rewind(errfp); // ���ļ�ָ��ָ��ͷ
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
		if (bInit[0] == false)referType = 1; // �ܹ���ʼ����˵����NMEA��ʽ��ֵ
		bInit[0] = true; // ������Ϊtrue
		rewind(trufp); // ���ļ�ָ��ָ��ͷ
	}

	// �����������
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