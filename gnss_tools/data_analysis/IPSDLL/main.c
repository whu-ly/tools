#include <stdio.h>
#include <direct.h> // mkdir
#include "BaseTime.h"
#include "BaseCmnFunc.h"
#include "RTKLIB2IPS.h"
#include "GNSSCmnFunc.h"
#include "CRTKPoint.h"
#include "rtklib.h"
#include "CGNSSApplication.h"
#include "rinex.h"


static bool bSingleProcess = true; // 单个处理，配置文件拖入；批处理，配置文件读入，且关闭所有输出


// 配置文件结构体，仅适用于该版程序
typedef struct tagSolOpt
{
	char FilePath[1024];    // 文件路径
	int DataType[2];        // 0=ROVE,1=BASE;0=NMEA,1=RTCM,2=RENIX
	char BaseFile[1024];    // 基准站文件名
	char RoveFile[1024];    // 流动站文件名
	char NaviFile[1024];    // 星历文件名
	char RefFile[1024];     // 参考文件（动态模式）
	double RefPoss[3];      // 参考坐标（静态模式）
	char DeviceType[1024];  // 接收机类型
	char bGNSSProcAll;      // 是否全时段处理
	GPSTIME st;             // 数据处理起始时间
	GPSTIME et;             // 数据处理结束时间

} SolOpt;

/* main functions */
int main_FILE(int argc, char* argv[]);
int get_oneobs(FILE* fp, rtcm_t* rtcm, int datatype);
int ReadOpt(char optF[], SolOpt* solopt);
int OpenFile(SolOpt* solopt);


int main(int argc, char* argv[])
{
	SOLVE_TYPE solveType = SOLVE_FILE;
	int flag = 0;

	switch (solveType)
	{
	case SOLVE_FILE: flag = main_FILE(argc, argv); break;
	default:                                       break;
	}
	printf("SUCCESS\n");
	return flag;
}


FILE* rove_fp, * base_fp, * navi_fp, * res_fp, * log_fp, * obs_fp, * bas_fp, * quality_fp, * BaseSat_fp, * csEst_fp, * phaseTrack_fp;
int main_FILE(int argc, char* argv[])
{
	// 获取配置文件路径
	char optFileName[1024] = { '\0' };

	if (bSingleProcess)
	{
		printf("Please enter the file path:\n");
		printf("For example: D:\\IPS\\option.opt (Chinese path not currently supported)\n\n> ");
		gets_s(optFileName, sizeof(optFileName));
	}
	else
	{
		strncpy(optFileName, argv[1], sizeof(optFileName) - 1);
		optFileName[sizeof(optFileName) - 1] = '\0';
		printf("Solving in progress...\n");
	}

	// 读取配置文件
	SolOpt solopt;
	memset(&solopt, 0, sizeof(SolOpt));
	solopt.bGNSSProcAll = 1;
	solopt.DataType[0] = solopt.DataType[1] = 2;
	if (!ReadOpt(optFileName, &solopt))return 0;
	// 判断真值文件路径是否为空
	if (solopt.RefFile[0] == '\0' && solopt.RefPoss[0] ==0.0&& solopt.RefPoss[1] ==0.0&& solopt.RefPoss[2] == 0.0)
	{
		printf("There are no truth files or coordinates...\n");
		return 0;
	}

	// 打开数据文件
	if (!OpenFile(&solopt))return 0;

	// 以下为数据流和模块调用流程
	static rtcm_t           rtcm[2];         // RTCM control {rov,base}
	static GPSEPH		    IPSEph[NSATMAX];
	static OBS_DATA		    IPSOBS;
	static OBS_DATA         IPSOBSBASE;
	static CGNSSOption      GNSSOption;
	static CGNSSSolution    GNSSSoltion;
	static CRTKPointBase    RTKPoint;
	GPSTIME                 gt;
	RINEX_HEAD              head[2]; // {rov,base}
	int  nsatIPS = 0, nsatIPSBASE = 0, nephIPS[2] = { 0 };
	bool bRove = true, bBase = true;

	// 初始化
	init_rtcm(rtcm + 0);
	init_rtcm(rtcm + 1);
	for (int i = 0; i < NSATMAX; i++)InitGPSEPH(&IPSEph[i]);
	InitOBSDATA(&IPSOBS);
	InitOBSDATA(&IPSOBSBASE);
	InitCGNSSOption(&GNSSOption);
	InitCGNSSSolution(&GNSSSoltion);
	InitCRTKPointBase(&RTKPoint);
	InitGPSTIME2(-10, 0, &gt);

	if (strncmp(solopt.DeviceType, "GDT_LowCost_uBloxF9P", sizeof(solopt.DeviceType)) == 0)
		GNSSOption.m_GNSSDeviceType = GDT_LowCost_uBloxF9P;
	else if (strncmp(solopt.DeviceType, "GDT_LowCost_uBloxF9PN", sizeof(solopt.DeviceType)) == 0)
		GNSSOption.m_GNSSDeviceType = GDT_LowCost_uBloxF9PN;
	else
		GNSSOption.m_GNSSDeviceType = GDT_LowCostReceiver;

	if (solopt.DataType[0] == 2)
	{
		InitRINEX_HEAD(head);
		RinexHeadRead(rove_fp, head + 0);
		readAllEphHead(navi_fp, &RTKPoint.m_EphComputer);
		readAllEphBody(navi_fp, &RTKPoint.m_EphComputer);
	}
	if (solopt.DataType[1] == 2)
	{
		InitRINEX_HEAD(head + 1);
		RinexHeadRead(base_fp, head + 1);
	}

	//将参考真值存进RTKPoint
	//真值坐标（静态）
	RTKPoint.m_RefXYZ[0] = solopt.RefPoss[0]; RTKPoint.m_RefXYZ[1] = solopt.RefPoss[1]; RTKPoint.m_RefXYZ[2] = solopt.RefPoss[2];
	//真值序列（动态）
	sprintf(RTKPoint.m_RefFn, "%s\\%s", solopt.FilePath, solopt.RefFile);

	StartTimer(); // 统计运行耗时
	while (!feof(rove_fp) && !feof(base_fp))
	{
		// 读取数据
		if (bRove)
		{
			if (solopt.DataType[0] == 2)
			{
				if (!RinexBodyRead(rove_fp, head + 0, &IPSOBS)) break;
			}
			else if (solopt.DataType[0] == 0 || solopt.DataType[0] == 1)
			{
				if (!get_oneobs(rove_fp, rtcm + 0, solopt.DataType[0])) break;
			}
			else return -999;
		}
		if (bBase)
		{
			if (solopt.DataType[1] == 2)
			{
				if (!RinexBodyRead(base_fp, head + 1, &IPSOBSBASE)) break;
			}
			else if (solopt.DataType[1] == 0 || solopt.DataType[1] == 1)
			{
				if (!get_oneobs(base_fp, rtcm + 1, solopt.DataType[1])) break;
			}
			else return -999;
		}

		/* covert to ips */
		// OBS rover
		if (solopt.DataType[0] == 0 || solopt.DataType[0] == 1) // 需要转换的是NMEA和RTCM数据格式
		{
			ConvertOBS(rtcm[0].obs.data, rtcm[0].obs.n, &IPSOBS, &nsatIPS);
		}

		// OBS base
		if (solopt.DataType[1] == 0 || solopt.DataType[1] == 1) // 需要转换的是NMEA和RTCM数据格式
		{
			if (rtcm[1].obs.n > 0)
			{
				ConvertOBS(rtcm[1].obs.data, rtcm[1].obs.n, &IPSOBSBASE, &nsatIPSBASE);

				if (IPSOBSBASE.nsat > 4)
				{
					GNSSOption.m_BaseXYZ[0][0] = rtcm[1].sta.pos[0];
					GNSSOption.m_BaseXYZ[0][1] = rtcm[1].sta.pos[1];
					GNSSOption.m_BaseXYZ[0][2] = rtcm[1].sta.pos[2];
				}
				rtcm[1].obs.n = 0;
			}
		}
		else if (solopt.DataType[1] == 2)
		{
			GNSSOption.m_BaseXYZ[0][0] = head[1].XYZPos[0];
			GNSSOption.m_BaseXYZ[0][1] = head[1].XYZPos[1];
			GNSSOption.m_BaseXYZ[0][2] = head[1].XYZPos[2];
		}

		// 流动站数据跨周，但星历周未更新时，需要手动跨周
		double dtime = MinusGPSTIME(IPSOBS.gt, gt);
		if (gt.GPSWeek > 10 && dtime < -302400.0)
		{
			IPSOBS.gt.GPSWeek += (int)round(fabs(dtime) / 604800.0);
		}
		dtime = MinusGPSTIME(IPSOBS.gt, IPSOBSBASE.gt);
		IPSOBSBASE.gt.GPSWeek += (int)round(fabs(dtime) / 604800.0);//不能解出周的问题

		// 输出观测值
		if (bRove)
		{
			OBS_DATA* obss = &IPSOBS;
			if (obs_fp)fprintf(obs_fp, "rove: %d,%.3f, %2d\n", obss->gt.GPSWeek, GetGPSTIMESow(obss->gt), obss->nsat);
			for (int j = 0; j < obss->nsat; j++)
			{
				char sat[5] = { 0 };
				satprn2nos((int)obss->obs[j].prn, sat);
				if (obs_fp)fprintf(obs_fp, "%s,", sat);
				for (int f = 0; f < NFREQ; f++)
				{
					if (obs_fp)fprintf(obs_fp, "%14.3f,%14.3f,%10.3f,%6.3f,",
						obss->obs[j].P[f],
						obss->obs[j].L[f],
						obss->obs[j].D[f],
						obss->obs[j].S[f]);
				}
				if (obs_fp)fprintf(obs_fp, "\n");
			}
			if (obs_fp)fprintf(obs_fp, "\n");
		}

		if (bBase)
		{
			OBS_DATA* obss = &IPSOBSBASE;
			if (obs_fp)fprintf(obs_fp, "base: %d,%.3f, %2d\n", obss->gt.GPSWeek, GetGPSTIMESow(obss->gt), obss->nsat);
			for (int j = 0; j < obss->nsat; j++)
			{
				char sat[5] = { 0 };
				satprn2nos((int)obss->obs[j].prn, sat);
				if (obs_fp)fprintf(obs_fp, "%s,", sat);
				for (int f = 0; f < NFREQ; f++)
				{
					if (obs_fp)fprintf(obs_fp, "%14.3f,%14.3f,%10.3f,%6.3f,",
						obss->obs[j].P[f],
						obss->obs[j].L[f],
						obss->obs[j].D[f],
						obss->obs[j].S[f]);
				}
				if (obs_fp)fprintf(obs_fp, "\n");
			}
			if (obs_fp)fprintf(obs_fp, "\n");
		}

		// 输出基站坐标
		if (bBase)
		{
			if (bas_fp)
				fprintf(bas_fp, "%8.1f,%14.3f,%14.3f,%14.3f\n", GetGPSTIMESow(IPSOBSBASE.gt), GNSSOption.m_BaseXYZ[0][0], GNSSOption.m_BaseXYZ[0][1], GNSSOption.m_BaseXYZ[0][2]);
		}

		// 读取星历
		if (solopt.DataType[0] == 0 || solopt.DataType[1] == 1)
		{
			ConvertNav(&rtcm[0].nav, IPSEph, NULL, nephIPS);
			for (int k = 0; k < NSATMAX; k++)
			{
				if (IPSEph[k].toe.GPSWeek < 10) continue;
				int sys = -1;
				satprn2no(k + 1, &sys);

				if (sys == SYSGPS)
				{
					RTKPoint.m_EphComputer.m_GPSEphDatas[k][0] = IPSEph[k];
					RTKPoint.m_EphComputer.m_bNavEph[ISYSGPS] = true;
				}
				if (sys == SYSBD2)
				{
					RTKPoint.m_EphComputer.m_BD2EphDatas[k - PRNBD2][0] = IPSEph[k];
					RTKPoint.m_EphComputer.m_bNavEph[ISYSBD2] = true;
				}
				if (sys == SYSBD3)
				{
					RTKPoint.m_EphComputer.m_BD3EphDatas[k - PRNBD3][0] = IPSEph[k];
					RTKPoint.m_EphComputer.m_bNavEph[ISYSBD3] = true;
				}
				if (sys == SYSGAL)
				{
					RTKPoint.m_EphComputer.m_GALEphDatas[k - PRNGAL][0] = IPSEph[k];
					RTKPoint.m_EphComputer.m_bNavEph[ISYSGAL] = true;
				}
				if (sys == SYSQZS)
				{
					RTKPoint.m_EphComputer.m_QZSEphDatas[k - PRNQZS][0] = IPSEph[k];
					RTKPoint.m_EphComputer.m_bNavEph[ISYSQZS] = true;
				}
			}
		}

		// 时间同步
		if (fabs(MinusGPSTIME(IPSOBS.gt, IPSOBSBASE.gt)) < 0.05)
		{
			bRove = true; bBase = true;
		}
		else if (MinusGPSTIME(IPSOBS.gt, IPSOBSBASE.gt) < 0.0)
		{
			bRove = true; bBase = false;
		}
		else
		{
			bRove = false; bBase = true;
		}

		// 设置起止时段
		if (solopt.bGNSSProcAll == false)
		{
			if (MinusGPSTIME(IPSOBS.gt, solopt.st) < 0.0)continue;
			if (MinusGPSTIME(IPSOBS.gt, solopt.et) > 0.0)continue;
		}

		// test，降低采样为1Hz，只处理整秒数据，暂时性措施，可随时关闭
		//if (IPSOBS.gt.fracOfSec > 0.05) continue;
		//if (GetGPSTIMESow(IPSOBS.gt) < 285400.0) continue;
		//if (GetGPSTIMESow(IPSOBS.gt) > 198318.0) continue;

		// GNSS解算与结果输出
		if (MinusGPSTIME(gt, IPSOBS.gt) < 0.0) // 防止流动站数据出现紊乱，比如历元后退
		{
			if (RTKPoint.m_bInitRTK)
			{
				InitRTK(&RTKPoint, &GNSSOption, &GNSSSoltion, &IPSOBS, &IPSOBSBASE);
				continue;
			}

			// tmp，输出静态数据周跳估计情况
			if (!RTKPoint.m_SPPoint.m_CSEstFp && csEst_fp)
			{
				RTKPoint.m_SPPoint.m_CSEstFp = csEst_fp;
			}

			if (runOneRTK(&RTKPoint))
			{
		        // tmp, 输出差分龄期
                static FILE* fpTmp = NULL;
                if (fpTmp == NULL)fopen_s(&fpTmp, "DifferentialTime.txt", "w");
				if (fpTmp) fprintf(fpTmp, "%8.1f,%8.1f,%5.2f\n",
					GetGPSTIMESow(RTKPoint.m_OBSTime[0]),
					GetGPSTIMESow(RTKPoint.m_OBSTime[1]),
					fabs(GetGPSTIMESow(RTKPoint.m_OBSTime[0]) - GetGPSTIMESow(RTKPoint.m_OBSTime[1])));

				//打印文件头
				static bool bPrintHead = true;
				if (bPrintHead)
				{
					if (RTKPoint.m_RefXYZ[0] != 0 && RTKPoint.m_RefXYZ[1] != 0 && RTKPoint.m_RefXYZ[2] != 0)
					{
						if (quality_fp)fprintf(quality_fp, "rove mode:static\n rove position: %10.4f%10.4f%10.4f\n", RTKPoint.m_RefXYZ[0], RTKPoint.m_RefXYZ[1], RTKPoint.m_RefXYZ[2]);
					}
					else
					{
						if (quality_fp)fprintf(quality_fp, "rove mode:kinematic\nrove posfile: %s\n", RTKPoint.m_RefFn);
					}
					if (quality_fp)fprintf(quality_fp, "GPS:L1,L2,L5\n");
					if (quality_fp)fprintf(quality_fp, "BD2:B1I,B2I,B3I\n");
					if (quality_fp)fprintf(quality_fp, "BD3:B1I,B1C,B2a\n");
					if (quality_fp)fprintf(quality_fp, "GAL:E1,E5b,E5a\n");
					if (quality_fp)fprintf(quality_fp, "sat PRN(IPS) az(deg) el(deg)   snr(dB-Hz) ddP(m) ddL(cycle) tdL(cycle) cP(m) cL(m) cD(m) CS_Type CS_GF(m) CS_MW(m)\n");
					bPrintHead = false;
				}
				//输出时间、卫星数
				printf(">%d,%.3f,%d\n", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt), RTKPoint.m_pOBSData[IROVE]->nsat);
				if (quality_fp)fprintf(quality_fp, ">%d,%.3f,%d,", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt),RTKPoint.m_pOBSData[IROVE]->nsat);
				
				//输出各频率参考星
				for (int i = 0; i < NFREQ; i++)
				{
					for (int j = 0; j < NSYS; j++)
					{
						int index= RTKPoint.m_BaseSat[i][j];
						if (index == -1)continue;
						int prn = RTKPoint.m_pOBSData[IROVE]->obs[index].prn;
						
						char sat[5] = { 0 };
						satprn2nos(prn, sat);
						if (BaseSat_fp)fprintf(BaseSat_fp, "%d %.3f %2d ", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt), i);
						if (BaseSat_fp)fprintf(BaseSat_fp, "%3s ", sat);
						if (BaseSat_fp)fprintf(BaseSat_fp, "%6d ", prn);
						if (BaseSat_fp)fprintf(BaseSat_fp, "%12.4f %12.4f\n", R2D* RTKPoint.m_SatStatus[IROVE][prn].azel[0], R2D* RTKPoint.m_SatStatus[IROVE][prn].azel[1]);
						if (quality_fp)fprintf(quality_fp, "%3s", sat);
					}
					//保证打印出的数据利于绘图分析
					if (RTKPoint.m_BaseSat[i][0]==-1&& RTKPoint.m_BaseSat[i][1] == -1&& RTKPoint.m_BaseSat[i][2] == -1&& RTKPoint.m_BaseSat[i][3] == -1&& RTKPoint.m_BaseSat[i][4] == -1&& RTKPoint.m_BaseSat[i][5] == -1)
					{
						if (BaseSat_fp)fprintf(BaseSat_fp, "%d %.3f %2d ", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt), i);
						if (BaseSat_fp)fprintf(BaseSat_fp, "null\n");
						if (quality_fp)fprintf(quality_fp,"null");
					}
					if (quality_fp)fprintf(quality_fp, ",");
				}
				if (quality_fp)fprintf(quality_fp, "\n");
				//遍历流动站观测卫星
				for (int i = 0; i < RTKPoint.m_pOBSData[IROVE]->nsat; i++)
				{
					GNSS_MEQ* pMEQ = RTKPoint.m_GLSInfo.MEQ + i;
					int prn = RTKPoint.m_pOBSData[IROVE]->obs[i].prn;
					char sat[5] = { 0 };
					satprn2nos(prn, sat);
					if (quality_fp)fprintf(quality_fp, "%3s", sat);
					if (quality_fp)fprintf(quality_fp, "%6d", prn);
					if (quality_fp)fprintf(quality_fp, "%4d", (pMEQ->OBSIndex < 0) ? 0 : 1);
					if (quality_fp)fprintf(quality_fp, "%12.4f%12.4f", R2D* RTKPoint.m_SatStatus[IROVE][prn].azel[0], R2D* RTKPoint.m_SatStatus[IROVE][prn].azel[1]);
					for (int f = 0; f < NFREQ; f++)
					{
						double lam = RTKPoint.m_SatStatus[IROVE][prn].lam[f];	
						if (quality_fp)fprintf(quality_fp, "%8.2f", IPSOBS.obs[i].S[f]);
						if (quality_fp)fprintf(quality_fp, "%16.4f", pMEQ->v_P[f]);
						if (quality_fp)fprintf(quality_fp, "%16.4f", (lam == 0.0) ? 0.0 : pMEQ->v_L[f] / lam);
						if (quality_fp)fprintf(quality_fp, "%16.4f", (lam == 0.0) ? 0.0 : pMEQ->tdv_L[f] / lam);
						if (quality_fp)fprintf(quality_fp, "%16.4f%16.4f%16.4f ",RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[0][f],RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[1][f], RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[2][f]);
						if (quality_fp)fprintf(quality_fp, "%4d", RTKPoint.m_SatStatus[IROVE][prn].CS_Type[f]);
						if (quality_fp)fprintf(quality_fp, "%16.4f%16.4f", RTKPoint.m_SatStatus[IROVE][prn].CS_dGF[f], RTKPoint.m_SatStatus[IROVE][prn].CS_MW[f]);

						// 输出后清空
						pMEQ->v_P[f] = 0.0;
						pMEQ->v_L[f] = 0.0;
						pMEQ->tdv_L[f] = 0.0;
						RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[0][f] = 0.0;
						RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[1][f] = 0.0;
						RTKPoint.m_SatStatus[IROVE][prn].OBSConsist[2][f] = 0.0;
					}
					if (quality_fp)fprintf(quality_fp, "\n");
				}
			}

			// tmp, 输出流动站相位跟踪及周跳情况
			if (0)
			{
				if (!phaseTrack_fp)
				{
					fopen_s(&phaseTrack_fp, "SignalTrack.txt", "w");
				}
				if (phaseTrack_fp)
				{
					for (int i = 0; i < IPSOBS.nsat; i++)
					{
						int prn = IPSOBS.obs[i].prn;
						double el = RTKPoint.m_SatStatus[IROVE][prn].azel[1] * R2D;
						if (el > 0.0 && el < 10.0) continue;
						char sprn[5];
						satprn2nos(prn, sprn);
						fprintf(phaseTrack_fp, "%4d,%8.1f,%3s,%3d,%d,%d,%d,%d,%d,%d\n",
							IPSOBS.gt.GPSWeek,
							GetGPSTIMESow(IPSOBS.gt),
							sprn, prn,
							(IPSOBS.obs[i].P[0] != 0.0) ? 1 : 0,
							(IPSOBS.obs[i].P[1] != 0.0) ? 1 : 0,
							(IPSOBS.obs[i].P[2] != 0.0) ? 1 : 0,
							(IPSOBS.obs[i].L[0] != 0.0) ? (RTKPoint.m_SatStatus[IROVE][prn].CS_Type[0] ? 2 : 1) : 0,
							(IPSOBS.obs[i].L[1] != 0.0) ? (RTKPoint.m_SatStatus[IROVE][prn].CS_Type[1] ? 2 : 1) : 0,
							(IPSOBS.obs[i].L[2] != 0.0) ? (RTKPoint.m_SatStatus[IROVE][prn].CS_Type[2] ? 2 : 1) : 0);
					}
				}
			}

			if (WriteSolution(RTKPoint.m_pGNSSSol))
			{
				if (res_fp)fprintf(res_fp, "%s", RTKPoint.m_pGNSSSol->m_SolGGA);//GNGGA
				if (log_fp)fprintf(log_fp, "%s", RTKPoint.m_pGNSSSol->m_SolLOG);//GNLOG
			}

			gt = IPSOBS.gt;
		}
	}
	double time = EndTimer();
	printf("Time consuption %8.1f (s)\n", time);

	if (rove_fp)fclose(rove_fp);
	if (base_fp)fclose(base_fp);
	if (navi_fp)fclose(navi_fp);
	if (res_fp)fclose(res_fp);
	if (log_fp)fclose(log_fp);
	if (obs_fp)fclose(obs_fp);
	if (bas_fp)fclose(bas_fp);
	if (quality_fp)fclose(quality_fp);
	if (BaseSat_fp)fclose(BaseSat_fp);
	if (csEst_fp)fclose(csEst_fp);
	if (phaseTrack_fp)fclose(phaseTrack_fp);

	return 1;
}


// 读取单历元NMEA格式数据
int get_oneobs_NMEA(FILE* fp, rtcm_t* rtcm)
{
	static char line[2048] = { 0 };
	char buff[20480] = { 0 };
	int i = 0;
	char c = '\r';
	bool read = false;

	while (!feof(fp))
	{
		if (!line[0]) fgets(line, sizeof(line), fp);
		if (!strncmp(line, "$PSTMTG,", strlen("$PSTMTG,"))) // begin
		{
			strncpy(buff + i, line, strlen(line));
			i += (int)strlen(line);
			strncpy(buff + i, &c, 1);
			i += 1;

			while (!feof(fp))
			{
				memset(line, '\0', sizeof(line));
				fgets(line, sizeof(line), fp);

				if (!strncmp(line, "$PSTMTG,", strlen("$PSTMTG,")))
				{
					read = true;
					break;
				}

				if ((i + strlen(line)) >= sizeof(buff)) //防止Buff超限
				{
					memset(buff, '\0', sizeof(buff));
					i = 0;
					break;
				}

				strncpy(buff + i, line, strlen(line));
				i += (int)strlen(line);
				strncpy(buff + i, &c, 1);
				i += 1;
			}
			if (read)
			{
				decodeNMEA(buff, rtcm);
				return 1;
			}
		}
		memset(line, '\0', sizeof(line));
		fgets(line, sizeof(line), fp);
	}

	return 0;
}


// 读取单历元RTCM3格式数据
int get_oneobs_RTCM(FILE* fp, rtcm_t* rtcm)
{
	while (!feof(fp))
	{
		if (input_rtcm3f(rtcm, fp) == 1) // 解码到观测数据
			return 1;
	}
	return 0;
}


// 读取单历元原始数据
int get_oneobs(FILE* fp, rtcm_t* rtcm, int datatype)
{
	int i = 0;
	switch (datatype)
	{
	case 0:  i = get_oneobs_NMEA(fp, rtcm); break; // NMEA
	case 1:  i = get_oneobs_RTCM(fp, rtcm); break;
	default: break;
	}
	return i;
}


void CheckFilePath(char* FilePath)
{
	if (!FilePath) return;

	int n = (int)strlen(FilePath);
	for (int i = 0; i < n; i++)
	{
		if (FilePath[i] == '/') FilePath[i] = '\\';
	}

	// 处理尾随空格
	while (n > 0 && isspace((unsigned char)FilePath[n - 1])) {
		FilePath[n - 1] = '\0';
		n--;
	}
}


void removeFileExtension(char* path) {
	char* dot = strrchr(path, '.');  // 查找最后一个点的位置
	if (dot != NULL) {
		*dot = '\0';  // 将最后一个点替换为字符串终止符
	}
}


// 读取配置文件（xxx.opt)
int ReadOpt(char optF[], SolOpt* solopt)
{
	char oneline[1024] = { '\0' }, ch[1024] = { '\0' }, flag = 1;
	int week = 0;
	double secs = 0.0;

	FILE* opt_fp = fopen(optF, "r");

	if (!opt_fp) {
		printf("open option file fail\n");
		return 0;
	}

	solopt->RefPoss[0] = solopt->RefPoss[1] = solopt->RefPoss[2] = 0.0;

	while (!feof(opt_fp))
	{
		memset(oneline, 0, 1024);
		memset(ch, 0, 1024);
		fgets(oneline, 1024, opt_fp);
		if (flag)
		{
			if (strncmp(oneline, "GNSS_FilePath", strlen("GNSS_FilePath")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->FilePath, ch, sizeof(solopt->FilePath));
				continue;
			}
			else if (strncmp(oneline, "GNSS_BaseFile", strlen("GNSS_BaseFile")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->BaseFile, ch, sizeof(solopt->BaseFile));
				continue;
			}
			else if (strncmp(oneline, "GNSS_RoveFile", strlen("GNSS_RoveFile")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->RoveFile, ch, sizeof(solopt->RoveFile));
				continue;
			}
			else if (strncmp(oneline, "GNSS_NaviFile", strlen("GNSS_NaviFile")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->NaviFile, ch, sizeof(solopt->NaviFile));
				continue;
			}
			else if (strncmp(oneline, "GNSS_RefFile", strlen("GNSS_RefFile")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->RefFile, ch, sizeof(solopt->RefFile));
				continue;
			}
			else if (strncmp(oneline, "GNSS_RoveType", strlen("GNSS_RoveType")) == 0) {
				sscanf(oneline + 32, "%d", &solopt->DataType[0]);
				continue;
			}
			else if (strncmp(oneline, "GNSS_BaseType", strlen("GNSS_BaseType")) == 0) {
				sscanf(oneline + 32, "%d", &solopt->DataType[1]);
				continue;
			}
			else if (strncmp(oneline, "GNSS_DeviceType", strlen("GNSS_DeviceType")) == 0) {
				//sscanf(oneline + 32, "%s", ch);
				sscanf(oneline + 32, "%[^;]", ch);
				CheckFilePath(ch);
				memcpy(solopt->DeviceType, ch, sizeof(solopt->DeviceType));
				continue;
			}
			else if (strncmp(oneline, "GNSS_RefPoss", strlen("GNSS_RefPoss")) == 0) {
				sscanf(oneline + 32, "%lf,%lf,%lf", &solopt->RefPoss[0], &solopt->RefPoss[1], &solopt->RefPoss[2]);
				continue;
			}
			else if (strncmp(oneline, "GNSS_bGNSSProcAll", strlen("GNSS_bGNSSProcAll")) == 0) {
				sscanf(oneline + 32, "%hhd", &solopt->bGNSSProcAll);
				continue;
			}
			else if (strncmp(oneline, "GNSS_BeginTime", strlen("GNSS_BeginTime")) == 0) {
				sscanf(oneline + 32, "%d,%lf", &week, &secs);
				InitGPSTIME2(week, secs, &solopt->st);
				continue;
			}
			else if (strncmp(oneline, "GNSS_EndTime", strlen("GNSS_EndTime")) == 0) {
				sscanf(oneline + 32, "%d,%lf", &week, &secs);
				InitGPSTIME2(week, secs, &solopt->et);
				continue;
			}
		}
	}

	// 判断文件路径是否为空
	if (solopt->FilePath[0] == '\0')
	{
		// 查找字符串 "\\opt" 的位置
		const char* optPosition = strstr(optF, "\\opt");

		// 检查是否找到 "\\opt"
		if (optPosition != NULL)
		{
			size_t length = optPosition - optF;
			strncpy(solopt->FilePath, optF, length);
			solopt->FilePath[length] = '\0';
		}
		else return 0;
	}

	return 1;
}


// 打开所有文件，包括数据文件和输出文件
int OpenFile(SolOpt* solopt)
{
	if (!solopt)return 0;

	char rove_fn[1024], base_fn[1024], navi_fn[1024], rfilepath[1024], res_fn[1024], log_fn[1024], obs_fn[1024], bas_fn[1024], quality_fn[1024], BaseSat_fn[1024], csEst_fn[1024], phaseTrack_fn[1024];
	
	// 原始数据文件
	sprintf(rove_fn, "%s\\%s", solopt->FilePath, solopt->RoveFile);
	sprintf(base_fn, "%s\\%s", solopt->FilePath, solopt->BaseFile);
	sprintf(navi_fn, "%s\\%s", solopt->FilePath, solopt->NaviFile);


	// 解算结果文件
	removeFileExtension(solopt->RoveFile);
	sprintf(rfilepath, "%s\\resi", solopt->FilePath);
	sprintf(res_fn, "%s\\resi\\%s_%s\\IPS_SV.pos", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(log_fn, "%s\\resi\\%s_%s\\IPS_SV.log", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(obs_fn, "%s\\resi\\%s_%s\\IPS_SV.obs", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(bas_fn, "%s\\resi\\%s_%s\\IPS_SV.bas", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(quality_fn, "%s\\resi\\%s_%s\\IPS_SV1.Residual", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(BaseSat_fn, "%s\\resi\\%s_%s\\IPS_SV_BaseSat1.Base", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(csEst_fn, "%s\\resi\\%s_%s\\CSEstimation.txt", solopt->FilePath, IPSSVERSION, solopt->RoveFile);
	sprintf(phaseTrack_fn, "%s\\resi\\%s_%s\\PhaseTrack.txt", solopt->FilePath, IPSSVERSION, solopt->RoveFile);

	if (_mkdir(rfilepath) != 0 && _mkdir(rfilepath) != -1)return 0;
	sprintf(rfilepath, "%s\\%s_%s", rfilepath, IPSSVERSION, solopt->RoveFile);
	if (_mkdir(rfilepath) != 0 && _mkdir(rfilepath) != -1)return 0;

	int error1 = fopen_s(&rove_fp, rove_fn, "rb"); // open file
	int error2 = fopen_s(&base_fp, base_fn, "rb");
	int error3 = fopen_s(&navi_fp, navi_fn, "rb");

	//fopen_s(&res_fp, res_fn, "w");
	//fopen_s(&log_fp, log_fn, "w");
	//fopen_s(&obs_fp, obs_fn, "w");
	//fopen_s(&bas_fp, bas_fn, "w");
	int error4 = fopen_s(&quality_fp, quality_fn, "w");
	fopen_s(&BaseSat_fp, BaseSat_fn, "w");

	fopen_s(&csEst_fp, csEst_fn, "w");
	fopen_s(&phaseTrack_fp, phaseTrack_fn, "w");

	if (error1 || error2 || error4)
	{
		printf("Fail to open the file......\n");
		return 0;
	}
	if (solopt->DataType[0] == 2 || solopt->DataType[1] == 2)
	{
		if (error3)
		{
			printf("Fail to open the file......\n");
			return 0;
		}
	}

	return 1;
}