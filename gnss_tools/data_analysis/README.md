1. 20250709, Lying, 分析时屏蔽部分北斗卫星: 
"C01","C02","C03","C04","C05","C06","C11","C12","C13","C16",
"C31","C33","C38","C39","C40","C56","C58","C59","C60"

//// tmp: 输出可见卫星数和PDOP, edit by Lying on 20241102
//static FILE* fpSat = NULL;
//if (fpSat == NULL)
//{
//	fopen_s(&fpSat, "NSatPDOP.txt", "w");
//}
//else
//{
//	int nsat[NSYS][NFREQ] = { {0} };
//	// 统计卫星数量
//	for (int i = 0; i < IPSOBS.nsat; i++)
//	{
//		int prn = IPSOBS.obs[i].prn;
//		int isys = RTKPoint.m_SatStatus[IROVE][prn].sys_id;
//		for (int f = 0; f < NFREQ; f++)
//		{
//			if (IPSOBS.obs[i].P[f] != 0.0)nsat[isys][f]++;
//		}
//	}
//	// 计算PDOP
//	double DOPs[3] = { 0.0 };
//	ComputeDOP(NULL, NULL, GNSSOption.m_ElevMask, DOPs, RTKPoint.m_SatStatus[IROVE], false, false);
//	// 输出
//	fprintf(fpSat, "%4d,%8.1f,", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt));
//	for (int isys = 0; isys < NSYS; isys++)
//	{
//		for (int f = 0; f < NFREQ; f++)
//		{
//			fprintf(fpSat, "%2d,", nsat[isys][f]);
//		}
//	}
//	fprintf(fpSat, "%6.1f\n", DOPs[1]);
//}
// tmp: 输出信噪比分布, edit by Lying on 20241102
static FILE* fpSnr = NULL;
if (fpSnr == NULL)
{
	fopen_s(&fpSnr, "SNR.txt", "w");
}
else
{
	// [0,20),[20,30),[30,40),[40,60),分频分系统分区间
	int snr[NSYS][NFREQ][4] = { {{0}} };
	for (int i = 0; i < IPSOBS.nsat; i++)
	{
		int prn = IPSOBS.obs[i].prn;
		int isys = RTKPoint.m_SatStatus[IROVE][prn].sys_id;
		for (int f = 0; f < NFREQ; f++)
		{
			if (IPSOBS.obs[i].S[f] > 0.0 && IPSOBS.obs[i].S[f] < 20.0)snr[isys][f][0]++;
			if (IPSOBS.obs[i].S[f] >= 20.0 && IPSOBS.obs[i].S[f] < 30.0)snr[isys][f][1]++;
			if (IPSOBS.obs[i].S[f] >= 30.0 && IPSOBS.obs[i].S[f] < 40.0)snr[isys][f][2]++;
			if (IPSOBS.obs[i].S[f] >= 40.0 && IPSOBS.obs[i].S[f] < 60.0)snr[isys][f][3]++;
		}
	}

	// 输出
	fprintf(fpSnr, "%4d,%8.1f,", IPSOBS.gt.GPSWeek, GetGPSTIMESow(IPSOBS.gt));
	for (int isys = 0; isys < NSYS; isys++)
	{
		for (int f = 0; f < NFREQ; f++)
		{
			//double tmp = snr[isys][f][0] + snr[isys][f][1] + snr[isys][f][2] + snr[isys][f][3];
			fprintf(fpSnr, "%6d,%6d,%6d,%6d,", snr[isys][f][0], snr[isys][f][1], snr[isys][f][2], snr[isys][f][3]);
		}
	}
	fprintf(fpSnr, "\n");
}