#include "rinex.h"
#include "BaseCmnFunc.h"
#include "BaseTime.h"
#include "GNSSCmnFunc.h"

#include <stdio.h>
#include <stdlib.h>


void InitRINEX_HEAD(RINEX_HEAD* pStruct)
{
	for (int i = 0; i < NSYS; i++)
	{
		for (int f = 0; f < NFREQ; f++)pStruct->PhaseShiftCorr[i][f] = 0;
		for (int j = 0; j < 4 * NFREQ; j++)pStruct->GNSSObsPos[i][j] = 0;
		pStruct->nGNSSTypeRead[i] = 0;
	}
	for (int f = 0; f < NFREQ; f++)
	{
		pStruct->PhaseShiftCorrX[f] = 0;
		for (int j = 0; j < 4; j++)pStruct->GPSObsPosX[f * 4 + j] = 0;
	}
	pStruct->XYZPos[0] = pStruct->XYZPos[1] = pStruct->XYZPos[2] = 0.0;
	memset(pStruct->GNSSTypeRead, 0, sizeof(pStruct->GNSSTypeRead));
}


// 观测值按自定义PRN号进行排序
static void SortOBS(OBS_DATA* src)
{
	if (!src || src->nsat <= 1)return;

	OBS_DATA_t dst[MAXOBS];
	int i, j = 0;
	int prnlist[NSATMAX + 1] = { -1 };

	memset(prnlist, -1, (NSATMAX + 1) * sizeof(int));

	for (i = 0; i < src->nsat; i++)prnlist[src->obs[i].prn] = i;

	for (i = 1; i < NSATMAX + 1; i++)
	{
		if (prnlist[i] < 0)continue;
		dst[j++] = src->obs[prnlist[i]];
	}
	memcpy(src->obs, dst, sizeof(OBS_DATA_t) * src->nsat);
}


/*
* @brief		读取rinex3.0以上格式观测数据文件头
* @param[in]	ifp			FILE			观测文件指针
* @param[out]	head		RINEX_HEAD		观测值文件头
* @param[out]	XYZ			double			头文件坐标
* @return		bool
* @note			没有考虑PHASE SHIFT！！！
* @internals
*/
bool RinexHeadRead(FILE* ifp, RINEX_HEAD* head)
{
	if (!ifp || !head) return false;

	const char sys_str[NSYS + 1] = { "GRCCEJ" };

	char buff[MAXSIZE], * label = buff + 60, ch[128] = { '\0' }, timeSys[10] = "GPS", GNSSType[NSYS][MAXCODE][10];
	int i, j, k, f, version = 0, sys_id = 0, GNSSTypeNum[NSYS] = { 0 };
	//double dVal = 0;

	while (fgets(buff, MAXSIZE, ifp))
	{
		if (strstr(label, "RINEX VERSION / TYPE"))
		{
			xstrmid(buff, 5, 1, ch);
			version = atoi(ch);
		}

		//else if (strstr(label, "REC # / TYPE / VERS"))
		//{
		//	xstrmid(buff, 20, 20, obsHead.recType);
		//}

		//else if (strstr(label, "ANT # / TYPE"))
		//{
		//	xstrmid(buff, 20, 20, obsHead.antType);
		//}

		else if (strstr(label, "APPROX POSITION XYZ"))
		{
			if (head)
			{
				head->XYZPos[0] = str2num(buff, 0, 14);
				head->XYZPos[1] = str2num(buff, 14, 14);
				head->XYZPos[2] = str2num(buff, 28, 14);
			}
		}

		//else if (strstr(label, "ANTENNA: DELTA H/E/N"))
		//{
		//	obsHead.ant[2] = str2num(buff, 0, 14);  // H
		//	obsHead.ant[0] = str2num(buff, 14, 14); // E
		//	obsHead.ant[1] = str2num(buff, 28, 14); // N
		//}

		else if (strstr(label, "SYS / # / OBS TYPES"))
		{
			sys_id = -1;

			for (k = 0; k < NSYS; k++)
			{
				if (buff[0] == sys_str[k])
				{
					sys_id = k; break;
				}
			}

			if (sys_id < 0) continue;

			GNSSTypeNum[sys_id] = (int)str2num(buff, 3, 3);

			for (i = 0, j = 7; i < GNSSTypeNum[sys_id]; i++, j += 4)
			{
				if (j > 58)
				{
					if (!fgets(buff, MAXSIZE, ifp)) return false;
					j = 7;
				}

				xstrmid(buff, j, 3, ch);

				// 有些BDS定义的是2
				if (buff[0] == 'C' && (ch[2] == 'I' || ch[2] == 'Q') && ch[1] == '1') ch[1] = '2';

				strcpy(GNSSType[sys_id][i], ch);
				//GNSSType[sys_id].push_back(string(ch));
			}

			// BD2和BD3设为一样
			if (sys_id == ISYSBD2)
			{
				GNSSTypeNum[ISYSBD3] = GNSSTypeNum[ISYSBD2];
				for (int i = 0; i < GNSSTypeNum[ISYSBD2]; i++)
					strcpy(GNSSType[ISYSBD3][i], GNSSType[ISYSBD2][i]);
				//GNSSType[ISYSBD3] = GNSSType[ISYSBD2];
			}
		}

		//else if (strstr(label, "SYS / PHASE SHIFT"))
		//{
		//	sys_id = -1;

		//	for (int k = 0; k < NSYS; k++)
		//	{
		//		if (buff[0] == sys_str[k])
		//		{
		//			sys_id = k; break;
		//		}
		//	}

		//	if (sys_id < 0) continue;


		//	xstrmid(buff, 2, 3, ch);
		//	dVal = str2num(buff, 6, 8);
		//	PhaseShift[sys_id][string(ch)] = dVal;

		//	// BD2和BD3设为一样
		//	if (sys_id == ISYSBD2)
		//	{
		//		PhaseShift[ISYSBD3][string(ch)] = dVal;
		//	}
		//}

		//else if (strstr(label, "INTERVAL"))
		//{
		//	obsHead.dt = str2num(buff, 0, 60);
		//}

		else if (strstr(label, "TIME OF FIRST OBS"))
		{
			xstrmid(buff, 48, 3, ch);
			if (ch[0] != ' ') // 空格
				strcpy(timeSys, ch);
		}

		else if (strstr(label, "END OF HEADER"))
		{
			break;
		}
	}

	if (version != 3)
	{
		printf("RINEX VERSION is not 3.0 !\n");
		return false;
	}

	if (strcmp(timeSys, "GPS"))
	{
		printf("time is not GPS !\n");
		return false;
	}

	char CodeType[4][2] = { "C", "L", "D", "S" };

	char GNSSCodePris[NSYS][5][12] = { { "CPYWMNSLX", "PYWCMNDSLX", "IQX"   , ""     , ""      }, // GPS
									 { "PC"       , "PC"        , "IQX"   , ""     , ""      }, // GLO
									 { "IQX"      , "IQX"       , "IQXA"  , ""     , ""      }, // BD2
									 { "IQX"      , "IQX"       , "IQXA"  , "DPXA" , "DPX"   }, // BD3
									 { "CABXZ"    , "IQX"       , "IQX"   , "IQX"  , "ABCXZ" }, // GAL
									 { "CSLXZ"    , "SLX"       , "IQXDPZ", "SLXEZ", ""      }, // QZS
	};

	char GNSSCodeFreq[NSYS][5][2] = { { "1","2","5"," "," " }, // GPS
									 { "1","2","3"," "," " }, // GLO
									 { "2","7","6"," "," " }, // BD2
									 { "2","7","6","1","5" }, // BD3
									 { "1","5","7","8","6" }, // GAL
									 { "1","2","5","6"," " }, // QZS
	};


	if (gs_bSwitchGNSSFrq) // 用于切换频点
	{
		for (f = 0; f < NFREQ; f++) // GPS
		{
			if (!strcmp(gs_strGPSFrq[f], "L1"))
			{
				strcpy(GNSSCodePris[ISYSGPS][f], "CPYWMNSLX");
				strcpy(GNSSCodeFreq[ISYSGPS][f], "1");
				//GNSSCodePris[ISYSGPS][f] = "CPYWMNSLX";
				//GNSSCodeFreq[ISYSGPS][f] = "1";
			}
			else if (!strcmp(gs_strGPSFrq[f], "L2"))
			{
				strcpy(GNSSCodePris[ISYSGPS][f], "PYWCMNDLSX"); // UBX 2S 频点经常没有数据，所以把S和L顺序调换
				strcpy(GNSSCodeFreq[ISYSGPS][f], "2");
				//GNSSCodePris[ISYSGPS][f] = "PYWCMNDSLX";
				//GNSSCodeFreq[ISYSGPS][f] = "2";
			}
			else if (!strcmp(gs_strGPSFrq[f], "L5"))
			{
				strcpy(GNSSCodePris[ISYSGPS][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSGPS][f], "5");
				//GNSSCodePris[ISYSGPS][f] = "IQX";
				//GNSSCodeFreq[ISYSGPS][f] = "5";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSGPS][f], "");
				strcpy(GNSSCodeFreq[ISYSGPS][f], "");
				//GNSSCodePris[ISYSGPS][f] = "";
				//GNSSCodeFreq[ISYSGPS][f] = "";
			}
		}

		for (f = 0; f < NFREQ; f++) // GLO
		{
			if (!strcmp(gs_strGLOFrq[f], "G1"))
			{
				strcpy(GNSSCodePris[ISYSGLO][f], "PC");
				strcpy(GNSSCodeFreq[ISYSGLO][f], "1");
				//GNSSCodePris[ISYSGLO][f] = "PC";
				//GNSSCodeFreq[ISYSGLO][f] = "1";
			}
			else if (!strcmp(gs_strGLOFrq[f], "G2"))
			{
				strcpy(GNSSCodePris[ISYSGLO][f], "PC");
				strcpy(GNSSCodeFreq[ISYSGLO][f], "2");
				//GNSSCodePris[ISYSGLO][f] = "PC";
				//GNSSCodeFreq[ISYSGLO][f] = "2";
			}
			else if (!strcmp(gs_strGLOFrq[f], "G3"))
			{
				strcpy(GNSSCodePris[ISYSGLO][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSGLO][f], "3");
				//GNSSCodePris[ISYSGLO][f] = "IQX";
				//GNSSCodeFreq[ISYSGLO][f] = "3";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSGLO][f], "");
				strcpy(GNSSCodeFreq[ISYSGLO][f], "");
				//GNSSCodePris[ISYSGLO][f] = "";
				//GNSSCodeFreq[ISYSGLO][f] = "";
			}
		}

		for (f = 0; f < NFREQ; f++) // BD2
		{
			if (!strcmp(gs_strBD2Frq[f], "B1I"))
			{
				strcpy(GNSSCodePris[ISYSBD2][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSBD2][f], "2");
				//GNSSCodePris[ISYSBD2][f] = "IQX";
				//GNSSCodeFreq[ISYSBD2][f] = "2";
			}
			else if (!strcmp(gs_strBD2Frq[f], "B2I"))
			{
				strcpy(GNSSCodePris[ISYSBD2][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSBD2][f], "7");
				//GNSSCodePris[ISYSBD2][f] = "IQX";
				//GNSSCodeFreq[ISYSBD2][f] = "7";
			}
			else if (!strcmp(gs_strBD2Frq[f], "B3I"))
			{
				strcpy(GNSSCodePris[ISYSBD2][f], "IQXA");
				strcpy(GNSSCodeFreq[ISYSBD2][f], "6");
				//GNSSCodePris[ISYSBD2][f] = "IQXA";
				//GNSSCodeFreq[ISYSBD2][f] = "6";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSBD2][f], "");
				strcpy(GNSSCodeFreq[ISYSBD2][f], "");
				//GNSSCodePris[ISYSBD2][f] = "";
				//GNSSCodeFreq[ISYSBD2][f] = "";
			}
		}

		for (f = 0; f < NFREQ; f++) // BD3
		{
			if (!strcmp(gs_strBD3Frq[f], "B1I"))
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "2");
				//GNSSCodePris[ISYSBD3][f] = "IQX";
				//GNSSCodeFreq[ISYSBD3][f] = "2";
			}
			else if (!strcmp(gs_strBD3Frq[f], "B2I") || !strcmp(gs_strBD3Frq[f], "B2b"))
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "7");
				//GNSSCodePris[ISYSBD3][f] = "IQX";
				//GNSSCodeFreq[ISYSBD3][f] = "7";
			}
			else if (!strcmp(gs_strBD3Frq[f], "B3I"))
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "IQXA");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "6");
				//GNSSCodePris[ISYSBD3][f] = "IQXA";
				//GNSSCodeFreq[ISYSBD3][f] = "6";
			}
			else if (!strcmp(gs_strBD3Frq[f], "B1C"))
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "DPX");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "1");
				//GNSSCodePris[ISYSBD3][f] = "ABCIQXYZ";
				//GNSSCodeFreq[ISYSBD3][f] = "1";
			}
			else if (!strcmp(gs_strBD3Frq[f], "B2a"))
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "PIQX");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "5");
				//GNSSCodePris[ISYSBD3][f] = "PIQX";
				//GNSSCodeFreq[ISYSBD3][f] = "5";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSBD3][f], "");
				strcpy(GNSSCodeFreq[ISYSBD3][f], "");
				//GNSSCodePris[ISYSBD3][f] = "";
				//GNSSCodeFreq[ISYSBD3][f] = "";
			}
		}

		for (f = 0; f < NFREQ; f++) // GAL
		{
			if (!strcmp(gs_strGALFrq[f], "E1"))
			{
				strcpy(GNSSCodePris[ISYSGAL][f], "CABXZ");
				strcpy(GNSSCodeFreq[ISYSGAL][f], "1");
				//GNSSCodePris[ISYSGAL][f] = "CABXZ";
				//GNSSCodeFreq[ISYSGAL][f] = "1";
			}
			else if (!strcmp(gs_strGALFrq[f], "E5a"))
			{
				strcpy(GNSSCodePris[ISYSGAL][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSGAL][f], "5");
				//GNSSCodePris[ISYSGAL][f] = "IQX";
				//GNSSCodeFreq[ISYSGAL][f] = "5";
			}
			else if (!strcmp(gs_strGALFrq[f], "E5b"))
			{
				strcpy(GNSSCodePris[ISYSGAL][f], "IQX");
				strcpy(GNSSCodeFreq[ISYSGAL][f], "7");
				//GNSSCodePris[ISYSGAL][f] = "IQX";
				//GNSSCodeFreq[ISYSGAL][f] = "7";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSGAL][f], "");
				strcpy(GNSSCodeFreq[ISYSGAL][f], "");
				//GNSSCodePris[ISYSGAL][f] = "";
				//GNSSCodeFreq[ISYSGAL][f] = "";
			}
		}

		for (f = 0; f < NFREQ; f++) // QZS
		{
			if (!strcmp(gs_strQZSFrq[f], "L1"))
			{
				strcpy(GNSSCodePris[ISYSQZS][f], "CSLXZ");
				strcpy(GNSSCodeFreq[ISYSQZS][f], "1");
				//GNSSCodePris[ISYSQZS][f] = "CSLXZ";
				//GNSSCodeFreq[ISYSQZS][f] = "1";
			}
			else if (!strcmp(gs_strQZSFrq[f], "L2"))
			{
				strcpy(GNSSCodePris[ISYSQZS][f], "SLX");
				strcpy(GNSSCodeFreq[ISYSQZS][f], "2");
				//GNSSCodePris[ISYSQZS][f] = "SLX";
				//GNSSCodeFreq[ISYSQZS][f] = "2";
			}
			else if (!strcmp(gs_strQZSFrq[f], "L5"))
			{
				strcpy(GNSSCodePris[ISYSQZS][f], "IQXDPZ");
				strcpy(GNSSCodeFreq[ISYSQZS][f], "5");
				//GNSSCodePris[ISYSQZS][f] = "IQXDPZ";
				//GNSSCodeFreq[ISYSQZS][f] = "5";
			}
			else
			{
				strcpy(GNSSCodePris[ISYSQZS][f], "");
				strcpy(GNSSCodeFreq[ISYSQZS][f], "");
				//GNSSCodePris[ISYSQZS][f] = "";
				//GNSSCodeFreq[ISYSQZS][f] = "";
			}
		}
	}

	bool bflag = false;
	char code[5];

	for (sys_id = 0; sys_id < NSYS; sys_id++)
	{
		for (int ncode = 0; ncode < 4; ncode++)
		{
			for (f = 0; f < NFREQ; f++)
			{
				bflag = false;

				// GPS得到X通道的观测值
				if (sys_id == 0)
				{
					//string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + "X";
					sprintf(code, "%s%sX", CodeType[ncode], GNSSCodeFreq[sys_id][f]);
					for (j = 0; j < GNSSTypeNum[sys_id]; j++)
					{
						if (!strcmp(code, GNSSType[sys_id][j]))
						{
							head->GPSObsPosX[ncode * NFREQ + f] = j + 1;
							break;
						}
					}
				}

				for (i = 0; i < strlen(GNSSCodePris[sys_id][f]); i++)
				{
					//string code = CodeType[ncode] + GNSSCodeFreq[sys_id][frq] + GNSSCodePris[sys_id][frq][i];
					sprintf(code, "%s%s%c", CodeType[ncode], GNSSCodeFreq[sys_id][f], GNSSCodePris[sys_id][f][i]);

					for (j = 0; j < GNSSTypeNum[sys_id]; j++)
					{
						if (!strcmp(code, GNSSType[sys_id][j]))
						{
							head->GNSSObsPos[sys_id][ncode * NFREQ + f] = j + 1;
							strcpy(head->GNSSTypeRead[sys_id][ncode * NFREQ + f], code);
							//head->GNSSTypeRead[sys_id][ncode * NFREQ + frq] = code;
							bflag = true;
							break;
						}
					}

					if (bflag) break;
				}

			}
		}
	}

	//// Phase Shift, X通道影响比较大,其它通道到底用加还是减还得验证下!!!
	////double PhaseShiftCorr[NSYS][NFREQ] = { 0.0 };
	////double PhaseShiftCorrX[NFREQ] = { 0.0 };
	//for (int i = 0; i < NSYS; i++)
	//{
	//	for (int j = 0; j < NFREQ; j++)
	//	{
	//		head->PhaseShiftCorr[i][j] = PhaseShift[i][head->GNSSTypeRead[i][NFREQ + j]];
	//	}
	//}

	//// 这里竟然用减!!!, RINEX3.02格式说明和IE8.7都是用加
	//// RINEX3.02给出来的公式是L_RENIX = L_Original + shift
	//// 所以要得到真实的值(L_Original),应该减去shift ??
	//head->PhaseShiftCorrX[0] = -PhaseShift[0]["L1X"];
	////PhaseShiftCorrX[1] = -PhaseShift[0]["L2X"];
	//head->PhaseShiftCorrX[1] = -PhaseShift[0]["L5X"];

	return true;
}


// 宝时得屏蔽卫星
int isInExcSats(const char sprn[5]) {
	char excSats[19][5] = {
		"C01","C02","C03","C04","C05","C06","C11","C12","C13","C16",
		"C31","C33","C38","C39","C40","C56","C58","C59","C60"
	};

	for (int i = 0; i < 19; i++) {
		if (strcmp(sprn, excSats[i]) == 0) {
			return 1; // 找到了
		}
	}
	return 0; // 不在数组中
}


/*
* @brief		读取rinex3.0以上格式观测数据体
* @param[in]	ifp			FILE			观测文件指针
* @param[in]	head		RINEX_HEAD		观测值文件头
* @param[out]	dst			OBS_DATA		观测值结构体
* @return		bool
* @note
* @internals
*/
bool RinexBodyRead(FILE* ifp, RINEX_HEAD* head, OBS_DATA* dst)
{
	if (!ifp || !dst || !head) return false;

	dst->flag = 0;
	dst->nsat = 0;
	for (int i = 0; i < NSYS; i++)dst->ngnss[i] = 0;
	memset(dst->obs, 0, MAXOBS * sizeof(OBS_DATA_t));

	char buff[MAXSIZE], * label = buff + 60;
	char ch[128] = { '\0' };

	int SatSYS = SYSALL;
	int flag = 0; // 事件标记
	int nsat = 0; // 每个历元卫星数
	int prn = 0, sys = 0, sys_id = 0;

	// 读数据主体
	while (fgets(buff, MAXSIZE, ifp))
	{
		if (feof(ifp)) break;

		/* decode obs epoch */
		if (buff[0] != '>')
		{
			if (strstr(label, "APPROX POSITION XYZ"))
			{
				if (head)
				{
					head->XYZPos[0] = str2num(buff, 0, 14);
					head->XYZPos[1] = str2num(buff, 14, 14);
					head->XYZPos[2] = str2num(buff, 28, 14);
				}
			}
			continue;
		}

		/* epoch flag: 3:new site,4:header info,5:external event */
		nsat = (int)str2num(buff, 32, 3);
		if (nsat <= 0) continue;

		flag = (int)str2num(buff, 31, 1);
		if (3 <= flag && flag <= 5)
		{
			// 3-5为时间标识，下面有说明，将其读取
			for (int p = 0; p < nsat; p++) fgets(buff, MAXSIZE, ifp);
			continue;
		}

		dst->gt = str2time(buff, 1, 28);

		int  nsatValid = 0; // 有效卫星数

		for (int i = 0; i < nsat; i++)
		{
			if (nsatValid >= MAXOBS)break;
			OBS_DATA_t* obst = dst->obs + nsatValid;

			fgets(buff, MAXSIZE, ifp);

			xstrmid(buff, 0, 3, ch);
			prn = satno2prn1(ch);
			if (prn == 0) continue;

			// for test, 屏蔽部分北斗卫星(欧洲测试需要)
			char sprn[5];
			satprn2nos(prn, sprn);
			if (isInExcSats(sprn)) continue;

			sys = SYSNON;
			satprn2no(prn, &sys);

			bool bpush = false;

			sys_id = Sys2Index(sys);

			if (SatSYS & sys)
			{
				//bpush = true;
				int pos = 0;
				for (int k = 0; k < NFREQ; k++)
				{
					bool bX = false;

					// L
					pos = head->GNSSObsPos[sys_id][NFREQ + k];
					if (pos > 0)
					{
						pos--;
						obst->L[k] = str2num(buff, 3 + 16 * pos, 14);
						obst->LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
						obst->SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
						if (obst->L[k] != 0.0) obst->L[k] += head->PhaseShiftCorr[0][k];

						// 如果W通道为0,则启动X通道
						bX = (sys == SYSGPS && obst->L[k] == 0.0 && head->GNSSTypeRead[sys_id][NFREQ + k][2] == 'W');
					}

					if (bX)
					{
						pos = head->GPSObsPosX[NFREQ + k];
						if (pos > 0)
						{
							pos--;
							obst->L[k] = str2num(buff, 3 + 16 * pos, 14);
							obst->LLI[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 14, 1);
							obst->SNR[k] = (unsigned char)str2num(buff, 3 + 16 * pos + 15, 1);
							if (obst->L[k] != 0.0) obst->L[k] += head->PhaseShiftCorrX[k];
						}

						// P
						pos = head->GPSObsPosX[k];
						if (pos > 0)
						{
							pos--;
							obst->P[k] = str2num(buff, 3 + 16 * pos, 14);
						}
						obst->code[k][0] = head->GNSSTypeRead[sys_id][k][1];
						obst->code[k][1] = 'X';

						// S
						pos = head->GPSObsPosX[NFREQ * 3 + k];
						if (pos > 0)
						{
							pos--;
							obst->S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
						}
					}


					// P, 如果没有值或者X通道还为0,则用原始通道
					if (obst->P[k] == 0.0)
					{
						pos = head->GNSSObsPos[sys_id][k];
						if (pos > 0)
						{
							pos--;
							obst->P[k] = str2num(buff, 3 + 16 * pos, 14);
						}
						obst->code[k][0] = head->GNSSTypeRead[sys_id][k][1];
						obst->code[k][1] = head->GNSSTypeRead[sys_id][k][2];
					}

					// S
					if (obst->S[k] == 0.0)
					{
						pos = head->GNSSObsPos[sys_id][NFREQ * 3 + k];
						if (pos > 0)
						{
							pos--;
							obst->S[k] = (float)str2num(buff, 3 + 16 * pos, 14);
						}
					}

					// D
					pos = head->GNSSObsPos[sys_id][NFREQ * 2 + k];
					if (pos > 0)
					{
						pos--;
						obst->D[k] = str2num(buff, 3 + 16 * pos, 14);
					}
				}
			}

			obst->prn = prn;
			nsatValid++;
		}

		dst->nsat = nsatValid;
		dst->flag = flag;

		SortOBS(dst);

		break;
	}

	return true;
}


bool readAllEphHead(FILE* m_fp, CEphemerisComputer* EphStores)
{
	char buff[MAXSIZE] = { '\0' };
	char* label = buff + 60;

	while (fgets(buff, MAXSIZE, m_fp))
	{
		if (feof(m_fp)) break;

		if (strstr(label, "IONOSPHERIC CORR"))
		{
			//if (!strncmp(buff, "GPSA", 4))
			//{
			//	for (i = 0, j = 5; i < 4; i++, j += 12) m_GPSION[i] = str2num(buff, j, 12);
			//}
			//else if (!strncmp(buff, "GPSB", 4))
			//{
			//	for (i = 0, j = 5; i < 4; i++, j += 12) m_GPSION[4 + i] = str2num(buff, j, 12);
			//}
			//else if (!strncmp(buff, "GAL", 3))
			//{
			//	for (i = 0, j = 5; i < 4; i++, j += 12) m_GALION[i] = str2num(buff, j, 12);
			//}
			//else if (!strncmp(buff, "BDSA", 4))
			//{
			//	for (i = 0, j = 5; i < 4; i++, j += 12) m_BD2ION[i] = m_BD3ION[i] = str2num(buff, j, 12);
			//}
			//else if (!strncmp(buff, "BDSB", 4))
			//{
			//	for (i = 0, j = 5; i < 4; i++, j += 12) m_BD2ION[i + 4] = m_BD3ION[i + 4] = str2num(buff, j, 12);
			//}
		}
		else if (strstr(label, "TIME SYSTEM CORR"))
		{
			//if (!strncmp(buff, "GPUT", 4))
			//{
			//	m_GPSUTC[0] = str2num(buff, 5, 17);
			//	m_GPSUTC[1] = str2num(buff, 22, 16);
			//	m_GPSUTC[2] = str2num(buff, 38, 7);
			//	m_GPSUTC[3] = str2num(buff, 45, 5);
			//}
			//else if (!strncmp(buff, "GLUT", 4))
			//{
			//	//m_GLOUTC[0] = str2num(buff, 5, 17);
			//	//m_GLOUTC[1] = str2num(buff, 22, 16);
			//}
			//else if (!strncmp(buff, "BDUT", 4))
			//{
			//	m_BD2UTC[0] = m_BD3UTC[0] = str2num(buff, 5, 17);
			//	m_BD2UTC[1] = m_BD3UTC[1] = str2num(buff, 22, 16);
			//	m_BD2UTC[2] = m_BD3UTC[2] = str2num(buff, 38, 7);
			//	m_BD2UTC[3] = m_BD3UTC[3] = str2num(buff, 45, 5);
			//}
			//else if (!strncmp(buff, "GAUT", 4))
			//{
			//	m_GALUTC[0] = str2num(buff, 5, 17);
			//	m_GALUTC[1] = str2num(buff, 22, 16);
			//	m_GALUTC[2] = str2num(buff, 38, 7);
			//	m_GALUTC[3] = str2num(buff, 45, 5);
			//}
			//else if (!strncmp(buff, "QZUT", 4))
			//{
			//	m_QZSUTC[0] = str2num(buff, 5, 17);
			//	m_QZSUTC[1] = str2num(buff, 22, 16);
			//	m_QZSUTC[2] = str2num(buff, 38, 7);
			//	m_QZSUTC[3] = str2num(buff, 45, 5);
			//}
		}
		else if (strstr(label, "END OF HEADER"))
		{
			return true;
		}

	}

	return true;
}


bool readAllEphBody(FILE* m_fp, CEphemerisComputer* EphStores)
{
	//for (int i = 0; i < NSATGPS; i++) EphStores->m_GPSEphDatas[i].clear();
	//for (int i = 0; i < NSATGLO; i++) EphStores->m_GLOEphDatas[i].clear();
	//for (int i = 0; i < NSATBD2; i++) EphStores->m_BD2EphDatas[i].clear();
	//for (int i = 0; i < NSATBD3; i++) EphStores->m_BD3EphDatas[i].clear();
	//for (int i = 0; i < NSATGAL; i++) EphStores->m_GALEphDatas[i].clear();
	//for (int i = 0; i < NSATQZS; i++) EphStores->m_QZSEphDatas[i].clear();

	char buff[MAXSIZE] = { '\0' };
	char ch[MAXSIZE] = { '\0' };
	char* label = buff + 60;

	GPSEPH GPSEph;
	GLOEPH GLOEph;
	GPSEPH BDSEph;
	GPSEPH GALEph;
	GPSEPH QZSEph;

	GPSTIME tof, toc, gt;
	double tow, tod, ttr;
	int week, dow, prn, sat, sys, i, iPos;

	if (!EphStores)return false;

	while (fgets(buff, MAXSIZE, m_fp))
	{
		if (feof(m_fp)) break;

		xstrmid(buff, 0, 3, ch);
		prn = satno2prn1(ch);
		if (prn == 0) continue;

		sat = satprn2no(prn, &sys);

		if (sat == 0 || sys == SYSNON) continue;

		if (sys == SYSGPS && sat <= NSATGPS)
		{
			GPSEph.prn = prn;

			GPSEph.toc = str2time(buff, 4, 19);

			GPSEph.f0 = str2num(buff, 23, 19);
			GPSEph.f1 = str2num(buff, 42, 19);
			GPSEph.f2 = str2num(buff, 61, 19);

			// 轨道1
			fgets(buff, MAXSIZE, m_fp);
			GPSEph.iode = (int)str2num(buff, 4, 19);
			GPSEph.crs = str2num(buff, 23, 19);
			GPSEph.deln = str2num(buff, 42, 19);
			GPSEph.M0 = str2num(buff, 61, 19);

			// 轨道2
			fgets(buff, MAXSIZE, m_fp);
			GPSEph.cuc = str2num(buff, 4, 19);
			GPSEph.e = str2num(buff, 23, 19);
			GPSEph.cus = str2num(buff, 42, 19);
			GPSEph.A = str2num(buff, 61, 19);
			GPSEph.A = GPSEph.A * GPSEph.A;

			// 轨道3
			fgets(buff, MAXSIZE, m_fp);
			GPSEph.toes = str2num(buff, 4, 19);
			GPSEph.cic = str2num(buff, 23, 19);
			GPSEph.OMG0 = str2num(buff, 42, 19);
			GPSEph.cis = str2num(buff, 61, 19);

			// 轨道4
			fgets(buff, MAXSIZE, m_fp);
			GPSEph.i0 = str2num(buff, 4, 19);
			GPSEph.crc = str2num(buff, 23, 19);
			GPSEph.omg = str2num(buff, 42, 19);
			GPSEph.OMGd = str2num(buff, 61, 19);

			// 轨道5
			fgets(buff, MAXSIZE, m_fp);
			GPSEph.idot = str2num(buff, 4, 19);
			GPSEph.code = (int)str2num(buff, 23, 19);
			GPSEph.week = (int)str2num(buff, 42, 19);
			//GPSEph.flag = (int)str2num(buff, 61, 19);

			// 轨道6
			fgets(buff, MAXSIZE, m_fp);
			int index = URA2Index(str2num(buff, 4, 19));
			GPSEph.sva = Index2URA(index);
			GPSEph.svh = (int)str2num(buff, 23, 19);
			GPSEph.tgd[0] = str2num(buff, 42, 19);
			GPSEph.iodc = (int)str2num(buff, 61, 19);

			// 轨道7
			fgets(buff, MAXSIZE, m_fp);
			ttr = str2num(buff, 4, 19);
			//GPSEph.fit = str2num(buff, 23, 19);

			// 时间，调整week
			InitGPSTIME2((int)GPSEph.week, GPSEph.toes, &gt);
			GPSEph.toe = adjweek(gt, GPSEph.toc);
			//GPSEph.ttr = adjweek(GPSTIME(GPSEph.week, ttr), GPSEph.toc);

			// 有空位就放，否则将第一个去掉
			iPos = 0;
			for (i = 0; i < MAXEPHNUM; i++)
			{
				if (EphStores->m_GPSEphDatas[sat - 1][i].toe.GPSWeek < 10)
				{
					iPos = i;
					break;
				}
			}
			EphStores->m_GPSEphDatas[sat - 1][iPos] = GPSEph;
			EphStores->m_bNavEph[ISYSGPS] = true;
		}

		//else if (sys == SYSGLO && sat <= NSATGLO)
		//{
		//	GLOEph.prn = prn;

		//	toc = str2time(buff, 4, 19);
		//	tow = _TOW(toc);
		//	week = toc.GPSWeek;
		//	toc = GPSTIME(week, floor((tow + 450.0) / 900.0) * 900);
		//	dow = (int)floor(tow / 86400.0);


		//	GLOEph.taun = -1.0 * str2num(buff, 23, 19);       /* -taun */
		//	GLOEph.gamn = str2num(buff, 42, 19);       /* +gamman */
		//	tod = str2num(buff, 61, 19);     /* tod in utc */

		//	tof = GPSTIME(week, tod + dow * 86400.0);
		//	tof = adjday(tof, toc);

		//	GLOEph.toe = utc2gpst(toc);   /* toc (gpst) */
		//	GLOEph.tof = utc2gpst(tof);   /* tof (gpst) */

		//	/* iode = tb (7bit), tb =index of UTC+3H within current day */
		//	GLOEph.iode = (int)(fmod(tow + 10800.0, 86400.0) / 900.0 + 0.5);

		//	 轨道1
		//	fgets(buff, MAXSIZE, m_fp);
		//	GLOEph.pos[0] = 1E3 * str2num(buff, 4, 19);
		//	GLOEph.vel[0] = 1E3 * str2num(buff, 23, 19);
		//	GLOEph.acc[0] = 1E3 * str2num(buff, 42, 19);
		//	GLOEph.svh = (int)str2num(buff, 61, 19);

		//	 轨道2
		//	fgets(buff, MAXSIZE, m_fp);
		//	GLOEph.pos[1] = 1E3 * str2num(buff, 4, 19);
		//	GLOEph.vel[1] = 1E3 * str2num(buff, 23, 19);
		//	GLOEph.acc[1] = 1E3 * str2num(buff, 42, 19);
		//	GLOEph.frq = (int)str2num(buff, 61, 19);

		//	 轨道3
		//	fgets(buff, MAXSIZE, m_fp);
		//	GLOEph.pos[2] = 1E3 * str2num(buff, 4, 19);
		//	GLOEph.vel[2] = 1E3 * str2num(buff, 23, 19);
		//	GLOEph.acc[2] = 1E3 * str2num(buff, 42, 19);
		//	GLOEph.age = (int)str2num(buff, 61, 19);

		//	/* some receiver output >128 for minus frequency number */
		//	if (GLOEph.frq > 128) GLOEph.frq -= 256;

		//	if (GLOEph.frq < -7 || 13 < GLOEph.frq)
		//	{
		//		char msg[1024] = { '\0' };
		//		sprintf(msg, "GLONASS invalid iFreq: sat=%2d fn=%d", GLOEph.prn, GLOEph.frq);
		//		DumpException(msg, BUGT_INFO, DUMP_LOCATION);
		//	}
		//	else
		//	{
		//		m_GLO_iFreq[sat] = GLOEph.frq;

		//		if (m_GLO_iFreq[sat] != gs_GLO_iFreq[sat])
		//		{
		//			char msg[1024] = { '\0' };
		//			sprintf(msg, "GLONASS Change iFreq: sat=%2d fn=%d->%d", GLOEph.prn, gs_GLO_iFreq[sat], GLOEph.frq);
		//			DumpException(msg, BUGT_INFO, DUMP_LOCATION);
		//		}
		//	}

		//	m_GLOEphDatas[sat - 1].push_back(GLOEph);
		//	m_bNavEph[ISYSGLO] = true;
		//}

		else if (sys == SYSBD2 && sat <= NSATBD2)
		{
			BDSEph.prn = prn;

			BDSEph.toc = str2time(buff, 4, 19);
			BDSEph.toc = BDST2GPST(BDSEph.toc, true); // 转成GPS时

			BDSEph.f0 = str2num(buff, 23, 19);
			BDSEph.f1 = str2num(buff, 42, 19);
			BDSEph.f2 = str2num(buff, 61, 19);

			// 轨道1
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.iode = (int)str2num(buff, 4, 19);
			BDSEph.crs = str2num(buff, 23, 19);
			BDSEph.deln = str2num(buff, 42, 19);
			BDSEph.M0 = str2num(buff, 61, 19);

			// 轨道2
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.cuc = str2num(buff, 4, 19);
			BDSEph.e = str2num(buff, 23, 19);
			BDSEph.cus = str2num(buff, 42, 19);
			BDSEph.A = str2num(buff, 61, 19);
			BDSEph.A = BDSEph.A * BDSEph.A;

			// 轨道3
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.toes = str2num(buff, 4, 19);
			BDSEph.cic = str2num(buff, 23, 19);
			BDSEph.OMG0 = str2num(buff, 42, 19);
			BDSEph.cis = str2num(buff, 61, 19);

			// 轨道4
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.i0 = str2num(buff, 4, 19);
			BDSEph.crc = str2num(buff, 23, 19);
			BDSEph.omg = str2num(buff, 42, 19);
			BDSEph.OMGd = str2num(buff, 61, 19);

			// 轨道5
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.idot = str2num(buff, 4, 19);
			// 
			BDSEph.week = (int)str2num(buff, 42, 19); // 3.02 定义了 BD周
			//

			if (BDSEph.toc.GPSWeek - BDSEph.week > 1000)
			{
				BDSEph.week += 1356; // 转换为GPS周
			}

			// 轨道6
			fgets(buff, MAXSIZE, m_fp);
			int index = URA2Index(str2num(buff, 4, 19));
			BDSEph.sva = Index2URA(index);
			BDSEph.svh = (int)str2num(buff, 23, 19);
			BDSEph.tgd[0] = str2num(buff, 42, 19);
			BDSEph.tgd[1] = str2num(buff, 61, 19);

			// 轨道7
			fgets(buff, MAXSIZE, m_fp);
			ttr = str2num(buff, 4, 19);
			BDSEph.iodc = (int)str2num(buff, 23, 19);

			// 时间，调整week
			InitGPSTIME2((int)BDSEph.week, BDSEph.toes, &gt);
			BDSEph.toe = adjweek(gt, BDSEph.toc);
			// 转换成GPS时
			BDSEph.toe = BDST2GPST(BDSEph.toe, true);
			//BDSEph.ttr = bdst2gpst(BDSEph.ttr);

			// 有空位就放，否则将第一个去掉
			iPos = 0;
			for (i = 0; i < MAXEPHNUM; i++)
			{
				if (EphStores->m_BD2EphDatas[sat - 1][i].toe.GPSWeek < 10)
				{
					iPos = i;
					break;
				}
			}
			EphStores->m_BD2EphDatas[sat - 1][iPos] = BDSEph;
			EphStores->m_bNavEph[ISYSBD2] = true;
		}

		else if (sys == SYSBD3 && sat <= NSATBD3)
		{
			BDSEph.prn = prn;

			BDSEph.toc = str2time(buff, 4, 19);
			BDSEph.toc = BDST2GPST(BDSEph.toc, true); // 转成GPS时

			BDSEph.f0 = str2num(buff, 23, 19);
			BDSEph.f1 = str2num(buff, 42, 19);
			BDSEph.f2 = str2num(buff, 61, 19);

			// 轨道1
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.iode = (int)str2num(buff, 4, 19);
			BDSEph.crs = str2num(buff, 23, 19);
			BDSEph.deln = str2num(buff, 42, 19);
			BDSEph.M0 = str2num(buff, 61, 19);

			// 轨道2
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.cuc = str2num(buff, 4, 19);
			BDSEph.e = str2num(buff, 23, 19);
			BDSEph.cus = str2num(buff, 42, 19);
			BDSEph.A = str2num(buff, 61, 19);
			BDSEph.A = BDSEph.A * BDSEph.A;

			// 轨道3
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.toes = str2num(buff, 4, 19);
			BDSEph.cic = str2num(buff, 23, 19);
			BDSEph.OMG0 = str2num(buff, 42, 19);
			BDSEph.cis = str2num(buff, 61, 19);

			// 轨道4
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.i0 = str2num(buff, 4, 19);
			BDSEph.crc = str2num(buff, 23, 19);
			BDSEph.omg = str2num(buff, 42, 19);
			BDSEph.OMGd = str2num(buff, 61, 19);

			// 轨道5
			fgets(buff, MAXSIZE, m_fp);
			BDSEph.idot = str2num(buff, 4, 19);
			// 
			BDSEph.week = (int)str2num(buff, 42, 19); // 3.02 定义了 BD周
			//

			if (BDSEph.toc.GPSWeek - BDSEph.week > 1000)
			{
				BDSEph.week += 1356; // 转换为GPS周
			}

			// 轨道6
			fgets(buff, MAXSIZE, m_fp);
			int index = URA2Index(str2num(buff, 4, 19));
			BDSEph.sva = Index2URA(index);
			BDSEph.svh = (int)str2num(buff, 23, 19);
			BDSEph.tgd[0] = str2num(buff, 42, 19);
			BDSEph.tgd[1] = str2num(buff, 61, 19);

			// 轨道7
			fgets(buff, MAXSIZE, m_fp);
			ttr = str2num(buff, 4, 19);
			BDSEph.iodc = (int)str2num(buff, 23, 19);

			// 时间，调整week
			InitGPSTIME2((int)BDSEph.week, BDSEph.toes, &gt);
			BDSEph.toe = adjweek(gt, BDSEph.toc);
			//BDSEph.ttr = adjweek(GPSTIME(BDSEph.week, ttr), BDSEph.toc);

			// 转换成GPS时
			BDSEph.toe = BDST2GPST(BDSEph.toe, true);
			//BDSEph.ttr = bdst2gpst(BDSEph.ttr);

			// 有空位就放，否则将第一个去掉
			iPos = 0;
			for (i = 0; i < MAXEPHNUM; i++)
			{
				if (EphStores->m_BD3EphDatas[sat - 1][i].toe.GPSWeek < 10)
				{
					iPos = i;
					break;
				}
			}
			EphStores->m_BD3EphDatas[sat - 1][iPos] = BDSEph;
			EphStores->m_bNavEph[ISYSBD3] = true;
		}

		else if (sys == SYSGAL && sat <= NSATGAL)
		{
			// GAL时间与GPS在秒上没有固定偏差

			GALEph.prn = prn;

			GALEph.toc = str2time(buff, 4, 19);

			GALEph.f0 = str2num(buff, 23, 19);
			GALEph.f1 = str2num(buff, 42, 19);
			GALEph.f2 = str2num(buff, 61, 19);

			// 轨道1
			fgets(buff, MAXSIZE, m_fp);
			GALEph.iode = (int)str2num(buff, 4, 19);
			GALEph.crs = str2num(buff, 23, 19);
			GALEph.deln = str2num(buff, 42, 19);
			GALEph.M0 = str2num(buff, 61, 19);

			// 轨道2
			fgets(buff, MAXSIZE, m_fp);
			GALEph.cuc = str2num(buff, 4, 19);
			GALEph.e = str2num(buff, 23, 19);
			GALEph.cus = str2num(buff, 42, 19);
			GALEph.A = str2num(buff, 61, 19);
			GALEph.A = GALEph.A * GALEph.A;

			// 轨道3
			fgets(buff, MAXSIZE, m_fp);
			GALEph.toes = str2num(buff, 4, 19);
			GALEph.cic = str2num(buff, 23, 19);
			GALEph.OMG0 = str2num(buff, 42, 19);
			GALEph.cis = str2num(buff, 61, 19);

			// 轨道4
			fgets(buff, MAXSIZE, m_fp);
			GALEph.i0 = str2num(buff, 4, 19);
			GALEph.crc = str2num(buff, 23, 19);
			GALEph.omg = str2num(buff, 42, 19);
			GALEph.OMGd = str2num(buff, 61, 19);

			// 轨道5
			fgets(buff, MAXSIZE, m_fp);
			GALEph.idot = str2num(buff, 4, 19);
			GALEph.code = (int)str2num(buff, 23, 19);
			/* bit 0 set: I/NAV E1-B */
			/* bit 1 set: F/NAV E5a-I */
			/* bit 2 set: F/NAV E5b-I */
			/* bit 8 set: af0-af2 toc are for E5a.E1 */
			/* bit 9 set: af0-af2 toc are for E5b.E1 */
			GALEph.week = (int)str2num(buff, 42, 19); // 当前定义的RINEX文件中的周为GPS周, 非GAL周

			// 轨道6
			fgets(buff, MAXSIZE, m_fp);
			int index = URA2Index(str2num(buff, 4, 19));
			GALEph.sva = Index2URA(index);
			GALEph.svh = (int)str2num(buff, 23, 19);
			/* bit     0: E1B DVS */
			/* bit   1-2: E1B HS */
			/* bit     3: E5a DVS */
			/* bit   4-5: E5a HS */
			/* bit     6: E5b DVS */
			/* bit   7-8: E5b HS */
			GALEph.tgd[0] = str2num(buff, 42, 19);  /* BGD E5a/E1 */
			GALEph.tgd[1] = str2num(buff, 61, 19);  /* BGD E5b/E1 */

			// 轨道7
			fgets(buff, MAXSIZE, m_fp);
			ttr = str2num(buff, 4, 19);

			// 时间，调整week
			InitGPSTIME2((int)GALEph.week, GALEph.toes, &gt);
			GALEph.toe = adjweek(gt, GALEph.toc);
			//GALEph.ttr = adjweek(GPSTIME(GALEph.week, ttr), GALEph.toc);

			// 有空位就放，否则将第一个去掉
			iPos = 0;
			for (i = 0; i < MAXEPHNUM; i++)
			{
				if (EphStores->m_GALEphDatas[sat - 1][i].toe.GPSWeek < 10)
				{
					iPos = i;
					break;
				}
			}
			EphStores->m_GALEphDatas[sat - 1][iPos] = GALEph;
			EphStores->m_bNavEph[ISYSGAL] = true;
		}

		if (sys == SYSQZS && sat <= NSATQZS)
		{
			QZSEph.prn = prn;

			QZSEph.toc = str2time(buff, 4, 19);

			QZSEph.f0 = str2num(buff, 23, 19);
			QZSEph.f1 = str2num(buff, 42, 19);
			QZSEph.f2 = str2num(buff, 61, 19);

			// 轨道1
			fgets(buff, MAXSIZE, m_fp);
			QZSEph.iode = (int)str2num(buff, 4, 19);
			QZSEph.crs = str2num(buff, 23, 19);
			QZSEph.deln = str2num(buff, 42, 19);
			QZSEph.M0 = str2num(buff, 61, 19);

			// 轨道2
			fgets(buff, MAXSIZE, m_fp);
			QZSEph.cuc = str2num(buff, 4, 19);
			QZSEph.e = str2num(buff, 23, 19);
			QZSEph.cus = str2num(buff, 42, 19);
			QZSEph.A = str2num(buff, 61, 19);
			QZSEph.A = QZSEph.A * QZSEph.A;

			// 轨道3
			fgets(buff, MAXSIZE, m_fp);
			QZSEph.toes = str2num(buff, 4, 19);
			QZSEph.cic = str2num(buff, 23, 19);
			QZSEph.OMG0 = str2num(buff, 42, 19);
			QZSEph.cis = str2num(buff, 61, 19);

			// 轨道4
			fgets(buff, MAXSIZE, m_fp);
			QZSEph.i0 = str2num(buff, 4, 19);
			QZSEph.crc = str2num(buff, 23, 19);
			QZSEph.omg = str2num(buff, 42, 19);
			QZSEph.OMGd = str2num(buff, 61, 19);

			// 轨道5
			fgets(buff, MAXSIZE, m_fp);
			QZSEph.idot = str2num(buff, 4, 19);
			QZSEph.code = (int)str2num(buff, 23, 19);
			QZSEph.week = (int)str2num(buff, 42, 19);
			//QZSEph.flag = (int)str2num(buff, 61, 19);

			// 轨道6
			fgets(buff, MAXSIZE, m_fp);
			int index = URA2Index(str2num(buff, 4, 19));
			QZSEph.sva = Index2URA(index);
			QZSEph.svh = (int)str2num(buff, 23, 19);
			QZSEph.tgd[0] = str2num(buff, 42, 19);
			QZSEph.iodc = (int)str2num(buff, 61, 19);

			// 轨道7
			fgets(buff, MAXSIZE, m_fp);
			ttr = str2num(buff, 4, 19);
			//QZSEph.fit = str2num(buff, 23, 19);

			// 时间，调整week
			InitGPSTIME2((int)QZSEph.week, QZSEph.toes, &gt);
			QZSEph.toe = adjweek(gt, QZSEph.toc);
			//QZSEph.ttr = adjweek(GPSTIME(QZSEph.week, ttr), QZSEph.toc);

			// 有空位就放，否则将第一个去掉
			iPos = 0;
			for (i = 0; i < MAXEPHNUM; i++)
			{
				if (EphStores->m_QZSEphDatas[sat - 1][i].toe.GPSWeek < 10)
				{
					iPos = i;
					break;
				}
			}
			EphStores->m_QZSEphDatas[sat - 1][iPos] = QZSEph;
			EphStores->m_bNavEph[ISYSQZS] = true;
		}
	}

	return true;
}