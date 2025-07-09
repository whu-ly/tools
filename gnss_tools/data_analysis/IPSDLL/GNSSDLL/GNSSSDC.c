#include "GNSSSDC.h"


///< Configure frequencies
char gs_strGPSFrq[NFREQ][5] = { "L1"  , "L2"  , "L5" };
char gs_strGLOFrq[NFREQ][5] = { ""  , ""  , "" };
char gs_strBD2Frq[NFREQ][5] = { "B1I" , "B2I" , "B3I" };
char gs_strBD3Frq[NFREQ][5] = { "B1I" , "B1C" , "B2a" };
char gs_strGALFrq[NFREQ][5] = { "E1"  , "E5b" , "E5a" };
char gs_strQZSFrq[NFREQ][5] = { ""  , ""  , "" };


void InitLOGDATA(LOG_DATA* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(LOG_DATA));
}


void InitOBSDATAT_PL(OBS_DATA_t_PL* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(OBS_DATA_t_PL));
}


void InitOBSDATAT(OBS_DATA_t* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(OBS_DATA_t));
}


void InitOBSDATA(OBS_DATA* pStruct)
{
	if (!pStruct)return;

	InitGPSTIME(&pStruct->gt);
	pStruct->flag = -1;
	pStruct->nsat = 0;
	memset(pStruct->ngnss, 0, sizeof(int) * NSYS);
	memset(pStruct->obs, 0, sizeof(OBS_DATA_t) * MAXOBS);
}


void InitGPSEPH(GPSEPH* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(GPSEPH));
	pStruct->toe.GPSWeek = pStruct->toc.GPSWeek = -1;
}


void InitGLOEPH(GLOEPH* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(GLOEPH));
	pStruct->toe.GPSWeek = pStruct->tof.GPSWeek = -1;
}


void InitAMBC(AMBC* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(AMBC));
	for (int f = 0; f < NFREQ; f++) pStruct->gt[f].GPSWeek = -1;
}


void InitSATSTATUS(SATSTATUS* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(SATSTATUS));
	pStruct->sys_id = ISYSNON;
	for (int f = 0; f < NFREQ; f++) pStruct->CS_GFTime[f].GPSWeek = -1;
	for (int f = 0; f < NFREQ; f++)pStruct->DD_L[f] = 0.0;//³õÊ¼»¯Ë«²î²Ð²î
	InitAMBC(&pStruct->AR_AmbCtrl);
}


void InitGNSS_MEQ(GNSS_MEQ* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(GNSS_MEQ));
}


void InitGNSS_GLSINFO(GNSS_GLSINFO* pStruct)
{
	if (!pStruct)return;

	memset(pStruct, 0, sizeof(GNSS_GLSINFO));
	SetValsI(pStruct->m_AmbIndex, -1, NSATMAX * NFREQ);
}