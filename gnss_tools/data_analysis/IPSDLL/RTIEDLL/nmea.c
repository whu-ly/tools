#include "rtklib.h"
#include <stdio.h>
#include <string.h>

#define BUFFSIZE        256
#define Freq0           -47122.395833492279 //Hz
#define Freq1           -40526.315789699554 //Hz
#define FREQ_CHOICE(Freq_Bit) (Freq_Bit==0 ? (Freq0) : (Freq1))
#define P2_34           5.820766091346740E-11 /* 2^-34 */
#define P2_46           1.421085471520200E-14 /* 2^-46 */
#define P2_59           1.734723475976810E-18 /* 2^-59 */

typedef enum
{
    GPSL1_SAT_ID_START      = 1,				//signal channel: L1
    GPSL1_SAT_ID_END        = 32,
    GPSL2_SAT_ID_START      = 401,              //signal channel: L2C
    GPSL2_SAT_ID_END        = 432,
    GPSL3_SAT_ID_START      = 501,				//signal channel: L5
    GPSL3_SAT_ID_END        = 532,

	GALL1_SAT_ID_START		= 301,				//signal channel: E1
	GALL1_SAT_ID_END		= 327,
	GALL2_SAT_ID_START		= 651,				//signal channel: E5b
	GALL2_SAT_ID_END		= 677,
	GALL3_SAT_ID_START		= 601,				//signal channel: E5a
	GALL3_SAT_ID_END		= 627,

    BDL1_SAT_ID_START1      = 140 + 1,          //signal channel: B1I
    BDL1_SAT_ID_END1        = 140 + 37,
    BDL1_SAT_ID_START2      = 140 + 800 + 38,
    BDL1_SAT_ID_END2        = 140 + 800 + 46,
    BDL2_SAT_ID_START       = 540 + 1,          //signal channel: B2I
    BDL2_SAT_ID_END         = 540 + 37,
    BDL3_SAT_ID_START       = 850 + 1,          //signal channel: B2a
    BDL3_SAT_ID_END         = 850 + 46
}Sat_ID;

typedef struct
{
    int week_n;
    double tow;
    int sats_tracked;
    unsigned int cpu_time;
    unsigned int time_validity;
    double clock_drift;
    int kf_config;
    int const_mask;
    int time_best_sat_type;
    int time_master_sat_type;
    int time_master_week_n;
    double time_master_tow;
    int time_master_validity;
    int flags_aux;
    double clock_bias_mt;
    unsigned int mfreq_constellation_mask;
    int leap_sec;
    int pps_edge;
    int mtb_ms;
    int mtb_timestamp;
    int checksum;

    int TG_TS_version;
    int TG_TS_front_end;
    int clock_steering;
} TG_data_t;

typedef struct
{
    int dsp_avail;
    int sat_id;
    double PseudoRange;
    double Doppler;
    double CarrierPhase;
    unsigned int dsp_flags_32;//LLI
    int CN0;
    int TrackedTime;
    int code_noise;
    int ave_phase_noise;
    int cycle_slip_cnt;
    int glonass_slot;
    int elevation_deg;
    int checksum;
    int dsp_available;
} TS_data_t;

//iFreq:1=L1.2=L2,3=L5
static char get_sys_prn(int sat_id, int *prn,int* iFreq)
{
     if((sat_id >= GPSL1_SAT_ID_START) && (sat_id <= GPSL1_SAT_ID_END))
     {
         *prn=sat_id;
         *iFreq=1;
         return SYS_GPS;
     }
     else if((sat_id >= GPSL2_SAT_ID_START) && (sat_id <= GPSL2_SAT_ID_END))
     {
         *prn=sat_id-400;
         *iFreq=2;
         return SYS_GPS;
     }
     else if((sat_id >= GPSL3_SAT_ID_START) && (sat_id <= GPSL3_SAT_ID_END))
     {
         *prn=sat_id-500;
         *iFreq=3;
         return SYS_GPS;
     }
	 else if ((sat_id >= GALL1_SAT_ID_START) && (sat_id <= GALL1_SAT_ID_END))
	 {
		 *prn = sat_id - 300;
		 *iFreq = 1;
		 return SYS_GAL;
	 }
	 else if ((sat_id >= GALL2_SAT_ID_START) && (sat_id <= GALL2_SAT_ID_END))
	 {
		 *prn = sat_id - 650;
		 *iFreq = 2;
		 return SYS_GAL;
	 }
	 else if ((sat_id >= GALL3_SAT_ID_START) && (sat_id <= GALL3_SAT_ID_END))
	 {
		 *prn = sat_id - 600;
		 *iFreq = 3;
		 return SYS_GAL;
	 }
     else if((sat_id >= BDL1_SAT_ID_START1) && (sat_id <= BDL1_SAT_ID_END1))
     {
         *prn=sat_id-140;
         *iFreq=1;
         return SYS_CMP;
     }
     else if((sat_id >= BDL1_SAT_ID_START2) && (sat_id <= BDL1_SAT_ID_END2))
     {
         *prn=sat_id-140-800;
         *iFreq=1;
         return SYS_CMP;
     }
     else if((sat_id >= BDL2_SAT_ID_START) && (sat_id <= BDL2_SAT_ID_END))
     {
         *prn=sat_id-540;
         *iFreq=2;
         return SYS_CMP;
     }
     else if((sat_id >= BDL3_SAT_ID_START) && (sat_id <= BDL3_SAT_ID_END))
     {
         *prn=sat_id-850;
         *iFreq=3;
         return SYS_CMP;
     }

     *prn = 0;
     *iFreq=0;
     return 0;
}

//iFreq:1=L1.2=L2,3=L5
static char get_code(int sys,int iFreq)
{
    if(sys==SYS_GPS)
    {
        if(iFreq==1)      return CODE_L1C;			/* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
        else if(iFreq==2) return CODE_L2C;		/* obs code: L2C/A,G1C/A (GPS,GLO) */
        else if(iFreq==3) return CODE_L5I;		/* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
    }
    else if(sys==SYS_GLO)
    {
        if(iFreq==1) return CODE_L1C;
        else if(iFreq==2) return CODE_L2C;
        else if(iFreq==3) return CODE_L3I;
    }
    else if(sys==SYS_CMP)
    {
        if(iFreq==1) return CODE_L2I;			/* obs code: B1I        (CMP) */
        else if(iFreq==2) return CODE_L7I;		/* obs code: E5bI,B2I   (GAL,CMP) */
        else if(iFreq==3) return CODE_L5D;		/* obs code: L5/E5aI    (GPS,GAL,QZS,SBS,CMP) */
    }
    else if(sys==SYS_GAL)
    {
        if(iFreq==1) return CODE_L1C;			/* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
        else if(iFreq==2) return CODE_L7I;		/* obs code: E5bI,B2I   (GAL,CMP) */
        else if(iFreq==3) return CODE_L5I;		/* obs code: L5/E5aI    (GPS,GAL,QZS,SBS,CMP) */
    }
    else if(sys==SYS_QZS)
    {
        if(iFreq==1) return CODE_L1C;
        else if(iFreq==2) return CODE_L2S;
        else if(iFreq==3) return CODE_L5I;
    }

    return 0;
}

static char getOneLine(char* line, char* buff)
{
	int i = 0;
	while (1)
	{
		if (buff[i] == '\0')	return 0;
		if (buff[i] == '\r')	return 1;
		if (buff[i] == '\n')	return 1;
		line[i] = buff[i];
		i++;
	}
	return 0;
}

static char findHead(char *line, char * buff, char *word)
{
    int j = 0;
    for(int i = 0; buff[i]!='\0'; i++)
    {
        if(strncmp(buff+i, word, strlen(word))==0)
        {
            for(j = 0; buff[i+j]!='\r'&&buff[i+j]!='\n'&&buff[i+j]!='\0'; j++)
            {
                line[j] = buff[i+j];
            }
            i = i+j;
            return 1;
        }
    }
    return 0;
}

static unsigned int getbyteu(char* pos,int len)
{
    if(len>=0&&len<=4)      //防止内存溢出
    {
        unsigned int a=0;
        memcpy(&a,pos,len);
        return a;
    }
    else return 0;
}

static int getbytes(char* pos,int len)
{
    unsigned int a=getbyteu(pos,len);

    if(len<=0||len>=4||!(a&(1u<<(len*8-1)))) return (int)a;
    return (int)(a|(~0u<<len*8)); /* extend sign */
}

static char decode_TGDATA(char *line, TG_data_t *tg_data)
{
    int num_fields_tg;
    if(strncmp(line, "$PSTMTG,", strlen("$PSTMTG,"))) return 0;

    num_fields_tg = sscanf(line, "$PSTMTG,%d,%lf,%d,%u,%d,%lf,%x,",
    &tg_data->week_n, &tg_data->tow, &tg_data->sats_tracked,
    &tg_data->cpu_time, &tg_data->time_validity, &tg_data->clock_drift, &tg_data->kf_config);

    if(num_fields_tg != 7) return 0;

    tg_data->TG_TS_version = (tg_data->kf_config >> 12) & 0xF;
    tg_data->TG_TS_front_end = (tg_data->kf_config >> 8) & 0xF;
    tg_data->clock_steering = (tg_data->TG_TS_version & 0x8) ? 1:0;
    tg_data->TG_TS_version &= 0x7; // remove steering bit from version

    if (tg_data->TG_TS_version != 2)
    {
        printf("TG/TS version not supported : %d\n", tg_data->TG_TS_version);
        return 0;
    }
    else // (tg_data->TG_TS_version == 2)
    {
        num_fields_tg = sscanf(line,
            "$PSTMTG,%d,%lf,%d,%d,%d,%lf,%x,%u,%u,%d,%d,%lf,%d,%d,%lf,%d,%d,%u,%d,%d*%x",
            &tg_data->week_n,
            &tg_data->tow,
            &tg_data->sats_tracked,
            &tg_data->cpu_time,
            &tg_data->time_validity,
            &tg_data->clock_drift,
            &tg_data->kf_config,
            &tg_data->const_mask,
            &tg_data->time_best_sat_type,
            &tg_data->time_master_sat_type,
            &tg_data->time_master_week_n,
            &tg_data->time_master_tow,
            &tg_data->time_master_validity,
            &tg_data->flags_aux,
            &tg_data->clock_bias_mt,
            &tg_data->mfreq_constellation_mask,
            &tg_data->leap_sec,
            &tg_data->pps_edge,
            &tg_data->mtb_ms,
            &tg_data->mtb_timestamp,
            &tg_data->checksum
            );
    }

    return 1;
}

static char decode_TSDATA(char *line, TS_data_t *ts_data)
{
    int num_fields,dsp_avail,sat_id,sys,prn,iFreq;;
    if(strncmp(line, "$PSTMTS,", strlen("$PSTMTS,")))  return 0;

    num_fields = sscanf(line, "$PSTMTS,%d,%d", &dsp_avail, &sat_id);
    if(!(sys=get_sys_prn(sat_id,&prn,&iFreq))) return 0;
    if((!prn)||(num_fields != 2))              return 0;

    num_fields = sscanf(line,
       "$PSTMTS,%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d*%X",
        &ts_data->dsp_available,
        &ts_data->sat_id,
        &ts_data->PseudoRange,
        &ts_data->Doppler,
        &ts_data->CarrierPhase,
        &ts_data->dsp_flags_32,
        &ts_data->CN0,
        &ts_data->TrackedTime,
        &ts_data->code_noise,
        &ts_data->ave_phase_noise,
        &ts_data->cycle_slip_cnt,
        &ts_data->glonass_slot,
        &ts_data->elevation_deg,
        &ts_data->checksum);

    return ts_data->dsp_available;
}

static char decode_GPSEph(nav_t* nav,unsigned char* line)
{
    if(!line||!nav) return 0;            //判断指针不为空
	if (strlen(line) >= BUFFSIZE) return 0;
    int i,sat_id,len,sys,prn,sat,iFreq;
    char hex_buff[BUFFSIZE],payload[BUFFSIZE];

    sscanf(line, "$PSTMEPHEM,%d,%d,%s", &sat_id, &len, hex_buff);

    for (i = 0; i < len; i++) sscanf(&hex_buff[i * 2], "%02x", &payload[i]);

    if(!(sys=get_sys_prn(sat_id,&prn,&iFreq))) return 0;
    if((sat=satno(sys,prn))>0)//星历赋值
    {
        double  dtemp,toe,toc;
        char    ctemp[10];
        i=0;

        nav->eph[sat-1].sat     =sat;

        //week,toe
        nav->eph[sat-1].week    =getbyteu(payload+i,2);                 i+=2;
        toe                     =getbyteu(payload+i,2)*16.0;            i+=2;

        //toc,iode1,iode2
        toc                     =getbyteu(payload+i,2)*16.0;            i+=2;
        nav->eph[sat-1].iode    =getbyteu(payload+i,1);                 i+=1;
        i+=1;

        //iodc,i_dot,spare1
        ctemp[0]=payload[i+2];ctemp[1]=payload[i+1];ctemp[2]=payload[i];i+=3;
        nav->eph[sat-1].iodc    =getbitu(ctemp,14,10);
        nav->eph[sat-1].idot    =getbits(ctemp,0,14)*P2_43*SC2RAD;
        i+=1;

        //omega_dot,reserved1,reserved2
        nav->eph[sat-1].OMGd    =getbytes(payload+i,3)*P2_43*SC2RAD;    i+=3;
        i+=1;

        //crs,crc
        nav->eph[sat-1].crs     =getbytes(payload+i,2)*P2_5;            i+=2;
        nav->eph[sat-1].crc     =getbytes(payload+i,2)*P2_5;            i+=2;

        //cus,cuc
        nav->eph[sat-1].cus     =getbytes(payload+i,2)*P2_29;           i+=2;
        nav->eph[sat-1].cuc     =getbytes(payload+i,2)*P2_29;           i+=2;

        //cis,cic
        nav->eph[sat-1].cis     =getbytes(payload+i,2)*P2_29;           i+=2;
        nav->eph[sat-1].cic     =getbytes(payload+i,2)*P2_29;           i+=2;

        //motion_difference,age_h,12_codes,spare3
        nav->eph[sat-1].deln    =getbytes(payload+i,2)*P2_43*SC2RAD;    i+=2;
        i+=2;

        //inclination
        nav->eph[sat-1].i0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //eccentricity
        nav->eph[sat-1].e       =getbyteu(payload+i,4)*P2_33;           i+=4;

        //root_a
        dtemp                   =getbyteu(payload+i,4)*P2_19;           i+=4;
        nav->eph[sat-1].A       =dtemp*dtemp;

        //mean_anomaly
        nav->eph[sat-1].M0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //omega_zero
        nav->eph[sat-1].OMG0    =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //perogee
        nav->eph[sat-1].omg     =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //time_group_delay,af2,af1
        nav->eph[sat-1].tgd[0]  =getbytes(payload+i,1)*P2_31;           i+=1;
        nav->eph[sat-1].f2      =getbytes(payload+i,1)*P2_55;           i+=1;
        nav->eph[sat-1].f1      =getbytes(payload+i,2)*P2_43;           i+=2;

        //af0,subframe1_available,subframe2_available,subframe3_available,available,health,predicted,accuracy
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];i+=4;
        nav->eph[sat-1].f0      =getbits(ctemp,10,22)*P2_31;
        nav->eph[sat-1].svh     =getbitu(ctemp,5,1);
        nav->eph[sat-1].sva     =getbitu(ctemp,0,4);

        nav->eph[sat-1].toe     =gpst2time(nav->eph[sat-1].week,toe);
        nav->eph[sat-1].toc     =gpst2time(nav->eph[sat-1].week,toc);
        nav->eph[sat-1].toes    =toe;
        nav->eph[sat-1].sva     =2;         //sva解码不对，先手动赋一个值

        return 1;
    }
    return 0;
}

static char decode_BDSEph(nav_t* nav,unsigned char* line)
{
    if(!line||!nav) return 0;            //判断指针不为空
    int i,sat_id,len,sys,prn,sat,iFreq;
    char hex_buff[2*BUFFSIZE],payload[BUFFSIZE];

    sscanf(line, "$PSTMEPHEM,%d,%d,%s", &sat_id, &len, hex_buff);

    for (i = 0; i < len; i++) sscanf(&hex_buff[i * 2], "%02x", &payload[i]);

    if(!(sys=get_sys_prn(sat_id,&prn,&iFreq))) return 0;
    if((sat=satno(sys,prn))>0)//星历赋值
    {
        double  dtemp,toe,toc;
        char    ctemp[10];
        i=0;

        nav->eph[sat-1].sat     =sat;

        //inclination
        nav->eph[sat-1].i0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //eccentricity
        nav->eph[sat-1].e       =getbyteu(payload+i,4)*P2_33;           i+=4;

        //root_a
        dtemp                   =getbyteu(payload+i,4)*P2_19;           i+=4;
        nav->eph[sat-1].A       =dtemp*dtemp;

        //mean_anomaly
        nav->eph[sat-1].M0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //omega_zero
        nav->eph[sat-1].OMG0    =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //perigee
        nav->eph[sat-1].omg     =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //toe,time_group_delay,aode 17+10+5
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        toe                     =getbitu(ctemp,15,17)*8.0;
        nav->eph[sat-1].tgd[0]  =getbits(ctemp,5,10)*1E-10;
        nav->eph[sat-1].iode    =getbitu(ctemp,0,5);
        i+=4;

        //omega_dot,A0 24+8
        nav->eph[sat-1].OMGd    =getbytes(payload+i,3)*P2_43*SC2RAD;
        i+=4;

        //af0,A1 24+8
        nav->eph[sat-1].f0      =getbytes(payload+i,3)*P2_33;
        i+=4;

        //sow,af2,is_geo 20+11+1
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].f2      =getbits(ctemp,1,11)*P2_33*P2_33;
        i+=4;

        //af1,subframe_avail 22+10
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].f1      =getbits(ctemp,10,22)*P2_50;
        i+=4;

        //motion_difference,A2,A3 16+8+8
        nav->eph[sat-1].deln    =getbytes(payload+i,2)*P2_43*SC2RAD;
        i+=4;

        //crs,B2,urai,reserved1 18+8+4+2
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].crs     =getbits(ctemp,14,18)*P2_6;
        nav->eph[sat-1].sva     =getbitu(ctemp,2,4);
        i+=4;

        //crc,B3,aodc,spare0 18+8+5+1
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].crc     =getbits(ctemp,14,18)*P2_6;
        nav->eph[sat-1].iodc    =getbitu(ctemp,1,5);
        i+=4;

        //cus,i_dot 18+14
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].cus     =getbits(ctemp,14,18)*P2_31;
        nav->eph[sat-1].idot    =getbits(ctemp,0,14)*P2_43*SC2RAD;
        i+=4;

        //cuc,B0,age_h 18+8+6
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].cuc     =getbits(ctemp,14,18)*P2_31;
        i+=4;

        //cis,B1,time_distance_h 18+8+6
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].cis     =getbits(ctemp,14,18)*P2_31;
        i+=4;

        //cic,nvm_reliable,predicted,tgd2,spare4 18+1+1+10+2
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].cic     =getbits(ctemp,14,18)*P2_31;
        nav->eph[sat-1].tgd[1]  =getbits(ctemp,2,10)*1E-10;
        i+=4;

        //toc,week,available,health 17+13+1+1
        ctemp[0]=payload[i+3];ctemp[1]=payload[i+2];ctemp[2]=payload[i+1];ctemp[3]=payload[i];
        nav->eph[sat-1].week    =getbitu(ctemp,2,13);
        toc                     =getbitu(ctemp,15,17)*8.0;
        nav->eph[sat-1].svh     =getbitu(ctemp,0,1);
        i+=4;

        nav->eph[sat-1].toe     =bdt2gpst(bdt2time(nav->eph[sat-1].week,toe)); /* bdt -> gpst */
        nav->eph[sat-1].toc     =bdt2gpst(bdt2time(nav->eph[sat-1].week,toc)); /* bdt -> gpst */
        nav->eph[sat-1].toes    =toe;
        nav->eph[sat-1].sva     =2;         //sva解码不对，先手动赋一个值

//        //输出调试信息
//        dtemp=0.0;
//        printf("C%02d %.12E %.12E %.12E\n",prn,nav->eph[sat-1].f0,nav->eph[sat-1].f1,nav->eph[sat-1].f2);
//        printf("%.12E %.12E %.12E %.12E\n",(double)nav->eph[sat-1].iode,nav->eph[sat-1].crs,nav->eph[sat-1].deln,nav->eph[sat-1].M0);
//        printf("%.12E %.12E %.12E %.12E\n",nav->eph[sat-1].cuc,nav->eph[sat-1].e,nav->eph[sat-1].cus,nav->eph[sat-1].A);
//        printf("%.12E %.12E %.12E %.12E\n",toe,nav->eph[sat-1].cic,nav->eph[sat-1].OMG0,nav->eph[sat-1].cis);
//        printf("%.12E %.12E %.12E %.12E\n",nav->eph[sat-1].i0,nav->eph[sat-1].crc,nav->eph[sat-1].omg,nav->eph[sat-1].OMGd);
//        printf("%.12E %.12E %.12E %.12E\n",nav->eph[sat-1].idot,dtemp,(double)nav->eph[sat-1].week,dtemp);
//        printf("%.12E %.12E %.12E %.12E\n",(double)nav->eph[sat-1].sva,dtemp,nav->eph[sat-1].tgd[0],nav->eph[sat-1].tgd[1]);
//        printf("%.12E %.12E\n",toc,(double)nav->eph[sat-1].iodc);

        return 1;
    }
    return 0;
}

static char decode_GALEph(nav_t* nav,unsigned char* line)
{
    if(!line||!nav) return 0;            //判断指针不为空
    int i,sat_id,len,sys,prn,sat,iFreq;
    char hex_buff[BUFFSIZE],payload[BUFFSIZE];

    sscanf(line, "$PSTMEPHEM,%d,%d,%s", &sat_id, &len, hex_buff);

    for (i = 0; i < len; i++) sscanf(&hex_buff[i * 2], "%02x", &payload[i]);

    if(!(sys=get_sys_prn(sat_id,&prn,&iFreq))) return 0;
    if((sat=satno(sys,prn))>0)//星历赋值
    {
        double  dtemp,toe,toc;
        char    ctemp[10];
        i=0;

        nav->eph[sat-1].sat     =sat;

        //1.week,toe,ephems_n
        nav->eph[sat-1].week    =getbyteu(payload+i,2);                 i+=2;
        ctemp[0]=payload[i+1];  ctemp[1]=payload[i];                    i+=2;
        toe                     =getbitu(ctemp,2,14)*60.0;

        //2.toc,iod_nav,SISA
        ctemp[0]=payload[i+2];  ctemp[1]=payload[i+1];
        ctemp[2]=payload[i];                                            i+=3;
        toc                     =getbitu(ctemp,10,14)*60.0;
        nav->eph[sat-1].iode    =getbitu(ctemp,0,10);
        i+=1;

        //3.age_h,BGD_E1_E5a,BGD_E1_E5b,E1BHS
        ctemp[0]=payload[i+3];  ctemp[1]=payload[i+2];
        ctemp[2]=payload[i+1];  ctemp[3]=payload[i];                    i+=4;
        nav->eph[sat-1].tgd[0]  =getbits(ctemp,12,10)*P2_32;
        nav->eph[sat-1].tgd[1]  =getbits(ctemp,2,10)*P2_32;

        //4.inclination
        nav->eph[sat-1].i0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //5.eccentricity
        nav->eph[sat-1].e       =getbyteu(payload+i,4)*P2_33;           i+=4;

        //6.root_a
        dtemp                   =getbyteu(payload+i,4)*P2_19;           i+=4;
        nav->eph[sat-1].A       =dtemp*dtemp;

        //7.mean_anomaly
        nav->eph[sat-1].M0      =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //8.omega_zero
        nav->eph[sat-1].OMG0    =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //9.perigee
        nav->eph[sat-1].omg     =getbytes(payload+i,4)*P2_31*SC2RAD;    i+=4;

        //10.i_dot,available,health,motion_difference
        ctemp[0]=payload[i+1];  ctemp[1]=payload[i];                    i+=2;
        nav->eph[sat-1].idot    =getbits(ctemp,2,14)*P2_43*SC2RAD;
        nav->eph[sat-1].sva     =getbitu(ctemp,1,1);
        nav->eph[sat-1].svh     =getbitu(ctemp,0,1);
        nav->eph[sat-1].deln    =getbytes(payload+i,2)*P2_43*SC2RAD;    i+=2;

        //11.crs,crc
        nav->eph[sat-1].crs     =getbytes(payload+i,2)*P2_5;            i+=2;
        nav->eph[sat-1].crc     =getbytes(payload+i,2)*P2_5;            i+=2;

        //12.cus,cuc
        nav->eph[sat-1].cus     =getbytes(payload+i,2)*P2_29;           i+=2;
        nav->eph[sat-1].cuc     =getbytes(payload+i,2)*P2_29;           i+=2;

        //13.cis,cic
        nav->eph[sat-1].cis     =getbytes(payload+i,2)*P2_29;           i+=2;
        nav->eph[sat-1].cic     =getbytes(payload+i,2)*P2_29;           i+=2;

        //14.omega_dot,SVID,E1BDVS,predicted
        nav->eph[sat-1].OMGd    =getbytes(payload+i,3)*P2_43*SC2RAD;    i+=3;
        i+=1;

        //15.af2,af1,word_available
        ctemp[0]=payload[i+3];  ctemp[1]=payload[i+2];
        ctemp[2]=payload[i+1];  ctemp[3]=payload[i];                    i+=4;
        nav->eph[sat-1].f2      =getbits(ctemp,26,6)*P2_59;
        nav->eph[sat-1].f1      =getbits(ctemp,5,21)*P2_46;

        //16.af0,spare0
        ctemp[0]=payload[i+3];  ctemp[1]=payload[i+2];
        ctemp[2]=payload[i+1];  ctemp[3]=payload[i];                    i+=4;
        nav->eph[sat-1].f0      =getbits(ctemp,1,31)*P2_34;

        //17.time_distance_h,spare1


        nav->eph[sat-1].toe     =gpst2time(nav->eph[sat-1].week,toe);
        nav->eph[sat-1].toc     =gpst2time(nav->eph[sat-1].week,toc);
        nav->eph[sat-1].toes    =toe;
        nav->eph[sat-1].sva     =2;         //sva解码不对，先手动赋一个值

        return 1;
    }
    return 0;
}

static char decode_PSTMEPHEM(nav_t* nav, unsigned char* line)
{
    int sat_id,sys,prn,iFreq;

    sscanf(line, "$PSTMEPHEM,%d", &sat_id);
    if(!(sys=get_sys_prn(sat_id,&prn,&iFreq))) return 0;

    switch(sys)
    {
        case SYS_GPS: {decode_GPSEph(nav,line);break;}
        case SYS_CMP: {decode_BDSEph(nav,line);break;}
        case SYS_GAL: {decode_GALEph(nav,line);break;}
        default:      {break;}
    }

    return 1;
}

static char decode_PSTMTS(obs_t *obs, TG_data_t tg, unsigned char* line)
{
	TS_data_t ts;
	int sys, prn, iFreq;
	int nMeas = obs->n;

    if (!decode_TSDATA((char *)line, &ts))              return 0;	// 数据解码,flag:2=L1,3=L1+L2,6=L1+L5,7=L1+L2+L5
    if (!(sys = get_sys_prn(ts.sat_id, &prn, &iFreq)))  return 0;

	if (iFreq == 1) memset(&(obs->data[nMeas]), '\0', sizeof(obsd_t));//数据清0
	//else return 0;

	//对观测值按照系统和prn进行排序
	for (int i = 0; i < nMeas; i++)
	{
		if (satno(sys, prn) < obs->data[i].sat)//如果prn较小
		{
			for (int j = nMeas; j > i; j--)//后面的prn后移，空出第i个
			{
				obs->data[j] = obs->data[j - 1];
			}
			memset(&(obs->data[i]), '\0', sizeof(obsd_t));//数据清0
			nMeas = i;
			break;
		}
		else if (satno(sys, prn) == obs->data[i].sat)//如果是非单频数据
		{
			nMeas = i;
			break;
		}
	}

	obs->data[nMeas].sat = satno(sys, prn);
	obs->data[nMeas].time = gpst2time(tg.week_n, tg.tow);//观测时间赋值
	obs->data[nMeas].code[iFreq - 1] = get_code(sys, iFreq);
	obs->data[nMeas].P[iFreq - 1] = ts.PseudoRange;
	obs->data[nMeas].L[iFreq - 1] = -ts.CarrierPhase;
    obs->data[nMeas].SNR[iFreq - 1] = ts.CN0 / SNR_UNIT + 0.5;
	obs->data[nMeas].LLI[iFreq - 1] = ((ts.dsp_flags_32) & (1UL << 4)) >> 4;

	double BFre[3] = { 1561098000.0,1207140000.0,1176450000.0 };	// B1I，B2I，B2a
	double LFre[3] = { 1575420000.0,1227600000.0,1176450000.0 };	// L1 C/A，L2 C/A，L5
	if (tg.clock_steering)
	{
		// 这里北斗可能还有问题，文档里说当 clock steering 被激活时，clock drift 可以作为 clock(GPS) 即 FREQ_CHOICE()返回值的精确值，
		// 但是 clock steering 激活与否的北斗多普勒计算公式不一致，不应该都是 -(ts.Doppler + clock(GPS)) 吗，edit by ly-0305
		// 经验证，GPS/BDS/GAL 第一频点多普勒解码正确，第三频点多普勒解码错误，edit by ly-0306
		if (sys == SYS_CMP)        obs->data[nMeas].D[iFreq - 1] = ts.Doppler - tg.clock_drift * BFre[0] / LFre[0];
		else if (sys == SYS_GPS || sys == SYS_GAL)   obs->data[nMeas].D[iFreq - 1] = ts.Doppler - tg.clock_drift;
	}
	else
	{
        if (sys == SYS_CMP)        obs->data[nMeas].D[iFreq - 1] = - ts.Doppler - FREQ_CHOICE(tg.TG_TS_front_end)*BFre[0] / LFre[0];
		else if (sys == SYS_GPS || sys == SYS_GAL)   obs->data[nMeas].D[iFreq - 1] = ts.Doppler - FREQ_CHOICE(tg.TG_TS_front_end);
		//GLO/GAL等系统待考虑
	}

	if (iFreq == 1) obs->n++;
	return 1;
}

static int check(unsigned char* line, int len)
{
    int i;
    char check_h = 0, check_s[10] = { 0 };

    if (len < 5) return 0;

    for (i = 1; i < len - 3; i++)
    {
        check_h ^= line[i];
    }
    sprintf(check_s, "%X", check_h); // sprintf带结束符

    if (strlen(check_s) == 1 && line[len - 2] == '0' && check_s[0] == line[len - 1]) return 1;
    if (strlen(check_s) == 2 && check_s[0] == line[len - 2] && check_s[1] == line[len - 1]) return 1;

    return 0;
}

extern int decodeNMEA(char *buff, rtcm_t *rtcm)
{
    TG_data_t tg;
    char line[2048]={'\0'};

    if (!buff || !rtcm)return 0;

    //PSTMTG
    if(findHead(line, buff, "$PSTMTG"))
    {
        int length = 1; // '\r'
        buff += (strlen(line) + length);//用实际读取长度右移
        decode_TGDATA(line, &tg);//读取时间等信息
        rtcm->obs.n=0;
        memset(line, '\0', sizeof(line));//用总长度清0

        while(getOneLine(line,buff))
        {
            buff += (strlen(line) + length);

            if (check(line, strlen(line)))
            {
                if (strncmp(line, "$PSTMTS,", strlen("$PSTMTS,")) == 0)
                    decode_PSTMTS(&(rtcm->obs), tg, line);
                else if (strncmp(line, "$PSTMEPHEM,", strlen("$PSTMEPHEM,")) == 0)
                    decode_PSTMEPHEM(&(rtcm->nav), line);
            }

            memset(line,'\0',sizeof(line));
        }

        return 1;
    }
    return 0;
}
