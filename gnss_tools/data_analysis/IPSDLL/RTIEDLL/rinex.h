#ifndef RTIETK_RINEX_H
#define RTIETK_RINEX_H

#include "GNSSSDC.h"
#include "CEphemerisComputer.h"

typedef struct tagRINEX_HEAD
{
    double PhaseShiftCorr[NSYS][NFREQ];
    double PhaseShiftCorrX[NFREQ];
    int GNSSObsPos[NSYS][4 * NFREQ]; // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
    double XYZPos[3];

    // GPS L2X 位置, 有的GPS卫星,L2W和L2X是交叉的,即有些卫星是L2W,有些卫星是L2X
    int GPSObsPosX[4 * NFREQ]; // P1,P2,P3,L1,L2,L5,D1,D2,D5,S1,S2,S5
    char GNSSTypeRead[NSYS][4 * NFREQ][10];
    int nGNSSTypeRead[NSYS];

}RINEX_HEAD;
void InitRINEX_HEAD(RINEX_HEAD* pStruct);

bool RinexHeadRead(FILE* ifp, RINEX_HEAD* head);

bool RinexBodyRead(FILE* ifp, RINEX_HEAD* head, OBS_DATA* dst);

bool readAllEphHead(FILE* m_fp, CEphemerisComputer* EphStores);

bool readAllEphBody(FILE* m_fp, CEphemerisComputer* EphStores);


#endif