#ifndef FPMAPPINGPROC_H
#define FPMAPPINGPROC_H

#include <string>
#include "shared.h"

class sort_fpEst
{
private:
    int *mparr;
public:
    sort_fpEst(int *parr) : mparr(parr){}
    bool operator()(int i, int j) const {return mparr[i] > mparr[j];}
};

void initDataStructs(shared *data);

void updateMappingFP(std::istringstream &tokenMsg, shared &data);

bool updateMappingFP(shared *data);

void updateFPmeas(shared *data);

void updatefpDatalink(shared *data);

void pointCorrespondance(shared *data);

void quat2dcm321(double quat[4], double dcm[3][3]);

void map_vector( const double T[3][3], const double vin[3], double vout[3] );

bool sortDescending(const std::pair<int,int> &a,const std::pair<int,int> &b);

void updateFPcov(double dt, shared *data);

void ComputeCinvD(int j, double hcam[], double focallength[], double Cfp[NUMFPMEAS][NUMFPSTATES], shared *data);

void findSigmaPoints(int i, fpDatabase *fpEst, double SigPoints[2*NUMFPSTATES][NUMFPSTATES]);

void CholeskyDecomposition(double P[NUMFPSTATES][NUMFPSTATES], double U[NUMFPSTATES][NUMFPSTATES]);

void doCorrespondance(bool sortConf, shared *data);

vehicleState time_delay_get(bool populate, uint64_t time, double dt, vehicleState *buffer, int bufferSize);

void time_delay_fill(bool populate, vehicleState &element, vehicleState *buffer, int bufferSize);

vehicleState interpolate_Vstate(double frac, const vehicleState& V1, const vehicleState& V2);

#endif // FPMAPPINGPROC_H
