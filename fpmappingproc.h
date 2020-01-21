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

void updateMappingFP(shared *data);

void updateFPmeas(shared *data);

void pointCorrespondance(shared *data);

void quat2dcm321(double quat[4], double dcm[3][3]);

void map_vector( double T[3][3], double vin[3], double vout[3] );

bool sortDescending(const std::pair<int,int> &a,const std::pair<int,int> &b);

void updateFPcov(double dt, shared *data);

void ComputeCinvD(int j, double hcam[], double focallength[], double Cfp[NUMFPMEAS][NUMFPSTATES], shared *data);

void doCorrespondance(bool sortConf, shared *data);

#endif // FPMAPPINGPROC_H
