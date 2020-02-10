#include "shared.h"
#include <math.h>

shared::shared()
{
    ptrvState   = new vehicleState;
    ptrfpHarris = new camMessage;
    ptrfpStates = new fpDatabase;
    ptrfpData   = new fpDatalink;
    filter_type = EKF;
}


shared::~shared()
{
    delete ptrvState;
    delete ptrfpHarris;
    delete ptrfpStates;
    delete ptrfpData;
}


void shared::setCamMsg(camMessage *fpMeas)
{
    *ptrfpHarris = *fpMeas;
}


void shared::setVehicleState(vehicleState *vState)
{
    *ptrvState = *vState;
}

void quat2dcm321(double qEst[4], double DCM[3][3])
{
    DCM[0][0] = qEst[0]*qEst[0] + qEst[1]*qEst[1] - qEst[2]*qEst[2] - qEst[3]*qEst[3];
    DCM[0][1] = 2*(qEst[1]*qEst[2] + qEst[0]*qEst[3]);
    DCM[0][2] = 2*(qEst[1]*qEst[3] - qEst[0]*qEst[2]);
    DCM[1][0] = 2*(qEst[1]*qEst[2] - qEst[0]*qEst[3]);
    DCM[1][1] = qEst[0]*qEst[0] - qEst[1]*qEst[1] + qEst[2]*qEst[2] - qEst[3]*qEst[3];
    DCM[1][2] = 2*(qEst[2]*qEst[3] + qEst[0]*qEst[1]);
    DCM[2][0] = 2*(qEst[1]*qEst[3] + qEst[0]*qEst[2]);
    DCM[2][1] = 2*(qEst[2]*qEst[3] - qEst[0]*qEst[1]);
    DCM[2][2] = qEst[0]*qEst[0] - qEst[1]*qEst[1] - qEst[2]*qEst[2] + qEst[3]*qEst[3];
}

void map_vector( double T[3][3], double vin[3], double vout[3] ) {

  int i, j;

  for( i=0; i<3; i++ ) {
    vout[i] = 0.0;
    for( j=0; j<3; j++ )
      vout[i] += T[i][j]*vin[j];
  }

}


void quatNorm(double q[4])
{
    double qmag = sqrt( q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if ( qmag > 0.01 && qmag < 100.0) {
        qmag = 1./qmag;
        for ( int i = 0; i < 4; i++ ) q[i] *= qmag;
    }
    else{
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
    }
}

void quat_mult( double a[4], double b[4], double c[4])
{

    c[0] = b[0]*a[0] - b[1]*a[1] - b[2]*a[2] - b[3]*a[3];
    c[1] = b[0]*a[1] + b[1]*a[0] - b[2]*a[3] + b[3]*a[2];
    c[2] = b[0]*a[2] + b[1]*a[3] + b[2]*a[0] - b[3]*a[1];
    c[3] = b[0]*a[3] - b[1]*a[2] + b[2]*a[1] + b[3]*a[0];

}
