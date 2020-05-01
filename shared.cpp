#include "shared.h"
#include <math.h>

shared::shared(int filterType)
{
    ptrvState   = new vehicleState;
    ptrfpHarris = new camMessage;
    ptrfpStates = new fpDatabase;
    ptrfpData   = new fpDatalink;
    if (filterType){
        filter_type = UKF;
    }else{
        filter_type = EKF;
    }
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
