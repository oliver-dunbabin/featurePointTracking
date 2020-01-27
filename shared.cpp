#include "shared.h"

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
