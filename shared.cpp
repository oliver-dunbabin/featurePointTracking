#include "shared.h"

shared::shared()
{
    ptrvState   = new vehicleState;
    ptrfpHarris = new camMessage;
    ptrfpStates = new fpDatabase;
}


shared::~shared()
{
    delete ptrvState;
    delete ptrfpHarris;
    delete ptrfpStates;
}


void shared::setCamMsg(camMessage *fpMeas)
{
    *ptrfpHarris = *fpMeas;
}


void shared::setVehicleState(vehicleState *vState)
{
    *ptrvState = *vState;
}
