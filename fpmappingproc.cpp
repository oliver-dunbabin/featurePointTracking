#include "fpmappingproc.h"
#include "matrix.h"
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <string.h>
#include <vector>

bool sortDescending(const std::pair<int,int> &a,const std::pair<int,int> &b)
{
    return (a.first > b.first);
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


void initDataStructs(shared *data)
{
    fpDatabase *fpEst   = data->getFpStates();
    camMessage *fpMeas  = data->getCamMsg();
    if(!fpEst->dbInit){
        fpEst->confThresh = 50;
        fpEst->dbInit = false;
        fpEst->newFpID = -1;

        for (int i = 0; i<fpMeas->NUMFPS; i++)
            fpMeas->assign_fp[i] = -1;

        for (int i = 0; i<NUMFPMEAS; i++)
            data->fpR[i][i] = SIGMAFPMEAS;
        double fpQdiag[NUMFPSTATES] = {0, 0, 0, SQ(0), SQ(0), SQ(0)};
        mat_diag((double *)data->fpQ, NUMFPSTATES, NUMFPSTATES, fpQdiag);
    }
}


bool updateMappingFP(shared *data)
{
    vehicleState *vEst = data->getVState();
    // Update process model
    data->fpTimeF = data->timer.now();
    std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::duration<double>>(data->fpTimeF-data->fpTimeI);
    updateFPcov(dt.count(), data);
    data->fpTimeI = data->timer.now();

    // Update measurement model
    if((data->gotCAMmsg) && (vEst != nullptr)){

        doCorrespondance(false, data);
        updateFPmeas(data);
        updatefpDatalink(data);
        return true;
    }
    return false;
}


void updatefpDatalink(shared *data)
{
    fpDatabase *fpEst   = data->getFpStates();
    fpDatalink *fpData  = data->getFpData();

    fpData->numCorresponded = fpEst->numCorresponded;
    fpData->confThreash     = fpEst->confThresh;
    for(int i = 0; i < DBSIZE; i++){
        fpData->confidence[i]   = fpEst->confidence[i];
        fpData->fpID[i]         = fpEst->fpID[i];
        for(int j = 0; j < NUMFPSTATES; j++){
            fpData->state[i][j] = fpEst->state[i][j];
            for(int k = 0; k < NUMFPSTATES; k++){
                fpData->fpP[i][j][k] = fpEst->fpP[i][j][k];
            }
        }
    }
}


void updateFPmeas(shared *data)
{
    fpDatabase *fpEst = data->getFpStates();
    camMessage *fpMeas = data->getCamMsg();
    vehicleState *vehicleEst = data->getVState();
    double vEst[3] = {vehicleEst->pos.X, vehicleEst->pos.Y, vehicleEst->pos.Z};
    double qEst[4] = {vehicleEst->quat.Q0, vehicleEst->quat.Q1, vehicleEst->quat.Q2, vehicleEst->quat.Q3};

    // DCM from inertial frame to Camera
    double Lc2i[3][3], Li2c[3][3];

    quat2dcm321(qEst, data->Li2c);
    for(int t = 0; t < 3; t++){
        for(int r = 0; r < 3; r++)
            Li2c[t][r] = data->Li2c[t][r];
    }
    //std::copy(&data->Li2c[0][0], &data->Li2c[0][0]+3*3, &Li2c[0][0]);
    mat_transpose((double *)Li2c, 3, 3, (double *)Lc2i);

    double focallengthx = (double)IMAGESIZEH/(IMAGESIZEH*tan(FOVH/2.)); //1/tan(FOVH/2.);
    double focallengthy = (double)IMAGESIZEV/(IMAGESIZEH*tan(FOVV/2.)); //1/tan(FOVV/2.);

    int i;
    for (i=0;i<fpMeas->NUMFPS;i++){
        if (fpMeas->assign_fp[i] < 0){
            for (int j = DBSIZE-1; j >= 0; j--){ // Smallest confidence had in the back elements of DB
                if(fpEst->confidence[j] < fpEst->confThresh){   /* If fp in database falls below some confidence threshold,
                                                                   it is candidate for replacement by uncorrelated measurement */
                    fpEst->newFpID++;
                    // feature point state given as:    y = [xa, ya, za, psi, theta, rho]'
                    // Set anchor position (inertial) to estimated vehicle location (inertial)
                    fpEst->state[j][0]  = vehicleEst->pos.X;
                    fpEst->state[j][1]  = vehicleEst->pos.Y;
                    fpEst->state[j][2]  = vehicleEst->pos.Z;
                    // Initialise inverse depth as 0.1 (Civera et. al. 2008)
                    fpEst->state[j][5]  = RHO;

                    // Use measurement model to determine elevation (theta) and azimuth (psi)
                    //          where:  (hx, hy, hz) = is the ray from anchor point to fp in camera frame (frd)
                    //
                    double hx = 1/(sqrt(1 + SQ((fpMeas->fpLocNorm[i][0])/focallengthx) + SQ((fpMeas->fpLocNorm[i][1])/focallengthy)));
                    double hy = (fpMeas->fpLocNorm[i][0])/(focallengthx*sqrt(1 + SQ((fpMeas->fpLocNorm[i][0])/focallengthx) + SQ((fpMeas->fpLocNorm[i][1])/focallengthy)));
                    double hz = (fpMeas->fpLocNorm[i][1])/(focallengthy*sqrt(1 + SQ((fpMeas->fpLocNorm[i][0])/focallengthx) + SQ((fpMeas->fpLocNorm[i][1])/focallengthy)));
                    double cam2fpCAM[] = {hx,hy,hz};
                    double cam2fpIN[3];
                    map_vector(Lc2i,cam2fpCAM,cam2fpIN);

                    fpEst->state[j][3]  = atan2(cam2fpIN[1],cam2fpIN[0]); /* psi */
                    fpEst->state[j][4]  = atan2(-cam2fpIN[2],sqrt(SQ(cam2fpIN[0])+SQ(cam2fpIN[1]))); /* theta */ //asin(-1.*cam2fpIN[2]); //

                   // Initialise state estimate covariance matrix for new fp
                    for(int k = 0; k < 3; k++){
                        fpEst->fpP[j][k][k] = SQ(SIGMAPOS);
                    }
                    fpEst->fpP[j][3][3] = SQ(SIGMAFPANG);
                    fpEst->fpP[j][4][4] = SQ(SIGMAFPANG);
                    fpEst->fpP[j][5][5] = SQ(SIGMAFPINVD);

                    fpEst->confidence[j] = fpEst->confThresh;
                    fpEst->fpID[j] = fpEst->newFpID;
                    break;
                }
            }
        } else{
            int n = fpMeas->assign_fp[i];
            switch (data->filter_type) {
            case UKF:
            {
                double SigPoints[NUMFPSTATES][2*NUMFPSTATES];
                std::vector<std::vector<double>> SigMeas;
                std::vector<std::vector<double>> sigPoints;
                double predMeas[2] = {0};
                int skip = 0;
                findSigmaPoints(n, fpEst, SigPoints);

                // Push each sigma point through measurement equation
                for(int k = 0; k < 2*NUMFPSTATES; k++){
                    double hcam[3], hin[3];
                    double psi   = SigPoints[3][k];
                    double theta = SigPoints[4][k];
                    double rho   = SigPoints[5][k];
                    double px    = SigPoints[0][k];
                    double py    = SigPoints[1][k];
                    double pz    = SigPoints[2][k];
                    double m[3];
                    double stheta   = sin(theta);
                    double ctheta   = cos(theta);
                    double spsi     = sin(psi);
                    double cpsi     = cos(psi);

                    // Unit vector from anchor point to feature point (local coordinate system)
                    m[0] =  cpsi*ctheta;    //cos(psi)*cos(theta);
                    m[1] =  spsi*ctheta;    //sin(psi)*cos(theta);
                    m[2] = -stheta;         //-sin(theta);
                    // Unit vector from vehicle location to feature point (local coordinate system)
                    hin[0] = rho*(px - vehicleEst->pos.X) + m[0];     // change from m/rho to (p - vEst)*rho
                    hin[1] = rho*(py - vehicleEst->pos.Y) + m[1];     // change from m/rho to (p - vEst)*rho
                    hin[2] = rho*(pz - vehicleEst->pos.Z) + m[2];     // change from m/rho to (p - vEst)*rho
                    // Unit vector from vehicle location to feature point (camera frame)
                    hcam[0] = Li2c[0][0]*hin[0] + Li2c[0][1]*hin[1] + Li2c[0][2]*hin[2];
                    hcam[1] = Li2c[1][0]*hin[0] + Li2c[1][1]*hin[1] + Li2c[1][2]*hin[2];
                    hcam[2] = Li2c[2][0]*hin[0] + Li2c[2][1]*hin[1] + Li2c[2][2]*hin[2];

                    // Make sure not finding singularity
                    if (hcam[0] > 0.01){
                        // Compute estimated pixel location of fp
                        std::vector<double> meas{ (focallengthx*hcam[1]/hcam[0]), (focallengthy*hcam[2]/hcam[0]) };
                        std::vector<double> state{ px, py, pz, psi, theta, rho };
                        SigMeas.push_back(meas);
                        sigPoints.push_back(state);
                        predMeas[0] += meas.at(0);
                        predMeas[1] += meas.at(1);
                    }
                }
                // Get mean of these sigma measurement results to give predicted measurement
                double l = SigMeas.size();
                predMeas[0] /= l;
                predMeas[1] /= l;


                if ((predMeas[0] > -1) && (predMeas[0] < 1) &&            // ensure predicted fp location lies on the camera sensor
                    (predMeas[1] > -(double)IMAGESIZEV/IMAGESIZEH) &&
                    (predMeas[1] < (double)IMAGESIZEV/IMAGESIZEH)){

                    // Calculate covariance of predicted measurement and cross
                    // covariance with state estimate
                    double psi   = fpEst->state[n][3];
                    double theta = fpEst->state[n][4];
                    double rho   = fpEst->state[n][5];
                    double px    = fpEst->state[n][0];
                    double py    = fpEst->state[n][1];
                    double pz    = fpEst->state[n][2];
                    double Py[2][2] = {0};
                    double Pxy[NUMFPSTATES][NUMFPSTATES] = {0};
                    for(uint k = 0; k < SigMeas.size(); k++){
                        double E_y[2][2], E_x[NUMFPSTATES][2];
                        double in_y[2] = {(SigMeas[k][0] - predMeas[0]),
                                          (SigMeas[k][1] - predMeas[1])};
                        double in_x[NUMFPSTATES] = {(sigPoints[k][0] - px),
                                                    (sigPoints[k][1] - py),
                                                    (sigPoints[k][2] - pz),
                                                    (sigPoints[k][3] - psi),
                                                    (sigPoints[k][4] - theta),
                                                    (sigPoints[k][5] - rho)};
                        mat_mult_T(in_y, 2, 1, in_y, 2, 1, (double *)E_y);
                        mat_mult_T(in_x, NUMFPSTATES, 1, in_y, 2, 1, (double *)E_x);
                        Py[0][0]    += E_y[0][0]/l; Py[0][1]  += E_y[0][1]/l;
                        Py[1][0]    += E_y[1][0]/l; Py[1][0]  += E_y[1][0]/l;
                        Pxy[0][0]   += E_x[0][0]/l; Pxy[0][1] += E_x[0][1]/l;
                        Pxy[1][0]   += E_x[1][0]/l; Pxy[1][1] += E_x[1][1]/l;
                        Pxy[2][0]   += E_x[2][0]/l; Pxy[2][1] += E_x[2][1]/l;
                        Pxy[3][0]   += E_x[3][0]/l; Pxy[3][1] += E_x[3][1]/l;
                        Pxy[4][0]   += E_x[4][0]/l; Pxy[4][1] += E_x[4][1]/l;
                        Pxy[5][0]   += E_x[5][0]/l; Pxy[5][1] += E_x[5][1]/l;
                    }
                    for (int j = 0; j < NUMFPMEAS; j++){
                        for(int k = 0; k < NUMFPMEAS; k++){
                            Py[j][k] += data->fpR[j][k];
                        }
                    }

                    // Calculate the Kalman gain for the UKF
                    double Kgainfp[NUMFPSTATES][NUMFPMEAS];
                    double Py_inv[2][2];
                    mat_invert((double *)Py, 2, (double *)Py_inv);
                    mat_mult((double *)Pxy, NUMFPSTATES, 2, (double *)Py_inv, 2, 2, (double *)Kgainfp);

                    // Get the residual (difference between measured and predicted measurements
                    double residual[2];
                    residual[0] = fpMeas->fpLocNorm[i][0] - predMeas[0];
                    residual[1] = fpMeas->fpLocNorm[i][1] - predMeas[1];

                    // Compute the new a posteriori state estimate
                    double xcorr[NUMFPSTATES];
                    mat_mult((double *)Kgainfp,NUMFPSTATES,NUMFPMEAS,(double *)residual,2,1,(double *)xcorr);
                    for(int j = 0; j < NUMFPSTATES; j++){
                        fpEst->state[n][j] += xcorr[j];
                    }

                    // Compute the new a posteriori covariance matrix and enforce symmetry
                    double PyKt[2][NUMFPSTATES], KPyKt[NUMFPSTATES][NUMFPSTATES];
                    mat_mult_T((double *)Py, 2, 2, (double *)Kgainfp, NUMFPSTATES, 2, (double *)PyKt);
                    mat_mult((double *)Kgainfp, NUMFPSTATES, 2, (double *)PyKt, 2, NUMFPSTATES, (double *)KPyKt);
                    for (int j = 0; j<NUMFPSTATES;j++){
                        for(int k = 0; k<=j; k++){
                            fpEst->fpP[n][j][k] -= KPyKt[j][k];
                            if(j != k)
                                fpEst->fpP[n][k][j] = fpEst->fpP[n][j][k];
                        }
                    }
                }
                break;
            }
            case EKF:
            default:
            {
                // Find the position vector of fp wrt to camera, hcam (in camera frame)
                double hcam[3], hin[3];
                double psi   = fpEst->state[n][3];
                double theta = fpEst->state[n][4];
                double rho   = fpEst->state[n][5];
                double px    = fpEst->state[n][0];
                double py    = fpEst->state[n][1];
                double pz    = fpEst->state[n][2];
                double m[3];
                double stheta   = sin(theta);
                double ctheta   = cos(theta);
                double spsi     = sin(psi);
                double cpsi     = cos(psi);
                // Unit vector from anchor point to feature point (local coordinate system)
                m[0] =  cpsi*ctheta;    //cos(psi)*cos(theta);
                m[1] =  spsi*ctheta;    //sin(psi)*cos(theta);
                m[2] = -stheta;         //-sin(theta);
                // Unit vector from vehicle location to feature point (local coordinate system)
                hin[0] = rho*(px - vehicleEst->pos.X) + m[0];     // change from m/rho to (p - vEst)*rho
                hin[1] = rho*(py - vehicleEst->pos.Y) + m[1];     // change from m/rho to (p - vEst)*rho
                hin[2] = rho*(pz - vehicleEst->pos.Z) + m[2];     // change from m/rho to (p - vEst)*rho
                // Unit vector from vehicle location to feature point (camera frame)
                hcam[0] = Li2c[0][0]*hin[0] + Li2c[0][1]*hin[1] + Li2c[0][2]*hin[2];
                hcam[1] = Li2c[1][0]*hin[0] + Li2c[1][1]*hin[1] + Li2c[1][2]*hin[2];
                hcam[2] = Li2c[2][0]*hin[0] + Li2c[2][1]*hin[1] + Li2c[2][2]*hin[2];

                // Make sure not finding singularity
                if (hcam[0] > 0.01){
                    // Compute estimated pixel location of fp
                    double fpPixEst[2];
                    fpPixEst[0] = focallengthx*hcam[1]/hcam[0];
                    fpPixEst[1] = focallengthy*hcam[2]/hcam[0];

                    // ensure predicted fp location lies on the camera sensor
                    if ((fpPixEst[0] > -1) && (fpPixEst[0] < 1) &&
                        (fpPixEst[1] > -(double)IMAGESIZEV/IMAGESIZEH) &&
                        (fpPixEst[1] < (double)IMAGESIZEV/IMAGESIZEH)){

                        // Calculate residual (difference between measured and estimated fp location)
                        double residual[2][1];
                        residual[0][0] = fpMeas->fpLocNorm[i][0] - fpPixEst[0];
                        residual[1][0] = fpMeas->fpLocNorm[i][1] - fpPixEst[1];
                        // Populate measurement Jacobian fp matrix, Cfp
                        double Cfp[NUMFPMEAS][NUMFPSTATES];
                        double foclen[2] = {focallengthx, focallengthy};
                        ComputeCinvD(n, hcam, foclen, Cfp, data);

                        // Determine the kalman gain, and update the states/ covariance
                        // build the matrix Rt = (Cfp*fpP*Cfp' + R)
                        double CfpP[NUMFPMEAS][NUMFPSTATES], CfpPCt[NUMFPMEAS][NUMFPMEAS], Rt[NUMFPMEAS][NUMFPMEAS];
                        mat_mult((double *)Cfp, NUMFPMEAS, NUMFPSTATES,(double *)fpEst->fpP[n], NUMFPSTATES,NUMFPSTATES,(double *)CfpP);
                        mat_mult_T((double *)CfpP,NUMFPMEAS,NUMFPSTATES, (double *)Cfp,NUMFPMEAS,NUMFPSTATES, (double *)CfpPCt);

                        int j,k;
                        for (j=0;j<NUMFPMEAS;j++){
                            for(k=0;k<NUMFPMEAS;k++){
                                Rt[j][k] = CfpPCt[j][k] + data->fpR[j][k];
                            }
                        }

                        /* Kalman gain: K = fpP*Cfp'*inv(Cfp*fpP*Cfp' + R)  */
                        double RtInv[NUMFPMEAS][NUMFPMEAS];
                        double Kgainfp[NUMFPSTATES][NUMFPMEAS];
                        mat_invert((double *)Rt,NUMFPMEAS,(double *)RtInv);
                        //mat_mult_T((double *)fpP[n],NUMFPSTATES,NUMFPSTATES,(double *)Cfp,NUMFPMEAS,NUMFPSTATES,(double *)fpPCt);
                        //mat_mult((double *)fpPCt,NUMFPSTATES,NUMFPMEAS,(double *)RtInv,NUMFPMEAS,NUMFPMEAS,(double *)Kgainfp);
                        mat_T_mult((double *)CfpP,NUMFPMEAS,NUMFPSTATES,(double *)RtInv,NUMFPMEAS,NUMFPMEAS,(double *)Kgainfp);

                        // Update the feature point state vector
                        double xcorr[NUMFPSTATES][1];
                        mat_mult((double *)Kgainfp,NUMFPSTATES,NUMFPMEAS,(double *)residual,2,1,(double *)xcorr);
                        for(j=0;j<NUMFPSTATES;j++){
                            fpEst->state[n][j] += xcorr[j][0];
                            //residual[0] -= Cfp[0][j]*xcorr[j];
                            //residual[1] -= Cfp[1][j]*xcorr[j];
                        }


                        // Update lower left half fp covariance matrix: P = P - KCfpP and enforce symmetry
                        double KfpCP[NUMFPSTATES][NUMFPSTATES];
                        mat_mult((double *)Kgainfp,NUMFPSTATES,NUMFPMEAS,(double *)CfpP, NUMFPMEAS,NUMFPSTATES, (double *)KfpCP);
                        for (j = 0; j<NUMFPSTATES;j++){
                            for(k = 0; k<=j; k++){
                                fpEst->fpP[n][j][k] -= KfpCP[j][k];
                                if(j != k){
                                    fpEst->fpP[n][k][j] = fpEst->fpP[n][j][k];
                                }
                            }
                        }
                    }
                }
                break;
            }}
        }
    }
    fpEst->dbInit = true;   // We have filled the DB at least once with sensor measurements
}


void updateFPcov(double dt, shared *data)
{
    fpDatabase *fpEst = data->getFpStates();
    if(fpEst->dbInit){
        // Feature point update
        int i,j;
        for(i=0;i<DBSIZE;i++){
            if(fpEst->fpID[i] >= 0){
                for(j=0;j<NUMFPSTATES;j++){
                    fpEst->fpP[i][j][j] += data->fpQ[j][j]*dt;           // fpQ currently set to zero
                }
            }
        }
    }
}


void doCorrespondance(bool sortConf, shared *data){
    fpDatabase *fpEst = data->getFpStates();

    if (fpEst->dbInit){
        if (sortConf){
            // Sort the database by confidence value (largest to smallest)
            int indices[DBSIZE];
            double tempState[DBSIZE][NUMFPSTATES];
            double tempfpP[DBSIZE][NUMFPSTATES][NUMFPSTATES];
            int tempConf[DBSIZE], tempID[DBSIZE];
            for (int i=0;i<DBSIZE;i++){indices[i] = i;}
            std::sort(indices,indices+DBSIZE, sort_fpEst(fpEst->confidence));
            for (int i = 0; i < DBSIZE; i++){
                tempState[i][0]  = fpEst->state[indices[i]][0];
                tempState[i][1]  = fpEst->state[indices[i]][1];
                tempState[i][2]  = fpEst->state[indices[i]][2];
                tempState[i][3]  = fpEst->state[indices[i]][3];
                tempState[i][4]  = fpEst->state[indices[i]][4];
                tempState[i][5]  = fpEst->state[indices[i]][5];
                tempConf[i] = fpEst->confidence[indices[i]];
                tempID[i] = fpEst->fpID[indices[i]];
                for (int k = 0; k < NUMFPSTATES; k++){
                    for (int j = 0; j < NUMFPSTATES; j++){
                        tempfpP[i][k][j] = fpEst->fpP[indices[i]][k][j];
                    }
                }
            }
            memcpy(&fpEst->state, &tempState[0], sizeof(fpEst->state));
            memcpy(&fpEst->confidence, &tempConf[0], sizeof(fpEst->confidence));
            memcpy(&fpEst->fpID, &tempID[0], sizeof(fpEst->fpID));
            memcpy(&fpEst->fpP, &tempfpP[0], sizeof(fpEst->fpP));
        }

        // Correspondance Test (match measurements with corresponding DB point
        pointCorrespondance(data);

    }else{
        initDataStructs(data);
    }
}


void pointCorrespondance(shared *data)
{
    fpDatabase *fpEst = data->getFpStates();
    camMessage *fpMeas = data->getCamMsg();
    vehicleState *vehicleEst = data->getVState();

    double Li2c[3][3];
    for(int t = 0; t < 3; t++){
        for(int r = 0; r < 3; r++)
            Li2c[t][r] = data->Li2c[t][r];
    }
    //std::copy(&data->Li2c[0][0], &data->Li2c[0][0]+3*3, Li2c[0][0]);
    double zVal[DBSIZE][fpMeas->NUMFPS];
    int expect2see_fp[DBSIZE]   = {0};
    double focallengthx = (double)IMAGESIZEH/(IMAGESIZEH*tan(FOVH/2.)); //1/tan(FOVH/2.);
    double focallengthy = (double)IMAGESIZEV/(IMAGESIZEH*tan(FOVV/2.)); //1/tan(FOVV/2.);

    int i,j,k;
    for (j=0; j<DBSIZE; j++){       // Estimated fp in database
        if(fpEst->fpID[j] >= 0){    // Make sure DB feature has been initialised
            /*********************************************
             *  Project estimates to the image plane  ****
             *********************************************/
            // Find the position vector of fp wrt to camera, hcam (in camera frame)
            double hcam[3], hin[3];
            double psi      = fpEst->state[j][3];
            double theta    = fpEst->state[j][4];
            double rho      = fpEst->state[j][5];
            double px       = fpEst->state[j][0];
            double py       = fpEst->state[j][1];
            double pz       = fpEst->state[j][2];
            double m[3];
            double stheta   = sin(theta);
            double ctheta   = cos(theta);
            double spsi     = sin(psi);
            double cpsi     = cos(psi);
            m[0] =  cpsi*ctheta;
            m[1] =  spsi*ctheta;
            m[2] = -stheta;
            hin[0] = rho*(px - vehicleEst->pos.X) + m[0];
            hin[1] = rho*(py - vehicleEst->pos.Y) + m[1];
            hin[2] = rho*(pz - vehicleEst->pos.Z) + m[2];
            hcam[0] = Li2c[0][0]*hin[0] + Li2c[0][1]*hin[1] + Li2c[0][2]*hin[2];
            hcam[1] = Li2c[1][0]*hin[0] + Li2c[1][1]*hin[1] + Li2c[1][2]*hin[2];
            hcam[2] = Li2c[2][0]*hin[0] + Li2c[2][1]*hin[1] + Li2c[2][2]*hin[2];

            if (hcam[0] > 0.01){

                // Project the position vector onto the image plane using the pinhole camera model
                double fpPixEst[2];
                fpPixEst[0] = focallengthx*hcam[1]/hcam[0];
                fpPixEst[1] = focallengthy*hcam[2]/hcam[0];

                if ((fpPixEst[0] > -1) && (fpPixEst[0] < 1) &&
                    (fpPixEst[1] > -(double)IMAGESIZEV/IMAGESIZEH) &&
                    (fpPixEst[1] < (double)IMAGESIZEV/IMAGESIZEH)){

                    // We expect to see the predicted fp
                    expect2see_fp[j] = 1;

                    // Populate measurement Jacobian fp matrix, Cfp
                    double Cfp[NUMFPMEAS][NUMFPSTATES];
                    double foclen[2] = {focallengthx, focallengthy};
                    ComputeCinvD(j, hcam, foclen, Cfp, data);

                    // Rt = inv(CPC'+R), where P = [Pnav 0; 0 Pfp] and C = [Cnav Cfp]
                    double CfpP[NUMFPMEAS][NUMFPSTATES], CfpPCt[NUMFPMEAS][NUMFPMEAS], Rt[NUMFPMEAS][NUMFPMEAS];
                    mat_mult((double *)Cfp, NUMFPMEAS, NUMFPSTATES,(double *)fpEst->fpP[j], NUMFPSTATES,NUMFPSTATES,(double *)CfpP);
                    mat_mult_T((double *)CfpP,NUMFPMEAS,NUMFPSTATES, (double *)Cfp,NUMFPMEAS,NUMFPSTATES, (double *)CfpPCt);

                    for (i=0;i<NUMFPMEAS;i++){
                        for(k=0;k<NUMFPMEAS;k++){
                            Rt[i][k] = CfpPCt[i][k] + data->fpR[i][k];
                        }
                    }

                    double inv_Rt[NUMFPMEAS][NUMFPMEAS];
                    mat_invert((double *)Rt,NUMFPMEAS,(double *)inv_Rt);

                    /*********************************************
                    *  Calculate the Mahalanobis distance   ******
                    *               z = e'*(CPC'+R)^(-1)*e  ******
                    **********************************************/
                    // Difference between measurement and estimate projection
                    for (i=0; i<fpMeas->NUMFPS; i++){ // measured fp
                        double res[2];

                        res[0] = fpMeas->fpLocNorm[i][0] - fpPixEst[0];
                        res[1] = fpMeas->fpLocNorm[i][1] - fpPixEst[1];

                        // z-value ( j is database point/estimate, i is measured point)
                        zVal[j][i] = res[0]*(inv_Rt[0][0]*res[0] + inv_Rt[0][1]*res[1]) + res[1]*(inv_Rt[1][0]*res[0] + inv_Rt[1][1]*res[1]);
                    }
                }
            }
        }
    }
    /*********************************************
    *   Match db points with measurements   ******
    **********************************************/
    /* bestCandidate_fp stores the best matching measurement location
     * for a point in the database;
     * bestZ_fp records the best (minimum) z value for a point in the database;
     * assignZ_fp represents the best (minimum) z value for the measured points;
     * assign_fp indicates which point in the database a measurement has
     * already been assigned to.*/

    int bestCandidate_fp[DBSIZE];
    double bestZ_fp[DBSIZE];
    double assignZ_fp[fpMeas->NUMFPS];
    for (int i = 0; i<fpMeas->NUMFPS; i++){
        assignZ_fp[i] = zTestLim;
        fpMeas->assign_fp[i] = -1;
    }

    for (int i = 0; i<DBSIZE; i++){
        bestCandidate_fp[i] = -1;
    }

    bool notAssigned = true;
    while (notAssigned){
        notAssigned = false;
        for(i=0; i<DBSIZE; i++){    // Loop through database points

            // If we expect to see database fp but it is not assigned
            if(expect2see_fp[i] && (bestCandidate_fp[i] < 0) ){

                bestZ_fp[i] = zTestLim;

                /* loop through measurements and find best candidate that's not
                already assigned to a better match */
                for (j=0; j<fpMeas->NUMFPS; j++){       // Loop through measurements

                    if( (zVal[i][j] < bestZ_fp[i]) && (zVal[i][j] < assignZ_fp[j]) ){
                        bestZ_fp[i] = zVal[i][j];
                        bestCandidate_fp[i] = j;
                    }
                }

                if(bestCandidate_fp[i] >= 0){
                    /* Here we check whether the best match for the current point in the
                     * database has already been paired with another - if so, and we beat
                     * the z-value of that pairing, the other loses its match and needs
                     * reassignment*/
                    if(fpMeas->assign_fp[bestCandidate_fp[i]] >= 0){
                        bestCandidate_fp[fpMeas->assign_fp[bestCandidate_fp[i]]] = -1;
                        notAssigned = true;
                    }
                    assignZ_fp[bestCandidate_fp[i]] = bestZ_fp[i];
                    fpMeas->assign_fp[bestCandidate_fp[i]] = i;
                }
            }
        }
    }
    /* Update the confidence we have in a particular feature point.
     * If we do not receive a measurement for a particular fp, and
     * we don't expect to see it, our confidence drops by 1. if we
     * do not receive a measurement, but expect to see it, our
     * confidence drops by 2. If we get a measurement for a fp,
     * our confidence increases */

    // Count number of corresponded points
    int numCorresponded = 0;
    for (i = 0; i<DBSIZE; i++){
        if (fpEst->fpID[i] >= 0){
            if (bestCandidate_fp[i] >= 0){
                numCorresponded++;
                fpEst->confidence[i] = MIN(fpEst->confidence[i]+3, 100);
            } else {
                if (expect2see_fp[i] > 0){
                    fpEst->confidence[i] = MAX(fpEst->confidence[i]-2, 0);
                } else{
                    fpEst->confidence[i] = MAX(fpEst->confidence[i]-1, 0);
                }
            }
        }
    }

    fpEst ->numCorresponded = numCorresponded;
    /* Update the initilisation and fp deletion confidence threshold
     * based on total number of corresponded features*/
    if (numCorresponded < fpMeas->NUMFPS*0.1)       // Poor correspondance number - set confidence threshold high, so replacement is fast
        fpEst->confThresh = 100*0.9;
    else if (numCorresponded < fpMeas->NUMFPS*0.3)
        fpEst->confThresh = 100*0.7;
    else if (numCorresponded < fpMeas->NUMFPS*0.5)  //NUMFPS*0.7)  // Average correspondance number
        fpEst->confThresh = 100*0.5;
    else if (numCorresponded < fpMeas->NUMFPS*0.9)
        fpEst->confThresh = 100*0.2;
    else                                            // Good correspondance number - set confidence threshold low, so replacement is slow
        fpEst->confThresh = 100*0.1;
}


void ComputeCinvD(int j, double hcam[3], double focallength[2], double Cfp[NUMFPMEAS][NUMFPSTATES], shared *data)
{
    fpDatabase *fpEst = data->getFpStates();
    vehicleState * vehicleEst = data->getVState();
    double Li2c[3][3];
    for(int t = 0; t < 3; t++){
        for(int r = 0; r < 3; r++)
            Li2c[t][r] = data->Li2c[t][r];
    }
    //std::copy(&data->Li2c[0][0], &data->Li2c[0][0]+9, Li2c);

    double psi      = fpEst->state[j][3];
    double theta    = fpEst->state[j][4];
    double rho      = fpEst->state[j][5];
    double px       = fpEst->state[j][0];
    double py       = fpEst->state[j][1];
    double pz       = fpEst->state[j][2];
    double stheta   = sin(theta);
    double ctheta   = cos(theta);
    double spsi     = sin(psi);
    double cpsi     = cos(psi);
    double focallengthx = focallength[0];
    double focallengthy = focallength[1];
    double fpPixEst[2];

    fpPixEst[0] = focallengthx*hcam[1]/hcam[0];
    fpPixEst[1] = focallengthy*hcam[2]/hcam[0];

    // Feature point state:     y           = [px, py, pz, psi, theta, rho]
    // Cam2fp vector (c frame): h           = Li2c * {rho * (pa - vEst) + m(psi,theta)}
    // Measurement vector:      z           = [u; v] = [fx * hy/hx; fy * hz/hx]
    // Construct measurement covarariance matrix
    //                          C = dz/dy   = [du/dpa[3], du/dpsi, du/dtheta, du/drho]
    //                                        [dv/dpa[3], dv/dpsi, dv/dtheta, dv/drho]
    //    where:                dz/dpa      = dz/dh * dh/dpa
    //                          dz/dpsi     = dz/dh * dh/dpsi
    //                          dz/dtheta   = dz/dh * dh/dtheta
    //                          dz/drho     = dz/dh * dh/drho

    double dzdPa[NUMFPMEAS][3], dzdrC[NUMFPMEAS][3];
    double dzdrI[NUMFPMEAS][3];
    // dz/dh    = [-fx*h[1]/h[0]^2, fx/h[0], 0]
    //            [-fy*h[2]/h[0]^2, 0, fy/h[0]]
    dzdrI[0][0] = -fpPixEst[0]/hcam[0];  dzdrI[0][1] = focallengthx/hcam[0];  dzdrI[0][2] = 0;
    dzdrI[1][0] = -fpPixEst[1]/hcam[0];  dzdrI[1][1] = 0;                     dzdrI[1][2] = focallengthy/hcam[0];

    mat_mult((double*)dzdrI,NUMFPMEAS,3,(double*)Li2c,3,3,(double*)dzdrC);
    mat_mult_const((double*)dzdrC,NUMFPMEAS,3,rho,(double*)dzdPa);
    double dzdTheta[NUMFPMEAS], dzdPsi[NUMFPMEAS], dzdRho[NUMFPMEAS], dmdTheta[3], dmdPsi[3], drdRho[3];

    // dh/dpsi      = Li2c * dm[3]/dpsi
    // dh/dtheta    = Li2c * dm[3]/dtheta
    // dh/drho      = Li2c * d(pa-vEst)/drho
    dmdPsi[0] = -spsi*ctheta;           dmdTheta[0] = -cpsi*stheta;     drdRho[0] = px - vehicleEst->pos.X;
    dmdPsi[1] =  cpsi*ctheta;           dmdTheta[1] = -spsi*stheta;     drdRho[1] = py - vehicleEst->pos.Y;
    dmdPsi[2] =  0;                     dmdTheta[2] = -ctheta;          drdRho[2] = pz - vehicleEst->pos.Z;

    mat_mult((double*)dzdrC,NUMFPMEAS,3,dmdPsi,3,1,dzdPsi);
    mat_mult((double*)dzdrC,NUMFPMEAS,3,dmdTheta,3,1,dzdTheta);
    mat_mult((double*)dzdrC,NUMFPMEAS,3,drdRho,3,1,dzdRho);
    Cfp[0][0] = dzdPa[0][0];    Cfp[1][0] = dzdPa[1][0];
    Cfp[0][1] = dzdPa[0][1];    Cfp[1][1] = dzdPa[1][1];
    Cfp[0][2] = dzdPa[0][2];    Cfp[1][2] = dzdPa[1][2];
    Cfp[0][3] = dzdPsi[0];      Cfp[1][3] = dzdPsi[1];
    Cfp[0][4] = dzdTheta[0];    Cfp[1][4] = dzdTheta[1];
    Cfp[0][5] = dzdRho[0];      Cfp[1][5] = dzdRho[1];

    /*double L11, L12, L13, L21, L22, L23, L31, L32, L33;
    L11 = Li2c[0][0]; L12 = Li2c[0][1]; L13 = Li2c[0][2];
    L21 = Li2c[1][0]; L22 = Li2c[1][1]; L23 = Li2c[1][2];
    L31 = Li2c[2][0]; L32 = Li2c[2][1]; L33 = Li2c[2][2];

    Cfp[0][0] = (+1.)*(-fpPixEst[0]*L11 + focallengthx*L21)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[0][1] = (+1.)*(-fpPixEst[0]*L12 + focallengthx*L22)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[0][2] = (+1.)*(-fpPixEst[0]*L13 + focallengthx*L23)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[1][0] = (+1.)*(-fpPixEst[1]*L11 + focallengthy*L31)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[1][1] = (+1.)*(-fpPixEst[1]*L12 + focallengthy*L32)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[1][2] = (+1.)*(-fpPixEst[1]*L13 + focallengthy*L33)/hcam[0];        // accomodates the hin changes (additional rho multiplier)
    Cfp[0][3] = (+1.)*(-fpPixEst[0]*(-L11*cos(theta)*sin(psi)+L12*cos(theta)*cos(psi))                + focallengthx*(-L21*cos(theta)*sin(psi)+L22*cos(theta)*cos(psi)               ))/(rho*hcam[0]);          // accomodates the hin changes (additional rho multiplier)
    Cfp[1][3] = (+1.)*(-fpPixEst[1]*(-L11*cos(theta)*sin(psi)+L12*cos(theta)*cos(psi))                + focallengthy*(-L31*cos(theta)*sin(psi)+L32*cos(theta)*cos(psi)               ))/(rho*hcam[0]);          // accomodates the hin changes (additional rho multiplier)
    Cfp[0][4] = (-1.)*(-fpPixEst[0]*(+L11*sin(theta)*cos(psi)+L12*sin(theta)*sin(psi)+L13*cos(theta)) + focallengthx*(+L21*sin(theta)*cos(psi)+L22*sin(theta)*sin(psi)+L23*cos(theta)))/(rho*hcam[0]);          // accomodates the hin changes (additional rho multiplier)
    Cfp[1][4] = (-1.)*(-fpPixEst[1]*(+L11*sin(theta)*cos(psi)+L12*sin(theta)*sin(psi)+L13*cos(theta)) + focallengthy*(+L31*sin(theta)*cos(psi)+L32*sin(theta)*sin(psi)+L33*cos(theta)))/(rho*hcam[0]);          // accomodates the hin changes (additional rho multiplier)
    Cfp[0][5] = (-1.)*(-fpPixEst[0]*(+L11*cos(theta)*cos(psi)+L12*cos(theta)*sin(psi)-L13*sin(theta)) + focallengthx*(+L21*cos(theta)*cos(psi)+L22*cos(theta)*sin(psi)-L23*sin(theta)))/(hcam[0]*SQ(rho));
    Cfp[1][5] = (-1.)*(-fpPixEst[1]*(+L11*cos(theta)*cos(psi)+L12*cos(theta)*sin(psi)-L13*sin(theta)) + focallengthy*(+L31*cos(theta)*cos(psi)+L32*cos(theta)*sin(psi)-L33*sin(theta)))/(hcam[0]*SQ(rho));
    Cfp[0][5] = (+1.)*(-fpPixEst[0]*(+L11*(px-vEst[0])+L12*(py-vEst[1])+L13*(pz-vEst[2]))             + focallengthx*(+L21*(px-vEst[0])+L22*(py-vEst[1])+L23*(pz-vEst[2])))/(hcam[0]);                      // accomodates the hin changes (rho on different term now)
    Cfp[1][5] = (+1.)*(-fpPixEst[1]*(+L11*(px-vEst[0])+L12*(py-vEst[1])+L13*(pz-vEst[2]))             + focallengthy*(+L31*(px-vEst[0])+L32*(py-vEst[1])+L33*(pz-vEst[2])))/(hcam[0]);                      // accomodates the hin changes (rho on different term now)
    Cfp[0][5] = (-1.)*(-fpPixEst[0]*(+L11*cos(theta)*cos(psi)+L12*cos(theta)*sin(psi)-L13*sin(theta)) + focallengthx*(+L21*cos(theta)*cos(psi)+L22*cos(theta)*sin(psi)-L23*sin(theta)))/(hcam[0]*SQ(rho));
    Cfp[1][5] = (-1.)*(-fpPixEst[1]*(+L11*cos(theta)*cos(psi)+L12*cos(theta)*sin(psi)-L13*sin(theta)) + focallengthy*(+L31*cos(theta)*cos(psi)+L32*cos(theta)*sin(psi)-L33*sin(theta)))/(hcam[0]*SQ(rho));
    */
}




void findSigmaPoints(int i, fpDatabase *fpEst, double SigPoints[NUMFPSTATES][2*NUMFPSTATES])
{
    double nP[NUMFPSTATES][NUMFPSTATES];
    mat_scalar_mult((double*)fpEst->fpP[i], NUMFPSTATES, NUMFPSTATES, NUMFPSTATES, (double *)nP);
    double U[NUMFPSTATES][NUMFPSTATES];
    CholeskyDecomposition(nP, U);

    for (int k = 0; k < NUMFPSTATES; k++){
        for (int j = 0; j < NUMFPSTATES; j++){
            SigPoints[j][k] = fpEst->state[i][j] + U[k][j];
            SigPoints[j][k+NUMFPSTATES] = fpEst->state[i][j] - U[k][j];
        }
    }
}


/* Computes the Cholesky Decomposition U for a symmetric matrix P
 * according to:
 *  P = U^(T)*U
 * P: Symmetric matrix
 * U: Upper triangular decomposition of P
*/
void CholeskyDecomposition(double P[NUMFPSTATES][NUMFPSTATES], double U[NUMFPSTATES][NUMFPSTATES])
{

    for (int i = 0; i < NUMFPSTATES; i++){
        double sum = 0;
        for (int k = 0; k <i; k++){
            sum += pow(U[k][i],2);
        }
        U[i][i] = sqrt(P[i][i] - sum);
        for (int j = 0; j < NUMFPSTATES; j++){
            if(j < i){
                U[i][j] = 0;
            }else if (j > i){
                sum = 0;
                for (int k = 0; k <i; k++){
                    sum += U[k][j]*U[k][i];
                }
                U[i][j] = (P[i][j] - sum)/U[i][i];
            }
        }
    }
}
