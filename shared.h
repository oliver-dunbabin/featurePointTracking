#ifndef SHARED_H
#define SHARED_H

#include <stdint.h>
#include <chrono>
#include <stdio.h>
#include <cstring>
#include <mutex>
#include "matrix.h"


#define EKF 0
#define UKF 1
#define t265_LATENCY 100

namespace constants {
    const double RHO = 0.5;                 // inverse depth initialisation (1/m)
    const int DBSIZE = 100;                 // Max number of features to track
    const int MAXFPS = 100;                 // Max number of fps from camera
    const int harrisValThresh = 0;          // Harris value threshold
    const int NUMFPSTATES = 6;              // Number of states per feature point
    const int NUMVSTATES = 7;               // Number of vehicle states
    const int NUMFPMEAS = 2;                // Number of measurements per feature point
    const int NUMVMEAS = 6;                 // Number of vehicle state measurements
    const double zTestLim = 0.6;            // Threshold for fp correspondance test

    // Filter covariance
    const double SIGMAFPMEAS = 0.05;        // feature point measurement noise in position (per full screen)
    const double SIGMAFPANG = 0.01;         // Covariance initialisation SD in fp angular states [rad]
    const double SIGMAFPINVD = 2;           // Covariance initialisaiton SD in fp inverse depth
    const double SIGMAACC = 0.1;            // acceleromer noise [m/s^2]
    const double SIGMAGYR = 0.001;          // gyro noise [rad/s]
    const double SIGMAPOS = 0.015;          // inertial position noise (m)

    // Camera sensor properties
    const double PIXELSIZE = 3.18;          // micrometers
    const int IMAGESIZEH = 1280;            // horizontal sensor resolution [pixels]
    const int IMAGESIZEV = 1024;            // vertical sensor resolution [pixels]
    const double FOVH = 0.888237;//1.04719755;          // stated sensor field of view [rad] //(measured)
    const double FOVV = 0.66617775;//0.83775804;          // stated sensor field of view [rad] //(measured)
    const int FOCALLENGTH = 2800;           // sensor focal length [micrometers]
    const double PI = 3.1415926535;         // PI (duh!)
    const double GRAVITY = 9.807;           // positive down in NED coordinates [m/s^2]

    // T265 properties
    const double t265_period = 0.005013;    // Update period of t265 (s)
    const double Lb2c[3][3] = {1., 0., 0.,  // Body (T265 frame) to camera frame DCM
                               0., 1., 0.,
                               0., 0., 1.};
    const double camR[3]    = {0., 0., -0.014};            // Position of cam wrt body (in body frame)

}

using namespace constants;
typedef std::chrono::high_resolution_clock Clock;

struct camMessage{
    uint64_t time_stmp;                     // Jevois program time for grabbing frame (first Byte read from sensor)
    uint64_t arrivalT;                      // Time message arrives at the filter
    uint16_t imgWidth;                      // Sensor horizontal resolution
    uint16_t imgHeight;                     // Sensor vertical resolution
    double fpLocNorm[MAXFPS][NUMFPMEAS];    // image plane location of feature points (x * y)
    int assign_fp[MAXFPS];                  // measurement corresponds to db member i
    int NUMFPS = 0;                         // Number of feature points extracted from frame
}__attribute__((packed));


struct fpDatabase{
    double state [DBSIZE][NUMFPSTATES];
    double fpP[DBSIZE][NUMFPSTATES][NUMFPSTATES];
    int confidence[DBSIZE];                 // confidence in database fp estimate
    int confThresh;                         // confidence threshold level (initialised at 50)
    int fpID[DBSIZE];
    int newFpID;
    int numCorresponded;
    bool dbInit = false;                    // has db been initialised?
}__attribute__((packed));


struct fpDatalink{
    float state [DBSIZE][NUMFPSTATES];
    float fpP[DBSIZE][NUMFPSTATES][NUMFPSTATES];
    int32_t fpID[DBSIZE];
    int8_t confidence[DBSIZE];
    int8_t confThreash;
    uint8_t numCorresponded;
}__attribute__((packed));


struct localPos{
    double X;
    double Y;
    double Z;
}__attribute__((packed));

struct quatAtt{
    double Q0;
    double Q1;
    double Q2;
    double Q3;
}__attribute__((packed));

struct vehicleState{
    localPos pos;
    quatAtt quat;
    uint64_t timestamp;
}__attribute__((packed));


class shared
{
private:
    vehicleState *ptrvState = nullptr;
    camMessage *ptrfpHarris = nullptr;
    fpDatabase *ptrfpStates = nullptr;
    fpDatalink *ptrfpData   = nullptr;

public:
    shared(int filterType = 0);
    ~shared();

    vehicleState *getVState(){return ptrvState;}
    camMessage *getCamMsg(){return ptrfpHarris;}
    fpDatabase *getFpStates(){return ptrfpStates;}
    fpDatalink *getFpData(){return ptrfpData;}
    void setCamMsg(camMessage *fpMeas);
    void setVehicleState(vehicleState *vState);
    double Li2b[3][3];
    double fpR[NUMFPMEAS][NUMFPMEAS] = {0};
    double fpQ[NUMFPSTATES][NUMFPSTATES] = {0};
    bool gotCAMmsg;
    bool gotPOSEmsg;
    int filter_type;
    vehicleState v_latent[t265_LATENCY];
    bool v_latent_initialised = false;

    Clock::time_point fpTimeI;
    Clock::time_point fpTimeF;
    Clock timer;
};

void quat2dcm321( double qEst[4], double DCM[3][3]);

void map_vector(const double T[3][3], const double vin[3], double vout[3] );

void quatNorm(double q[4]);

void quat_mult( double a[4], double b[4], double c[4]);


#endif // SHARED_H
