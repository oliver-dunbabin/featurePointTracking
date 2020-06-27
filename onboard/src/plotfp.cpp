#include "plotfp.h"
#include <vector>
#include "coordinatetransform.h"


plotFP::plotFP(bool gif)
{
    save_to_gif = gif;
    setup_plot();
}


void plotFP::setup_plot()
{
    est  << "set terminal qt 0 size 800, 800\n";
    est  << "set xrange [-0.5:8]\nset yrange [0.5:-7]\nset zrange [2:-4]\n";
    est  << "set view equal xyz\n";
    est  << "set view 180, 270, 0.55, 1\n";
    meas << "set terminal qt 1\n";
    meas << "set xrange [-1:1]\nset yrange [0.8:-0.8]\n";
    est  << "set grid\n";
    est  << "set datafile missing \"NaN\"\n";
    meas << "set grid\n";
    meas << "set datafile missing \"NaN\"\n";

    if(save_to_gif){
        meas << "set term gif animate medium size 640,480 background '#ffffff' delay 6\n";
        meas << "set output \"measurement.gif\"\n";
        est  << "set term gif animate medium size 640,480 background '#ffffff' delay 6\n";
        est  << "set output \"estimate.gif\"\n";
    }
}

void plotFP::plot(shared *data, int frame)
{
    fpDatabase *fpEst   = data->getFpStates();
    camMessage *fpMeas  = data->getCamMsg();
    vehicleState *vEst  = data->getVState();

    double frustrum[3][13] = {0.25, 0, 0.25, 0.25, 0, 0.25, 0.25, 0, 0.25, 0.25, 0, 0.25, 0.25,
                              0.25, 0, 0.25, 0.25, 0, 0.25, -0.25, 0, -0.25, -0.25, 0, -0.25, 0.25,
                              0.15, 0, 0.15, -0.15, 0, -0.15, -0.15, 0, -0.15, 0.15, 0, 0.15, 0.15};
    double transFrust[3][13];
    // Calculate frustrum attitude
    double Li2b[3][3], Li2c[3][3], Lb2i[3][3], Lc2i[3][3];
    double pEst[3] = {vEst->pos.X, vEst->pos.Y, vEst->pos.Z};
    double db2c_i[3];
    double qi2b[4] = {vEst->quat.Q0, vEst->quat.Q1, vEst->quat.Q2, vEst->quat.Q3};
    quat2dcm(qi2b, Li2b);
    mat_mult((double *)Lb2c, 3, 3, (double *)Li2b, 3, 3, (double *)Li2c);
    mat_transpose((double *)Li2b, 3, 3, (double *)Lb2i);
    mat_transpose((double *)Li2c, 3, 3, (double *)Lc2i);
    map_vector(Lb2i, camR, db2c_i);
    for (int t = 0; t < 3; t++){pEst[t] += db2c_i[t];}

    std::vector<boost::tuple<double, double, double>> finalFrust;
    mat_T_mult((double *)Li2c, 3, 3, (double *)frustrum, 3, 13, (double *)transFrust);
    // Define camera frustrum in inertia frame
    for(int i = 0; i < 13; i++){
        //finalFrust.push_back(boost::make_tuple(frustrum[0][i], frustrum[1][i], frustrum[2][i]));
        finalFrust.push_back(boost::make_tuple(transFrust[0][i]+pEst[0], transFrust[1][i]+pEst[1], transFrust[2][i]+pEst[2]));
    }

    // Estimated fp Locations from inverse depth states
    std::vector<boost::tuple<double, double, double, double>> estPos;
    for (int i = 0; i < DBSIZE; i++){
        if (fpEst->fpID[i] >= 0){
            if( (fpEst->confidence[i] > 70  && fpEst->state[i][5] > 0.0) ) {
                double Xc = ( fpEst->state[i][0] - pEst[0] ) + cos(fpEst->state[i][4])*cos(fpEst->state[i][3])/fpEst->state[i][5];
                double Yc = ( fpEst->state[i][1] - pEst[1] ) + cos(fpEst->state[i][4])*sin(fpEst->state[i][3])/fpEst->state[i][5];
                double Zc = ( fpEst->state[i][2] - pEst[2] ) - sin(fpEst->state[i][4])/fpEst->state[i][5];
                double X  = Xc + pEst[0];
                double Y  = Yc + pEst[1];
                double Z  = Zc + pEst[2];
                estPos.push_back(boost::make_tuple(X, Y, Z, -fpEst->confidence[i]));
            }
        }
    }

    std::vector<boost::tuple<double, double>> estMeas;
    double focallengthx = IMAGESIZEH/(IMAGESIZEH*tan(FOVH/2.));
    double focallengthy = IMAGESIZEV/(IMAGESIZEH*tan(FOVV/2.));
    // Project fp estimates onto camrea frame (predicted measurements)
    for (int i = 0; i < DBSIZE; i++){
        double hcam[3], hin[3];
        double px = fpEst->state[i][0];
        double py = fpEst->state[i][1];
        double pz = fpEst->state[i][2];
        double Psi = fpEst->state[i][3];
        double Theta = fpEst->state[i][4];
        double Rho = fpEst->state[i][5];
        double stheta   = sin(Theta);
        double ctheta   = cos(Theta);
        double spsi     = sin(Psi);
        double cpsi     = cos(Psi);
        double m[3];
        m[0] =  cpsi*ctheta;    //cos(psi)*cos(theta);
        m[1] =  spsi*ctheta;    //sin(psi)*cos(theta);
        m[2] = -stheta;         //-sin(theta);
        hin[0] = Rho*(px - pEst[0]) + m[0];
        hin[1] = Rho*(py - pEst[1]) + m[1];
        hin[2] = Rho*(pz - pEst[2]) + m[2];
        hcam[0] = Li2c[0][0]*hin[0] + Li2c[0][1]*hin[1] + Li2c[0][2]*hin[2];
        hcam[1] = Li2c[1][0]*hin[0] + Li2c[1][1]*hin[1] + Li2c[1][2]*hin[2];
        hcam[2] = Li2c[2][0]*hin[0] + Li2c[2][1]*hin[1] + Li2c[2][2]*hin[2];
        if (hcam[0] > 0.01){
            double fpPixEst[2];
            fpPixEst[0] = hcam[1]/hcam[0]*focallengthx;
            fpPixEst[1] = hcam[2]/hcam[0]*focallengthy;
            if (fpEst->confidence[i] > 70)
                estMeas.push_back(boost::make_tuple(fpPixEst[0], fpPixEst[1]));
        }
    }

    std::vector<boost::tuple<double,double>> rawMeas;
    for(int i = 0; i < fpMeas->NUMFPS; i++){
        rawMeas.push_back(boost::make_tuple(fpMeas->fpLocNorm[i][0], fpMeas->fpLocNorm[i][1]));
    }

    // Finally plot estimates and measurements in separate windows
    meas << "set title \"Measurements\"\n";
    meas << "unset label 1\n";
    meas << "set label 1 \"frame no.: " + std::to_string(frame) + "\" at -0.9,-0.7 font \",14\"\n";
    meas << "plot '-' with points pt 7 ps 1.5 title 'measurement', ";
    if (!estMeas.empty()){
        meas << "'-' with points pt 7 title 'estimate'\n";
    }
    meas.send1d(rawMeas);
    if (!estMeas.empty()){meas.send1d(estMeas);}
    meas.flush();
    est  << "set cbrange [-100:" + std::to_string(-fpEst->confThresh) + "]\n";
    est  << "set title \"# corresponded:    " + std::to_string(fpEst->numCorresponded) + "/" +
            std::to_string(DBSIZE) + "\" font \",20\"\n";
    est  << "splot '-' with points pt 7 ps 1 palette notitle,";
    est  << " '-' with lines lc rgb \"red\" title 'camera FOV'\n";
    est.send1d(estPos);
    est.send1d(finalFrust);
    est.flush();
}
