#include "plotfp.h"
#include <vector>


plotFP::plotFP(bool gif)
{
    save_to_gif = gif;
    setup_plot();
}


void plotFP::setup_plot()
{
    est  << "set terminal qt 0 size 800, 800\n";
    est  << "set xrange [-0.5:2]\nset yrange [2:-2]\nset zrange [2:-2]\n";
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
    double Li2b[3][3];
    double qi2b[4] = {vEst->quat.Q0, vEst->quat.Q1, vEst->quat.Q2, vEst->quat.Q3};
    quat2dcm321(qi2b, Li2b);
    std::vector<boost::tuple<double, double, double>> finalFrust;
    mat_T_mult((double *)Li2b, 3, 3, (double *)frustrum, 3, 13, (double *)transFrust);
    for(int i = 0; i < 13; i++){
        //finalFrust.push_back(boost::make_tuple(frustrum[0][i], frustrum[1][i], frustrum[2][i]));
        finalFrust.push_back(boost::make_tuple(transFrust[0][i], transFrust[1][i], transFrust[2][i]));
    }

    // Estimated fp Locations
    std::vector<boost::tuple<double, double, double, double>> estPos;
    for (int i = 0; i < DBSIZE; i++){
        if (fpEst->fpID[i] >= 0){
            if( (fpEst->confidence[i] > 90  && fpEst->state[i][5] > 0.0) ) {
                double X = ( fpEst->state[i][0] - vEst->pos.X ) + cos(fpEst->state[i][4])*cos(fpEst->state[i][3])/fpEst->state[i][5];
                double Y = ( fpEst->state[i][1] - vEst->pos.Y ) + cos(fpEst->state[i][4])*sin(fpEst->state[i][3])/fpEst->state[i][5];
                double Z = ( fpEst->state[i][2] - vEst->pos.Z ) - sin(fpEst->state[i][4])/fpEst->state[i][5];
                estPos.push_back(boost::make_tuple(X, Y, Z, -fpEst->confidence[i]));
            }
        }
    }

    std::vector<boost::tuple<double,double>> rawMeas;
    for(int i = 0; i < fpMeas->NUMFPS; i++)
        rawMeas.push_back(boost::make_tuple(fpMeas->fpLocNorm[i][0], fpMeas->fpLocNorm[i][1]));

    meas << "set title \"Measurements\"\n";
    meas << "unset label 1\n";
    meas << "set label 1 \"frame no.: " + std::to_string(frame) + "\" at -0.9,-0.7 font \",14\"\n";
    meas << "plot '-' with points pt 7 ps 1.5 title 'measurement'\n";
    meas.send1d(rawMeas);
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
