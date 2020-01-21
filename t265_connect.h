#ifndef T265_CONNECT_H
#define T265_CONNECT_H

#include "rs.hpp"
#include <iostream>
#include <iomanip>
#include <math.h>
#include <pthread.h>
#include <float.h>
#include <thread>
#include <mutex>
#include "coordinatetransform.h"
#include <stdint.h>
#include "shared.h"

class T265_Connect
{
public:
    T265_Connect(std::string orientation);

    T265_Connect();

    ~T265_Connect(){
        stopThread();
    }

    void startThread();

    void stopThread();

    bool getData(vehicleState *X);

    void setData(vehicleState *X);

    void handle_quit(int sig);

private:
    void read_RS();

    void readPose(vehicleState *X);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    vehicleState pose_data;

    std::string CamOrient;

    bool time_to_exit;
    bool read_status;

    // Threading
    std::mutex mu;
    std::thread readThread;

};

#endif // T265_CONNECT_H
