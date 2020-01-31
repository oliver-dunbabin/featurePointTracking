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
#include "circularbuffer.h"
#include <atomic>
#define poseBufLen 100

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

    bool setData(vehicleState *X);

    void handle_quit();

    std::atomic<bool> config_error;

private:
    void read_RS();

    void readPose(vehicleState *X);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    CircularBuffer<vehicleState> pose_dataBuf;

    std::string CamOrient;

    bool time_to_exit;

    // Threading
    std::mutex mu;
    std::thread readThread;
    bool read_status;

};

#endif // T265_CONNECT_H
