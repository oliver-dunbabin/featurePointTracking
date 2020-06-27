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

// Class used to get data from RealSense T265
class T265_Connect
{
public:
    T265_Connect(std::string orientation);

    T265_Connect();

    ~T265_Connect(){
        stopThread();
    }

    void startThread();             // Starts a thread to read from T265

    void stopThread();              // Stops read thread

    bool getData(vehicleState *X);  // Get pose from vehicle pose buffer

    bool setData(vehicleState *X);  // Push latest pose message into circular buffer

    void handle_quit();             // Handle used to safely close read thread upon ctr+c

    std::atomic<bool> config_error; // Flag handles T265 configuration error - exits program

private:
    void read_RS();     // Infinite loop reads T265

    void readPose(vehicleState *X);     // Reads latest message from T265

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    CircularBuffer<vehicleState> pose_dataBuf;  // Circular buffer of pose messages

    std::string CamOrient;  // Camera orientation (w.r.t vehicle body frame and gravity vector)

    bool time_to_exit;      // Flag used to exit thread (e.g. upon ctrl+c command)

    // Threading objects
    std::mutex mu;
    std::thread readThread;
    bool read_status;

};

#endif // T265_CONNECT_H
