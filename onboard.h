#ifndef ONBOARD_H
#define ONBOARD_H

#include "fpmappingproc.h"
#include "rwjevois.h"
#include "jevoisserialport.h"
#include "stdio.h"
#include "t265_connect.h"
#include "logdata.h"

class Onboard
{
public:

    Onboard(bool log);
    ~Onboard();

    bool time_to_exit;

    void updateOnboard();

    void stopOnboard();

    void testSensors();

    void testCamera();

private:

    JevoisSerialPort *sPort;
    JevoisSerialPort *sPort_exit;
    rwJevois *camera;
    rwJevois *camera_exit;
    T265_Connect t265;
    T265_Connect *t265_exit;

    shared data;
    bool log_data;
    LogData *fpMeasFile;
    LogData *PoseFile;
    LogData *fpEstFile;

    void sensorUpdate();

    bool camUpdate();

    bool vehicleUpdate();

    void logEstimates();

    void handle_quit();

};

#endif // ONBOARD_H
