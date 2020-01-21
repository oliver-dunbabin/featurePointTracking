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

    void updateOnboard();

    void stopOnboard();

    void handle_quit(int);

private:

    JevoisSerialPort *sPort;
    JevoisSerialPort *sPort_exit;
    rwJevois *camera;
    rwJevois *camera_exit;
    T265_Connect t265;
    T265_Connect *t265_exit;

    shared data;
    bool time_to_exit;
    bool log_data;
    LogData *fpMeasFile;
    LogData *PoseFile;
    LogData *fpEstFile;

    void sensorUpdate();

    bool camUpdate();

    bool vehicleUpdate();

    void logEstimates();

};

#endif // ONBOARD_H
