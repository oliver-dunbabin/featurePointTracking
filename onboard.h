#ifndef ONBOARD_H
#define ONBOARD_H

#include "fpmappingproc.h"
#include "rwjevois.h"
#include "jevoisserialport.h"
#include "stdio.h"
#include "t265_connect.h"
#include "logdata.h"
#include <stdlib.h>
#include "plotfp.h"

#define ENDFILE 0
#define ERR -1
#define OK 1

class Onboard
{
public:

    Onboard(bool log);
    Onboard(std::string file, std::string path, bool log);
    ~Onboard();

    bool time_to_exit;

    void updateOnboard();

    void simulation();

    void stopOnboard();

    void testSensors();

    void testCamera();

private:

    bool plot = false;
    bool log_data;
    bool first_vEst = true;
    JevoisSerialPort *sPort;
    JevoisSerialPort *sPort_exit;
    rwJevois *camera;
    rwJevois *camera_exit;
    T265_Connect t265;
    T265_Connect *t265_exit;
    plotFP visualise;
    uint32_t tdelay1;

    shared data;
    LogData *fpMeasFile;
    LogData *PoseFile;
    LogData *fpEstFile;
    std::string filename;
    std::string filedir;
    FILE *measfile, *posefile;

    void sensorUpdate();

    void setFileDir(std::string path){filedir = path;}

    template<typename T>
    bool readFromFile(T *datastruct, const char *filename, FILE *file){
        if (feof(file)){
            printf("end of file reached for %s", filename);
            return ENDFILE;
        }else if(file != nullptr){
            fread(datastruct, sizeof(T), 1, file);
            return OK;
        }else{
            printf("could not open file %s", filename);
            return ERR;
        }

    }

    bool camUpdate();

    bool vehicleUpdate();

    void logEstimates();

    void handle_quit();

};

#endif // ONBOARD_H
