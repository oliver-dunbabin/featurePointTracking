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

// Class used to run onboard code - i.e. handles the filter, sensor streams, plotting, and logging functions.
class Onboard
{
public:

    Onboard(bool log, int filterType);
    Onboard(std::string file, std::string path, bool log, int filterType);
    ~Onboard();

    bool time_to_exit;

    void updateOnboard();   // This function is called to start the onboard code; that is, reads sensors, performs filter update, and logs data to binary file

    void simulation();      // This function reads data from previously saved binary files, and runs the filter (no sensors necessary). Used for filter tuning and parameter purposes.

    void stopOnboard();     // Stops the onboard code; that is, stops sensor threads.

    void testSensors();     // This function is called strictly to test connected T265 and Jevois sensors

    void testCamera();      // This function is called strictly to test connected camera

private:

    bool gif = false;               // Set flag to save plots as GIF
    bool log_data;                  // Set flag if we want to log fp estimates
    bool use_vicon = true;          // Set flag if using Vicon motion capture system for validation purposes
    std::thread ViconThread;        // Thread to read from Vicon datalink
    JevoisSerialPort *sPort;        // Serial port object
    JevoisSerialPort *sPort_exit;   // Points to sPort (serial port object), and used to exit serial read thread upon program exit
    rwJevois *camera;               // Camera object
    rwJevois *camera_exit;          // Points to camera (camera object), and used to exit camera read thread upon program exit
    T265_Connect t265;              // Realsense T265 object
    T265_Connect *t265_exit;        // Points to t265 (T265 object), and used to exit T265 read thread upon program exit
    plotFP visualise;               // GNU plot object used to visualise data

    shared data;                    // Shared object used to store, and share pertinent filter values
    LogData *fpMeasFile;            // Binary logfile object to save fp measurements
    LogData *PoseFile;              // Binary logfile object to save vehicle pose values
    LogData *fpEstFile;             // Binary logfile object to save fp estimates
    std::string filename;           // Prefix of name of binary files (yearyearr-monthmonth-dayday_hrhr-minmin-secsec)
    std::string filedir;            // Directory where logfiles are saved
    FILE *measfile, *posefile;      // FILE object - simulation reads from these

    void sensorUpdate();    // Updates messages from T265 and camera

    void setFileDir(std::string path){filedir = path;}

    // Template function to read from saved binary files (used in simulation)
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

    bool camUpdate();       // Updates camera message

    bool vehicleUpdate();   // Updates T265 message

    void logEstimates();    // Logs fp estimates to binary file

    void handle_quit();     // Helper function to cangle program exit (e.g. ctrl+c)

    void getViconData();    // Gets data from Vicon motion tracking system for validation purposes

    void startViconThread();// Starts Vicon read thread

};

#endif // ONBOARD_H
