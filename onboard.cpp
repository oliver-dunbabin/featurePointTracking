#include "onboard.h"
#include <unistd.h>

Onboard::Onboard(bool log)
{
    data.gotCAMmsg  = false;
    data.gotPOSEmsg = false;
    time_to_exit    = false;

    std::vector<std::string> portID{"/dev/ttyACM0","/dev/ttyACM1", "/dev/ttyACM2","/dev/ttyACM3"};
    int baudrate = 9600;
    sPort       = new JevoisSerialPort(baudrate, portID, false);
    camera      = new rwJevois(sPort);
    sPort_exit  = sPort;
    camera_exit = camera;
    t265_exit   = &t265;

    log_data = log;
    if(log_data){
        fpMeasFile  = new LogData("-measurement.txt");
        PoseFile    = new LogData("-pose.txt");
        fpEstFile   = new LogData("-estimate.txt");
    }else{
        fpMeasFile  = nullptr;
        PoseFile    = nullptr;
        fpEstFile   = nullptr;
    }
}


Onboard::~Onboard()
{
    delete sPort;
    delete camera;
    delete fpMeasFile;
    delete PoseFile;
    delete fpEstFile;
    sPort = nullptr;
    camera = nullptr;
    fpMeasFile = nullptr;
    PoseFile = nullptr;
    fpEstFile = nullptr;


}

void Onboard::updateOnboard()
{

    sPort->start();

    bool clock_set = camera->setJevoisClock();

    camera->start();

    t265.startThread();

    data.fpTimeI = data.timer.now();

    if (clock_set){
        std::cout << "Clock Set successful" << std::endl;
        while(!time_to_exit){

            sensorUpdate();

            updateMappingFP(&data);

        }
    }else{
        perror("Could not synchronise JEVOIS and HOST clocks");
    }

    handle_quit();

}

void Onboard::testCamera()
{

    sPort->start();

    bool clock_set = camera->setJevoisClock();

    camera->start();

    data.fpTimeI = data.timer.now();

    if (clock_set){
        while(!time_to_exit){

            camUpdate();
        }
    }else{
        perror("Could not synchronise JEVOIS and HOST clocks");
    }

    handle_quit();
}

bool Onboard::camUpdate()
{
    harrisMessageFP sensorMsg;
    camMessage fpMsg;
    std::string camString;

    bool msgGrab = camera->harrisMsgBuf.pop(&sensorMsg);
    if (msgGrab){
        fpMsg.time_stmp     = sensorMsg.time;
        fpMsg.imgHeight     = sensorMsg.imageHeight;
        fpMsg.imgWidth      = sensorMsg.imageWidth;

        camString += std::to_string(sensorMsg.time);

        int j = 0;
        for(int i = 0; i < constants::MAXFPS; i++){

            camString += "," + std::to_string(sensorMsg.fpVal[i]) + "," + std::to_string(sensorMsg.fpCoord[i][0]) + "," + std::to_string(sensorMsg.fpCoord[i][1]);

            if(sensorMsg.fpVal[i] > constants::harrisValThresh){
                double fpLocRow = (sensorMsg.fpCoord[i][1] - 0.5*(double)fpMsg.imgHeight)/(0.5*(double)fpMsg.imgHeight)*constants::IMAGESIZEV/((double)constants::IMAGESIZEH);
                double fpLocCol = (sensorMsg.fpCoord[i][0] - 0.5*((double)fpMsg.imgWidth))/(0.5*(double)fpMsg.imgWidth)*constants::IMAGESIZEH/((double)constants::IMAGESIZEH);
                fpMsg.fpLocNorm[j][0] = fpLocCol;
                fpMsg.fpLocNorm[j][1] = fpLocRow;
                j++;
            }
        }

        if(log_data){
            fpMeasFile->saveData(camString);
            logEstimates();
        }

        fpMsg.NUMFPS = j;
        if(j > 0){
            data.setCamMsg(&fpMsg);
            return true;
        }
    }
    return false;
}


bool Onboard::vehicleUpdate()
{
    vehicleState *vEst    = data.getVState();
    bool msgGrab = t265.getData(vEst);
    if (msgGrab){
        data.setVehicleState(vEst);
        if(log_data){
            std::string posedata;
            posedata += std::to_string(vEst->timestamp) + "," + std::to_string(vEst->pos.X) + "," + std::to_string(vEst->pos.Y) + "," + std::to_string(vEst->pos.Z)
                    + std::to_string(vEst->quat.Q0) + "," + std::to_string(vEst->quat.Q1) + "," + std::to_string(vEst->quat.Q2) + "," + std::to_string(vEst->quat.Q3);
            PoseFile->saveData(posedata);
        }
    }
    return msgGrab;
}


void Onboard::logEstimates()
{
    const fpDatabase *fpEst     = data.getFpStates();
    const camMessage *fpMsg     = data.getCamMsg();
    const vehicleState *vEst    = data.getVState();
    std::string estData;
    estData += std::to_string(fpMsg->time_stmp);
    estData += "," + std::to_string(vEst->pos.X) + "," + std::to_string(vEst->pos.Y) + "," + std::to_string(vEst->pos.Z) + "," +std::to_string(vEst->quat.Q0)
            + "," + std::to_string(vEst->quat.Q1) + "," + std::to_string(vEst->quat.Q2) + "," + std::to_string(vEst->quat.Q3);

    for (int i = 0; i < constants::DBSIZE; i++){
        if(fpEst->fpID[i] >= 0){
            estData += "," + std::to_string(fpEst->fpID[i]) + "," + std::to_string(fpEst->confidence[i]);
            for(int j = 0; j < constants::NUMFPSTATES; j++)
                estData += "," + std::to_string(fpEst->state[i][j]);
        }else{
            estData += ",-1,-1,NAN,NAN,NAN,NAN,NAN,NAN";
        }
    }

    fpEstFile->saveData(estData);
}


void Onboard::sensorUpdate()
{
    data.gotCAMmsg = camUpdate();
    data.gotPOSEmsg = vehicleUpdate();
}


void Onboard::stopOnboard()
{
    time_to_exit = true;
    std::cout << "\nCLOSING FILES\n" << std::endl;
    fpEstFile->close();
    fpMeasFile->close();
    PoseFile->close();

}


// -------------------------------------------------------------------
//   Quit Handler
// -------------------------------------------------------------------
void Onboard::handle_quit()
{

    try{
        camera_exit->handle_quit();
    }
    catch (int error){
        fprintf(stderr,"\nWarning, could not stop CAMERA READ interface\n");
    }

    try{
        sPort_exit->handle_quit();
    }
    catch (int error){
        fprintf(stderr,"\nWarning, could not stop SERIAL PORT interface\n");
    }

    try{
        t265_exit->handle_quit();
    }
    catch (int error){
        fprintf(stderr,"\nWarning, could not stop T265 interface\n");
    }

    try{
        stopOnboard();
    }
    catch (int error) {
        fprintf(stderr,"\nWarning, could not stop ONBOARD interface\n");
    }

}
