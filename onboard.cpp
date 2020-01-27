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
        fpMeasFile  = new LogData("measurement", sizeof("measurement"), true);
        PoseFile    = new LogData("pose", sizeof("pose"), true);
        fpEstFile   = new LogData("estimate", sizeof("estimate"), true);
    }else{
        fpMeasFile  = nullptr;
        PoseFile    = nullptr;
        fpEstFile   = nullptr;
    }
}


Onboard::Onboard(std::string filename, std::string path, bool log)
{
    data.gotCAMmsg  = false;
    data.gotPOSEmsg = false;
    time_to_exit    = false;

    readfile = filename;
    setFileDir(path);
    log_data = log;

    sPort   = nullptr;
    camera  = nullptr;

    if(log_data){
        std::string estpath = filepath + "/estimate/" + readfile + "_estimate.fpBIN";
        fpEstFile   = new LogData(estpath.c_str(), sizeof(estpath.c_str()), false);
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

            bool dbUpdated = updateMappingFP(&data);

            if(log_data && dbUpdated){
                fpDatalink *fpData = data.getFpData();
                fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
            }

        }
    }else{
        perror("Could not synchronise JEVOIS and HOST clocks");
    }

    handle_quit();

}


void Onboard::simulation()
{
    std::string campath = filepath + "/measurement/" + readfile + "_measurement.fpBIN";
    std::string pospath = filepath + "/pose/" + readfile + "_pose.fpBIN";

    while (!time_to_exit){
        camMessage fpMsg;
        vehicleState vEst;
        int gotpose = readFromFile<vehicleState>(&vEst, pospath.c_str());
        int gotcam  = readFromFile<camMessage>(&fpMsg, campath.c_str());

        uint64_t time_i;
        if(gotpose == OK){
            time_i = vEst.timestamp;
            data.setVehicleState(&vEst);
        }else if (gotpose == ENDFILE){
            std::cout << "End of file reached" << std::endl;
            break;
        }else {
            std::cout << "could not open pose file" << std::endl;
            break;
        }

        if(gotcam == OK){
            data.setCamMsg(&fpMsg);

            while (!data.gotCAMmsg){
                uint64_t time_f;
                if(gotpose == OK){
                    data.setVehicleState(&vEst);

                    if(vEst.timestamp > fpMsg.time_stmp){
                        data.gotCAMmsg = true;
                        time_i = fpMsg.time_stmp;
                    }else{
                        gotpose = readFromFile<vehicleState>(&vEst, pospath.c_str());
                        data.setVehicleState(&vEst);
                    }

                    time_f = vEst.timestamp;
                    double dt = (double)(time_f - time_i);
                    updateFPcov(dt, &data);
                    time_i = time_f;
                }
            }
            doCorrespondance(false, &data);
            updateFPmeas(&data);
            updatefpDatalink(&data);
            if(log_data){
                fpDatalink *fpData = data.getFpData();
                fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
            }
            data.gotCAMmsg = false;
        }else if (gotcam == ENDFILE){
            std::cout << "End of file reached" << std::endl;
            break;
        }else{
            std::cout << "could not open cam file" << std::endl;
            break;
        }
    }
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


void Onboard::testSensors()
{

    sPort->start();

    bool clock_set = camera->setJevoisClock();

    camera->start();

    t265.startThread();

    data.fpTimeI = data.timer.now();

    if (clock_set){
        while(!time_to_exit){

            sensorUpdate();

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
    //std::string camString;

    bool msgGrab = camera->harrisMsgBuf.pop(&sensorMsg);
    if (msgGrab){
        fpMsg.time_stmp     = sensorMsg.time;
        fpMsg.sendT         = sensorMsg.sendT;
        fpMsg.imgHeight     = sensorMsg.imageHeight;
        fpMsg.imgWidth      = sensorMsg.imageWidth;

        //camString += std::to_string(sensorMsg.time) + "," + std::to_string(sensorMsg.sendT);

        int j = 0;
        for(int i = 0; i < constants::MAXFPS; i++){

            //camString += "," + std::to_string(sensorMsg.fpVal[i]) + "," + std::to_string(sensorMsg.fpCoord[i][0]) + "," + std::to_string(sensorMsg.fpCoord[i][1]);
            if(sensorMsg.fpVal[i] > constants::harrisValThresh){
                double fpLocRow = (sensorMsg.fpCoord[i][1] - 0.5*(double)fpMsg.imgHeight)/(0.5*(double)fpMsg.imgHeight)*constants::IMAGESIZEV/((double)constants::IMAGESIZEH);
                double fpLocCol = (sensorMsg.fpCoord[i][0] - 0.5*((double)fpMsg.imgWidth))/(0.5*(double)fpMsg.imgWidth)*constants::IMAGESIZEH/((double)constants::IMAGESIZEH);
                fpMsg.fpLocNorm[j][0] = fpLocCol;
                fpMsg.fpLocNorm[j][1] = fpLocRow;
                j++;
            }
        }

        fpMsg.NUMFPS = j;
        if(j > 0){
            data.setCamMsg(&fpMsg);

            //std::cout << "\n" << camString << "\n";
            if(log_data){
                //fpMeasFile->saveData(camString);
                //logEstimates();
                fpMeasFile->saveBinary<camMessage>(&fpMsg, sizeof(camMessage));
            }

            return true;
        }
    }
    return false;
}


bool Onboard::vehicleUpdate()
{
    vehicleState vEst;
    bool msgGrab = t265.getData(&vEst);
    if (msgGrab){
        data.setVehicleState(&vEst);
        /*std::string posedata;
        posedata += std::to_string(vEst.timestamp) + "," + std::to_string(vEst.pos.X) + "," + std::to_string(vEst.pos.Y) + "," + std::to_string(vEst.pos.Z) + ","
                + std::to_string(vEst.quat.Q0) + "," + std::to_string(vEst.quat.Q1) + "," + std::to_string(vEst.quat.Q2) + "," + std::to_string(vEst.quat.Q3);
        std::cout << "\n" << pose_data << std::endl;
        */
        if(log_data){
            //PoseFile->saveData(posedata);
            PoseFile->saveBinary<vehicleState>(&vEst, sizeof(vehicleState));
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
