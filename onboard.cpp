#include "onboard.h"
#include <unistd.h>
#include <sys/stat.h>

Onboard::Onboard(bool log):visualise(plot)
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
    if(log){
        fpMeasFile  = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/measurement", WRITE);
        PoseFile    = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/pose", WRITE);
        fpEstFile   = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/estimate", WRITE);
    }else{
        fpMeasFile  = nullptr;
        PoseFile    = nullptr;
        fpEstFile   = nullptr;
    }
}


Onboard::Onboard(std::string file, std::string path, bool log):visualise(plot)
{
    data.gotCAMmsg  = false;
    data.gotPOSEmsg = false;
    time_to_exit    = false;

    filename = file;
    setFileDir(path);
    log_data = log;

    sPort       = nullptr;
    camera      = nullptr;

    if(log_data){
        std::string estpath = filedir + "estimate/" + filename + ".fpBIN";
        struct stat buffer;
        int i = 0;
        while(stat(estpath.c_str(), &buffer) == 0){
            i++;
            estpath = filedir + "estimate/" + filename + "(" + std::to_string(i) + ")" + ".fpBIN";
        }
        fpEstFile   = new LogData(estpath, -1);
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

    if(t265.config_error){
        handle_quit();
        return;
    }

    data.fpTimeI = data.timer.now();

    if (clock_set){
        while(!time_to_exit){

            sensorUpdate();

            bool dbUpdated = updateMappingFP(&data);

            if(log_data && dbUpdated){
                fpDatalink *fpData = data.getFpData();
                fprintf(stderr,"\t%s\n\t%s\n\t%s\n\n", fpMeasFile->getFilename().c_str(), fpEstFile->getFilename().c_str(),PoseFile->getFilename().c_str());
                using namespace std::chrono;
                high_resolution_clock::time_point t1 = high_resolution_clock::now();
                fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                duration<double> dt = duration_cast<duration<double, std::micro>>(t2 - t1);
                printf("write time: %f\n", dt.count());
            }

        }
    }else{
        perror("Could not synchronise JEVOIS and HOST clocks");
    }

    handle_quit();

}


void Onboard::simulation()
{
    std::string campath = filedir + "measurement/" + filename + ".fpBIN";
    std::string pospath = filedir + "pose/" + filename + ".fpBIN";

    uint32_t frame = 0;
    measfile = fopen(campath.c_str(), "rb");
    posefile = fopen(pospath.c_str(), "rb");
    std::cout << fpEstFile->getFilename();



    while (!time_to_exit){
        camMessage fpMsg;
        vehicleState vEst;
        int gotpose = readFromFile<vehicleState>(&vEst, pospath.c_str(), posefile);
        int gotcam  = readFromFile<camMessage>(&fpMsg, campath.c_str(), measfile);
        time_delay_fill(data.v_latent_initialised, vEst, data.v_latent, t265_LATENCY);
        data.v_latent_initialised = true;

        uint64_t time_i;
        if(gotpose == OK){
            time_i = vEst.timestamp;
            data.setVehicleState(&vEst);
        }else if (gotpose == ENDFILE){
            printf("\n\nEnd of file reached");
            break;
        }else {
            printf("\n\ncould not open pose file");
            break;
        }

        if(gotcam == OK){
            data.setCamMsg(&fpMsg);

            while (!data.gotCAMmsg){
                uint64_t time_f;
                if(gotpose == OK){

                    if(vEst.timestamp > fpMsg.time_stmp){
                        data.gotCAMmsg = true;
                        time_i = fpMsg.time_stmp;
                    }else{
                        gotpose = readFromFile<vehicleState>(&vEst, pospath.c_str(), posefile);
                        data.setVehicleState(&vEst);
                        time_delay_fill(data.v_latent_initialised, vEst, data.v_latent, t265_LATENCY);
                    }

                    time_f = vEst.timestamp;
                    double dt = (double)(time_f - time_i);
                    updateFPcov(dt, &data);
                    time_i = time_f;
                }
            }

            vehicleState vState = time_delay_get(data.v_latent_initialised, fpMsg.time_stmp, t265_period, data.v_latent, t265_LATENCY);
            data.setVehicleState(&vState);
            doCorrespondance(false, &data);
            updateFPmeas(&data);
            updatefpDatalink(&data);
            if(log_data){
                fpDatalink *fpData = data.getFpData();
                for(int i=0; i<DBSIZE; i++){printf("(%f,%f,%f,%f,%f,%f)", fpData->state[i][0], fpData->state[i][1], fpData->state[i][2], fpData->state[i][3],
                        fpData->state[i][4], fpData->state[i][5]);}
                printf("\n\n\n");
                fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
            }
            data.gotCAMmsg = false;
            frame ++;
            //visualise.plot(&data, frame);
        }else if (gotcam == ENDFILE){
            printf("\n\nEnd of file reached\n");
            break;
        }else{
            printf("\n\nCould not open cam file\n");
            break;
        }
    }
    fclose(measfile);
    fclose(posefile);
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

    if(t265.config_error){
        handle_quit();
        return;
    }

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

    bool msgGrab = camera->harrisMsgBuf.pop(&sensorMsg);
    if (msgGrab){
        fpMsg.time_stmp     = sensorMsg.time;
        fpMsg.sendT         = sensorMsg.sendT;
        fpMsg.imgHeight     = sensorMsg.imageHeight;
        fpMsg.imgWidth      = sensorMsg.imageWidth;

        vehicleState vState = time_delay_get(data.v_latent_initialised, fpMsg.time_stmp, t265_period, data.v_latent, t265_LATENCY);
        data.setVehicleState(&vState);
        //printf("IN:\t%lu,%f,%f,%f,%f,%f,%f,%f\n",vState.timestamp,vState.pos.X, vState.pos.Y, vState.pos.Z, vState.quat.Q0, vState.quat.Q1, vState.quat.Q2, vState.quat.Q3);
        //printf("frame time:\t%lu\n", fpMsg.time_stmp);

        int j = 0;
        for(int i = 0; i < constants::MAXFPS; i++){

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

        time_delay_fill(data.v_latent_initialised, vEst, data.v_latent, t265_LATENCY);
        data.v_latent_initialised = true;

        //printf("RS:\t%lu,%f,%f,%f,%f,%f,%f,%f\n",vEst.timestamp, vEst.pos.X, vEst.pos.Y, vEst.pos.Z, vEst.quat.Q0, vEst.quat.Q1, vEst.quat.Q2, vEst.quat.Q3);
        //printf("RS time:\t%lu\n", vEst.timestamp);
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
    printf("\n\nCLOSING FILES\n");
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
