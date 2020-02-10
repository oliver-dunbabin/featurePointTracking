#include "onboard.h"
#include <unistd.h>
#include <sys/stat.h>
#include "DataStreamClient.h"

Onboard::Onboard(bool log):visualise(gif)
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
        fpMeasFile  = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/measurement", BINWRITE);
        PoseFile    = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/pose", BINWRITE);
        fpEstFile   = new LogData("/home/oliver/MySoftware/featurePointTracking/testdata/estimate", BINWRITE);
        filename    = fpMeasFile->getFilename();
    }else{
        fpMeasFile  = nullptr;
        PoseFile    = nullptr;
        fpEstFile   = nullptr;
    }
}


Onboard::Onboard(std::string file, std::string path, bool log):visualise(gif)
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
            estpath = filedir + "estimate/" + filename + "(" + std::to_string(i) + ")" + ".txt";
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
        if(use_vicon){
            //getViconData();
            ViconThread = std::thread(&Onboard::startViconThread,this);
        }

        while(!time_to_exit){

            sensorUpdate();

            bool dbUpdated = updateMappingFP(&data);

            if(log_data && dbUpdated){
                fpDatalink *fpData = data.getFpData();
                using namespace std::chrono;
                high_resolution_clock::time_point t1 = high_resolution_clock::now();
                fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
                high_resolution_clock::time_point t2 = high_resolution_clock::now();
                duration<double> dt = duration_cast<duration<double, std::micro>>(t2 - t1);
            }

        }
    }else{
        perror("Could not synchronise JEVOIS and HOST clocks");
    }
    if(use_vicon){
        if (ViconThread.joinable()){
            ViconThread.join();
        }
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
                //fpEstFile->saveBinary<fpDatalink>(fpData, sizeof(fpDatalink));
                logEstimates();
            }
            data.gotCAMmsg = false;
            frame ++;
            visualise.plot(&data, frame);
            usleep(50000);
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

        printf("RS:\t%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",vEst.timestamp, vEst.pos.X, vEst.pos.Y, vEst.pos.Z, vEst.quat.Q0, vEst.quat.Q1, vEst.quat.Q2, vEst.quat.Q3);
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
            if((fpEst->confidence[i] > 0) && (fpEst->state[i][5] > 0)){
                estData += "," + std::to_string(fpEst->fpID[i]) + "," + std::to_string(fpEst->confidence[i]);
                for(int j = 0; j < constants::NUMFPSTATES; j++)
                    estData += "," + std::to_string(fpEst->state[i][j]);
            }
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


void Onboard::getViconData(){
    std::string name = "/home/oliver/MySoftware/featurePointTracking/testdata/vicon/" + fpMeasFile->getFilename() + ".txt";
    LogData viconfile(name, -1);
    using namespace ViconDataStreamSDK::CPP;
    // *********************************
    //      Make a new Vicon client  *
    //      Put IP address here      *
    // *********************************
    std::string HostName = "192.168.1.100:801";
    Client MyClient;

    // Connect to a server
    printf("\n\nConnecting to Vicon at %s ...", HostName.c_str());
    while( !MyClient.IsConnected().Connected )
    {
        // Direct connection

        bool ok = false;
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
        if(!ok)
        {
            printf("\n\nWarning - connection failed");
        }

        printf(".");
        sleep(1);
    }
    printf("\n\nConnected to Vicon at %s\n", HostName.c_str());
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableMarkerRayData();
    MyClient.EnableDeviceData();
    MyClient.EnableDebugData();

    // Set the streaming mode
    MyClient.SetStreamMode( StreamMode::ClientPull );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward,
                             Direction::Right,
                             Direction::Down ); // Z-down (FRD - coordinate)

    for (int k = 0; k < 100; k ++){
        // Get Vicon data
        while(MyClient.GetFrame().Result != Result::Success){
            sleep(1);
            printf(".");
        }

        unsigned short SubjectCount = MyClient.GetSubjectCount().SubjectCount;
        double jevoisPos[3], jevoisAtt[4], fpPos[5][3];
        for(unsigned short SubIndex = 0; SubIndex < SubjectCount; ++SubIndex){
            // Get subject name
            std::string SubjectName = MyClient.GetSubjectName(SubIndex).SubjectName;

            // Get segment name
            int i = 0;
            unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
            for (unsigned short SegIndex = 0; SegIndex < SegmentCount; ++SegIndex){
                std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegIndex ).SegmentName;
                if ((SubjectName == "Jevois") || (SegmentName == "Jevois")){
                    Output_GetSegmentGlobalTranslation JevoisXYZ = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    Output_GetSegmentGlobalRotationQuaternion JevoisQuat = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    jevoisPos[0] = JevoisXYZ.Translation[0]*0.001; jevoisPos[1] = JevoisXYZ.Translation[1]*0.001; jevoisPos[2] = JevoisXYZ.Translation[2]*0.001;
                    double ViconQuat[4] = {JevoisQuat.Rotation[3], JevoisQuat.Rotation[0], JevoisQuat.Rotation[1], JevoisQuat.Rotation[2]};
                    double q_vb2b[4] = {0., 1., 0., 0.};
                    quat_mult(ViconQuat, q_vb2b, jevoisAtt);
                }else if ((SubjectName == "Wand") || (SegmentName == "Wand")){
                    unsigned int MarkerCount = MyClient.GetMarkerCount(SubjectName).MarkerCount;
                    for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex){
                        // Get marker name
                        std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

                        // Get the global marker translation
                        Output_GetMarkerGlobalTranslation MarkerXYZ = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                        fpPos[i][0] = MarkerXYZ.Translation[0]; fpPos[i][1] = MarkerXYZ.Translation[1]; fpPos[i][2] = MarkerXYZ.Translation[2];
                        i++;
                    }
                }
            }
        }
        std::string msg = std::to_string(jevoisPos[0]) + "," + std::to_string(jevoisPos[1]) + "," + std::to_string(jevoisPos[2]) + "," +
                std::to_string(jevoisAtt[0]) + "," + std::to_string(jevoisAtt[1]) + "," + std::to_string(jevoisAtt[2]) + "," + std::to_string(jevoisAtt[3]);
        for ( int l = 0; l < 5; l++)
            msg += "," + std::to_string(fpPos[l][0]) + "," + std::to_string(fpPos[l][1]) + "," + std::to_string(fpPos[l][2]);
        viconfile.saveData(msg);
    }
    viconfile.close();
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    // Disconnect and dispose
    printf("\n\nDisconnecting ...\n");
    MyClient.Disconnect();
}


void Onboard::startViconThread(){
    std::string name = "/home/oliver/MySoftware/featurePointTracking/testdata/vicon/" + fpMeasFile->getFilename() + ".txt";
    LogData viconfile(name, -1);
    using namespace ViconDataStreamSDK::CPP;
    using namespace std::chrono;
    // *********************************
    //      Make a new Vicon client  *
    //      Put IP address here      *
    // *********************************
    std::string HostName = "192.168.1.100:801";
    Client MyClient;

    // Connect to a server
    printf("\n\nConnecting to Vicon at %s ...", HostName.c_str());
    while( !MyClient.IsConnected().Connected )
    {
        // Direct connection

        bool ok = false;
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
        if(!ok)
        {
            printf("\n\nWarning - connection failed");
        }

        printf(".");
        sleep(1);
    }
    printf("\n\nConnected to Vicon at %s\n", HostName.c_str());
    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableMarkerRayData();
    MyClient.EnableDeviceData();
    MyClient.EnableDebugData();

    // Set the streaming mode
    MyClient.SetStreamMode( StreamMode::ClientPull );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward,
                             Direction::Right,
                             Direction::Down ); // Z-down (FRD - coordinate)

    while(!time_to_exit){
        // Get Vicon data
        while(MyClient.GetFrame().Result != Result::Success){
            sleep(1);
            printf(".");
        }

        time_point<system_clock,microseconds> ts = time_point_cast<microseconds>(system_clock::now());
        uint64_t time_stmp = ts.time_since_epoch().count();
        unsigned short SubjectCount = MyClient.GetSubjectCount().SubjectCount;
        double jevoisPos[3], jevoisAtt[4], fpPos[5][3];
        for(unsigned short SubIndex = 0; SubIndex < SubjectCount; ++SubIndex){
            // Get subject name
            std::string SubjectName = MyClient.GetSubjectName(SubIndex).SubjectName;

            // Get segment name
            int i = 0;
            unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
            for (unsigned short SegIndex = 0; SegIndex < SegmentCount; ++SegIndex){
                std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegIndex ).SegmentName;
                if ((SubjectName == "Jevois") || (SegmentName == "Jevois")){
                    Output_GetSegmentGlobalTranslation JevoisXYZ = MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
                    Output_GetSegmentGlobalRotationQuaternion JevoisQuat = MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
                    jevoisPos[0] = JevoisXYZ.Translation[0]*0.001; jevoisPos[1] = JevoisXYZ.Translation[1]*0.001; jevoisPos[2] = JevoisXYZ.Translation[2]*0.001;
                    double ViconQuat[4] = {JevoisQuat.Rotation[3], JevoisQuat.Rotation[0], JevoisQuat.Rotation[1], JevoisQuat.Rotation[2]};
                    double q_vb2b[4] = {0., 1., 0., 0.};
                    quat_mult(ViconQuat, q_vb2b, jevoisAtt);
                }else if ((SubjectName == "Wand") || (SegmentName == "Wand")){
                    unsigned int MarkerCount = MyClient.GetMarkerCount(SubjectName).MarkerCount;
                    for (unsigned int MarkerIndex = 0; MarkerIndex < MarkerCount; ++MarkerIndex){
                        // Get marker name
                        std::string MarkerName = MyClient.GetMarkerName( SubjectName, MarkerIndex ).MarkerName;

                        // Get the global marker translation
                        Output_GetMarkerGlobalTranslation MarkerXYZ = MyClient.GetMarkerGlobalTranslation( SubjectName, MarkerName );
                        fpPos[i][0] = MarkerXYZ.Translation[0]; fpPos[i][1] = MarkerXYZ.Translation[1]; fpPos[i][2] = MarkerXYZ.Translation[2];
                        i++;
                    }
                }
            }
        }
        std::string msg = std::to_string(time_stmp) + "," + std::to_string(jevoisPos[0]) + "," + std::to_string(jevoisPos[1]) + "," + std::to_string(jevoisPos[2]) + "," +
                std::to_string(jevoisAtt[0]) + "," + std::to_string(jevoisAtt[1]) + "," + std::to_string(jevoisAtt[2]) + "," + std::to_string(jevoisAtt[3]);
        for ( int l = 0; l < 5; l++)
            msg += "," + std::to_string(fpPos[l][0]) + "," + std::to_string(fpPos[l][1]) + "," + std::to_string(fpPos[l][2]);
        viconfile.saveData(msg);
    }

    viconfile.close();

    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    // Disconnect and dispose
    printf("\n\nDisconnecting ...\n");
    MyClient.Disconnect();
}
