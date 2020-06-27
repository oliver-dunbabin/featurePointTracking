#include "rwjevois.h"
#include "chrono"
#include <sstream>

// Checksum calculation
unsigned char calculateCheckSum(unsigned char *buf, int byteCount, int index){
    unsigned char csum = 0;
    for (int i = index; i < byteCount; i++){
        csum += buf[i];
    }
    return csum;
}

// Initialises serial port with string value, and creates harris message circular buffer of size CBUFLEN
rwJevois::rwJevois(JevoisSerialPort *serial_port_):harrisMsgBuf(CBUFLEN)
{
    //assign serial port object passed into class
    serial_port = serial_port_;
    usleep(1000);
    //initialise counters
    init_counters();
}

// Initialises serial port to default value, and creates harris message circular buffer of size CBUFLEN
rwJevois::rwJevois() : harrisMsgBuf(CBUFLEN),serial_port()
{
    usleep(1000);
    init_counters();
}

rwJevois::~rwJevois()
{}


void rwJevois::init_counters()
{
    reading_status = false;     // whether read thread running
    writing_status = false;     // whether write thread running
    write_count = 0;
    serial_port->bytesRead = 0;

    time_to_exit = false;       // signals thread exit
}


void rwJevois::init_camera_proc()
{
    // Configure Jevois camera to Harris Corner detection via serial port
    const std::vector<std::string> msg {"setpar serout USB\0", "setmapping2 YUYV 320 240 28.3 OliverDunbabin HarrisCorner\0"}; // Changed resolution to match aspect ratio of camera sensor
    for (auto& x : msg){
        std::string full_msg;
        write_messages(x);
        usleep(1000);
        read_messages(full_msg);    // Reads response from camera
        usleep(1000);
    }
}

// Synchronises Jevois' clock with computer
// Because we can only set Jevois clock with second resolution, we send message on the second
bool rwJevois::setJevoisClock()
{
    using namespace std;
    using namespace std::chrono;
    bool time_set = false;
    time_point<high_resolution_clock> time1 = high_resolution_clock::now();
    int deltaT;
    do{
        auto now = system_clock::now();
        auto now_ms = time_point_cast<milliseconds>(now);
        auto now_ms_int = now_ms.time_since_epoch().count();
        int r = now_ms_int % 1000;
        if (r == 0){
            //Send clock message to Jevois
            time_t time = system_clock::to_time_t(now);
            tm utc_time = *gmtime(&time);
            string months   = to_string(utc_time.tm_mon+1);
            string days     = to_string(utc_time.tm_mday);
            string hours    = to_string(utc_time.tm_hour);
            string mins     = to_string(utc_time.tm_min);
            string years    = to_string(utc_time.tm_year + 1900);
            string secs     = to_string(utc_time.tm_sec);

            string stime = "date " + string(2 - months.length(), '0') + months + string(2 - days.length(), '0') + days
                    + string(2 - hours.length(), '0') + hours + string(2 - mins.length(), '0') + mins
                    + years + "." + string(2 - secs.length(), '0') + secs;
            this->write_messages(stime);
            time_set = true;
            read_messages();
            time_point<high_resolution_clock> time2 = high_resolution_clock::now();
            deltaT = duration_cast<duration<int,std::micro>>(time2 - now).count();
            printf("\n\n%i", deltaT);
            usleep(1000);
            return true;
        }else{
            time_point<high_resolution_clock> time2 = high_resolution_clock::now();
            deltaT = duration_cast<duration<int,std::milli>>(time2 - time1).count();
            usleep(100);
        }
    }while (!time_set && deltaT < 10000);
    return false;
}


int rwJevois::write_messages(const std::string &msg, std::chrono::time_point<std::chrono::system_clock> prevT)
{
    // write message to serial port
    int len = serial_port->write_msg(msg);
    auto now = std::chrono::system_clock::now();
    auto deltaT = std::chrono::duration_cast<std::chrono::duration<int,std::micro>>(now - prevT).count();

    if(len > 1){
        printf("\n\nSENT: [   %s   ] with delay: %i micro sec", msg.c_str(), deltaT);
    }

    // update counters
    write_count ++;

    // Number of bites written
    return len;
}

int rwJevois::write_messages(const std::string &msg)
{
    // write message to serial port
    int len = serial_port->write_msg(msg);


    if(len > 1){
        printf("\n\nSENT: [   %s   ]", msg.c_str());
    }

    // update counters
    write_count ++;

    // Number of bites written
    return len;
}


void rwJevois::read_messages(std::string &full_msg)
{
    int received_all = 0; // received full message

    while(!received_all && !time_to_exit)
    {
        char msg;
        received_all = serial_port->read_msg(&msg);
        // Fill vector with message bytes until EOL character
        full_msg += msg;
        // Check to see if buffer has been exceeded
        if (full_msg.size() >= BUFFLEN) {
            fprintf(stderr,"\nMessage length exceeds BUFFLEN %i",BUFFLEN);
            return;
        }else if(full_msg.size() > 1 && received_all){
            printf("\n\nRECEIVED: [   %s   ]", full_msg.c_str());
            return;
        }
    }
}


void rwJevois::read_messages()
{
    bool received_all = false; // received full message
    std::string full_msg;
    while(!received_all && !time_to_exit)
    {
        char msg;
        received_all = serial_port->read_msg(&msg);
        // Fill vector with message bytes until EOL character
        full_msg += msg;
        // Check to see if buffer has been exceeded
        if (full_msg.size() >= BUFFLEN) {
            fprintf(stderr,"\nMessage length exceeds BUFFLEN %i",BUFFLEN);
            return;
        }else if(full_msg.size() > 1 && received_all){
            printf("\n\nRECEIVED: [   %s   ]", full_msg.c_str());
            return;
        }
    }
    printf("\n\nRECEIVED: [   %s   ]", full_msg.c_str());
}


void rwJevois::readMsg()
{
    int newBytes = 0;
    int index = 0;
    int headerBytes = sizeof(struct msg_header);
    int done            = 0;
    int msgBytes        = 0;    // Size of message payload
    int payloadBytes    = 0;    // message payload including checksum
    unsigned char *bf;
    unsigned char *bfPayload;
    unsigned char csum;
    unsigned char msgCsum;
    bool bufFree;
    harrisMessageFP tempHarrisMsg;

    if (serial_port->bytesRead >= BUFFLEN || serial_port->bytesRead < 0) serial_port->bytesRead = 0;
    newBytes = serial_port->read_port(serial_port->buffer.buf + serial_port->bytesRead, BUFFLEN - serial_port->bytesRead);
    newBytes = (newBytes >= (BUFFLEN - serial_port->bytesRead)) ? (BUFFLEN - serial_port->bytesRead) : (newBytes < 0 ? 0 : newBytes);
    if (newBytes > 0){
        serial_port->bytesRead += newBytes;
    }

    while(  (serial_port->bytesRead > index + headerBytes)  &&  // only check message if we have at least header in buffer
            (index < BUFFLEN)                               &&  // index must remain within buffer size limit
            !done ){                                            // only check for message if we haven't found a full one


        // check the buffer for sync bytes -> start of message packet
        if ((serial_port->buffer.buf[index] == SERIAL_SYNC1) &&
            (serial_port->buffer.buf[index+1] == SERIAL_SYNC2)){

            // Pointing to start of buffer
            bf = &(serial_port->buffer.buf[index]);

            memcpy(&tempHarrisMsg.harrisHeader, (char *)bf, headerBytes);

            // Check length of message -> should be less than buffer size
            msgBytes        = tempHarrisMsg.harrisHeader.messageSize;
            payloadBytes    = msgBytes - headerBytes;

            if ( (payloadBytes > 0) && (msgBytes) < BUFFLEN){

                // Continue if entire message has been read from port
                if ( serial_port->bytesRead >= (msgBytes + index)){

                    bfPayload = &(bf[headerBytes]);

                    // Get checksum of message
                    msgCsum = tempHarrisMsg.harrisHeader.csum;

                    csum = calculateCheckSum((unsigned char *)bfPayload, payloadBytes, 0);

                    // verify message passes checksum
                    if (true){//(msgCsum == csum){

                        // Check message is from Jevois
                        switch (tempHarrisMsg.harrisHeader.messageID){
                            case JEVOISMSGID:
                                memcpy(&tempHarrisMsg, (char *)bf, msgBytes);
                                bufFree = harrisMsgBuf.push(tempHarrisMsg);
                                write_count++;

                                // Time msg was parsed
                                using namespace std::chrono;
                                milliseconds time_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                                //dispPreview(tempHarrisMsg, time_ms);
                        }
                    }
                    index += msgBytes;
                } else {
                    done = 1;
                }
            } else {
                index += headerBytes;
            }
        } else{
            index++;
        }
    }
    serial_port->clear_port(index);
}


void rwJevois::dispPreview(const harrisMessageFP msg, std::chrono::milliseconds t)
{
    std::stringstream stream;
    stream << "\n\n\t " << t.count();
    stream << "\n" << write_count << "\t:" << msg.time << "," << (double)msg.sendT/1000 << "," << msg.imageWidth << "," << msg.imageHeight;
    for (int b = 0; b < 10; b++){
        stream << ",(" << msg.fpVal[b] << "," << msg.fpCoord[b][0] << "," << msg.fpCoord[b][1] << ")";
    }
    std::string str = stream.str();
    printf("%s",(stream.str()).c_str());
}


void rwJevois::start()
{
    // Check serial port
    if (serial_port->initStatus != SERIAL_PORT_OPEN ){
        std::string portError = serial_port->getPortID();
        fprintf(stderr,"\nERROR: Serial port %s not open at ", portError.c_str());
        throw 1;
    }

    // Set camera configuration
    init_camera_proc();

    // -------------------------------------------------
    // Start read thread
    // -------------------------------------------------
    const std::string msg{"streamon\0"};
    write_messages(msg);
    usleep(1000);

    printf("\n\nSTART CAM READ THREAD");
    readThread = std::thread(&rwJevois::read_thread,this);
    std::cout << std::flush;

}

void rwJevois::stop()
{
    printf("\n\nCLOSING CAM READ THREADS AND RESTARTING CAMERA");

    //Signal exit
    time_to_exit = true;

    write_messages("streamoff\0");
    usleep(100000);
    write_messages("restart\0");
    usleep(100000);

    if(reading_status){
        reading_status = false;
        if(readThread.joinable()){
            readThread.join();
        }
    }
}


// -------------------------------------------------------------------
//   Quit Handler
// -------------------------------------------------------------------
void rwJevois::handle_quit()
{
    try{
        stop();
    }
    catch (int error) {
        fprintf(stderr,"\nWarning, could not stop Jevois interface\n");
    }
}

void rwJevois::read_thread()
{
    if(reading_status){
        fprintf(stderr,"\nCAM: Read thread already running");
        return;
    }else{
        reading_status = true;
        while(!time_to_exit)
        {
            readMsg();
        }
        reading_status = false;
        return;
    }
}

