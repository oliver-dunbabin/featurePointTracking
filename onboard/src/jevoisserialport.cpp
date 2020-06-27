#include "jevoisserialport.h"

// -------------------------------------------------------------------------------------------------
// Constructor/ Destructor
// -------------------------------------------------------------------------------------------------
JevoisSerialPort::JevoisSerialPort(int bR, std::vector<std::string> pID, bool db)
{
    initialise_defaults();
    baudRate = bR;
    portID = pID;
    debug = db;
}

JevoisSerialPort::JevoisSerialPort()
{
    initialise_defaults();
}

void JevoisSerialPort::initialise_defaults()
{
    baudRate = 9600;
    portID.push_back("/dev/ttyACM0");
    file = -1;
    initStatus = SERIAL_PORT_CLOSED;

    debug = false;
}

JevoisSerialPort::~JevoisSerialPort()
{}


// -------------------------------------------------------------------------------------------------
// Initialise serial port (open port and set options)
// -------------------------------------------------------------------------------------------------
void JevoisSerialPort::init()
{
    printf("\n\nOPENING PORT");
    std::string port = "\0";
    for (auto& x : portID)
    {
        file = _open_port(&x);

        if (file == -1){
            fprintf(stderr,"\n\nDevice not connected to %s", x.c_str());
            continue;
        }
        else{
            printf("\n\nDevice connected to %s", x.c_str());
            port = x;
            break;
        }
    }

    bool setupSuccess = _setup_port(baudRate);

    if(!setupSuccess){
        fprintf(stderr,"\n\nCould not configure port %s\n", port.c_str());
        throw EXIT_FAILURE;
    }

    if(file <= 0){
        fprintf(stderr,"\n\nConnection attempt to port %s with baudrate %i FAILED\n", port.c_str(), baudRate);
        throw EXIT_FAILURE;
    }

    printf("\n\nCONNECTED to port %s with baudrate %i", port.c_str(), baudRate);

    initStatus = SERIAL_PORT_OPEN;

    return;
}


// -------------------------------------------------------------------------------------------------
// Close the serial port
// -------------------------------------------------------------------------------------------------
void JevoisSerialPort::deInit()
{
    printf("\n\nCLOSING CAMERA SERIAL PORT");
    if (file < 0){
        fprintf(stderr,"\n\nNo serial connection exists");
        return;
    }

    int result = close(file);

    if(result){
        fprintf(stderr, "\n\nWARNING: Error on port close (%i)", result);
    }

    initStatus = SERIAL_PORT_CLOSED;
}


// -------------------------------------------------------------------
//   Convenience functions
// -------------------------------------------------------------------
void JevoisSerialPort::start()
{
    init();
}

void JevoisSerialPort::stop()
{
    if(initStatus)
        deInit();
}

// -------------------------------------------------------------------
//   Quit Handler
// -------------------------------------------------------------------
void JevoisSerialPort::handle_quit()
{

    try{
        stop();
    }
    catch (int error) {
        fprintf(stderr,"\n\nWarning, could not stop serial interface");
    }
}


// -------------------------------------------------------------------------------------------------
// Open the serial port file descriptor
// -------------------------------------------------------------------------------------------------
int JevoisSerialPort::_open_port(std::string *port)
{
    // O_RDWR - Read and Write
    // O_NOCTTY - Ignore special chars like ctrl-c
    // O_NONBLOCK - prevents open from blocking for a long time
    file = open(port->c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(file == -1){
        /* Could not open port*/
        return(-1);
    }
    else{
        // Manipulates file descriptor - sets file flag to 0
        fcntl(file,F_SETFL,0);
    }
    return file;
}


// -------------------------------------------------------------------------------------------------
// Setup the ports configuration
// -------------------------------------------------------------------------------------------------
bool JevoisSerialPort::_setup_port(int bR)
{

    // Check file descriptor
    if(!isatty(file)){
        fprintf(stderr,"\n\nERROR: file descriptor %d is NOT a serial port", file);
        return false;
    }

    // Read file descriptor configuration
    struct termios config;
    if(tcgetattr(file,&config) < 0){
        fprintf(stderr,"\n\nERROR: could not read configuration of file %d", file);
                return false;
    }

    printf("\n\nSerial port attributes read.");

    /*config.c_cflag = (config.c_cflag & ~CSIZE) | CS8;
    config.c_iflag &= ~IGNBRK;
    config.c_lflag = 0;

    config.c_oflag = 0;
    config.c_cc[VMIN] = 0;
    config.c_cc[VMIN] = 5;

    config.c_iflag &= ~(IXON | IXOFF | IXANY);
    config.c_cflag |= (CLOCAL | CREAD);

    config.c_cflag &= ~(PARENB | PARODD);
    config.c_cflag |= 0;
    config.c_cflag &= ~CSTOPB;
    config.c_cflag &= ~CRTSCTS;*/


    // Set input flags - turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    //config.c_iflag &= ~(IGNBRK|BRKINT|ICRNL|INLCR|PARMRK|INPCK|ISTRIP);
    config.c_iflag |= IGNPAR;
    config.c_iflag &= ~IGNBRK;
    config.c_iflag &= ~(IXON | IXOFF | IXANY);


    // Set output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR supression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //config.c_oflag &= ~(OCRNL|ONLCR|ONLRET|ONOCR|OFILL|OPOST);
    config.c_oflag = 0;

    /*#ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif*/


    // Set local Options
    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    //config.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN|ISIG);
    config.c_lflag = 0;  // set local mode (non-canonical, no echo,...)

    // Set Control Options
    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    //config.c_cflag &= ~(CSIZE|PARENB);
    //config.c_cflag |= 0;
    config.c_cflag |= (CLOCAL|CREAD);
    config.c_cflag &= ~CSIZE;       // NEWLY ADDED
    config.c_cflag &= ~PARENB;      // NEWLY ADDED
    config.c_cflag &= ~CSTOPB;      // NEWLY ADDED
    config.c_cflag &= ~CRTSCTS;     // NEWLY ADDED
    config.c_cflag |= CS8;


    // Set Control Character Options
    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN] = 0;
    config.c_cc[VTIME] = 5;

    // Set the Baudrate
    // set baudrate according to object initialisation (default: 9600)
    speed_t baud;
    switch(bR){
    case 9600: baud = B9600;
        break;
    case 57600: baud = B57600;
        break;
    case 115200: baud = B115200;
        break;
    case 921600: baud = B921600;
        break;
    default: baud = B9600;
    }
    cfsetspeed(&config, baud);

    // set attributes
    usleep(20000);
    if (tcsetattr(file,TCSANOW|TCSAFLUSH,&config) < 0){
        fprintf(stderr,"\n\nERROR: could no set configuration of file %d",file);
        return false;
    }
    usleep(20000);
    tcflush(file,TCIOFLUSH);
    return true;
}


// -------------------------------------------------------------------------------------------------
// Start reading from port
// -------------------------------------------------------------------------------------------------
int JevoisSerialPort::read_msg(char *pmsg)
{
    uint8_t msgReceived = false;

    int result = read(file,pmsg,1);

    if (result > 0){
        if (*pmsg == '\n' || *pmsg == '\r' || *pmsg == '\0')
            return msgReceived = 1;
        else
            return msgReceived = 0;
    } else{
        fprintf(stderr,"\n\nERROR: Could not read from file %d",file);
        return -1;
    }
}


int JevoisSerialPort::read_port(void *pmsg, int numBytes)
{
    int availBytes = 0;
    ioctl(file, FIONREAD, &availBytes);
    if (availBytes > 0){
        availBytes = std::min(availBytes, numBytes);
        availBytes = read(file, (char *)pmsg, availBytes);
        received += std::max(0,availBytes);
    }
    return availBytes;
}


void JevoisSerialPort::clear_port(int index)
{
    if (initStatus){
        if (index > 0){
            if (index >= bytesRead){
                bytesRead = 0;
            }else{
                memmove(buffer.buf, &(buffer.buf[index]), bytesRead - index);
                bytesRead -= index;
            }
        }else{
            bytesRead = 0;
        }
    }
}



// -------------------------------------------------------------------------------------------------
// Write to open port
// -------------------------------------------------------------------------------------------------
int JevoisSerialPort::write_msg(const std::string &msg)
{
    int len = msg.length();
    char buffer[len+1];
    msg.copy(buffer,len,0);
    buffer[len] = '\0';

    // Write buffer to serial port
    int bytesWritten = static_cast<int>(write(file,buffer,len+1));
    tcdrain(file);

    return bytesWritten;
}
