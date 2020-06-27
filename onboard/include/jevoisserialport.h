#ifndef JEVOISSERIALPORT_H
#define JEVOISSERIALPORT_H

#include <string>
#include <cstring>
#include <sys/time.h>
#include <sys/termios.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <sys/ioctl.h>

// Status flags
#define SERIAL_PORT_OPEN    1
#define SERIAL_PORT_CLOSED  0
#define SERIAL_PORT_ERROR   -1
#define BUFFLEN 100*1024

//using namespace std;

struct msg{
    unsigned char buf[BUFFLEN];
    unsigned long time;
}__attribute__((packed));

// Serial port class - TO DO: This class is old and needs to be cleaned up somewhat
class JevoisSerialPort
{
public:
    JevoisSerialPort(int, std::vector<std::string>, bool);          // Initialise specific parameters
    JevoisSerialPort();                                             // Initialise defaults
    ~JevoisSerialPort();

    void init();                                                    // Initialise serial connection
    void deInit();

    void setPortID(const std::vector<std::string>* );               // Set host port
    std::string getPortID(){return port;}                           // Retrieve portID

    void setBaudRate(int baud);                                     //  Set rate of serial data
    int getBaudRate(){return baudRate;}                             // Retrieve baud rate

    int file;                                                       // File descriptor for serial connection with autopilot

    int read_msg(char *);                                           // Reads byte from serial port (returns success if encounters EOL character)
    int read_port(void *pmsg, int numBytes);                        // Reads numBytes or all bytes from serial buffer, whichever is smaller
    void clear_port(int index);                                     // Clears serial port up to element index

    int write_msg(const std::string &);                             // Write string to serial port

    void start();                                                   // Initializes serial port - TO DO: Unnecessary to have start() and init()
    void stop();                                                    // Closes serial port - TO DO: Unnecessary to have stop() and deInit()

    void handle_quit();                                             // Helper function to close serial port on program exit (e.g. ctrl+c)

    int bytesRead;
    int initStatus;
    int received;
    msg buffer;             // buffer of bytes which arrive at serial port

private:
    void initialise_defaults();                 // If no user input, initialise port to default setting
    int _open_port(std::string * );             // Opens the serial port
    bool _setup_port(int );                     // Sets the attributes/options for serial port

    //std::mutex mutex;
    //std::thread *reading_thread;
    int baudRate;                               // Rate of serial data
    std::vector<std::string> portID;            // Host port to which autopilot is connected
    std::string port;
    //int file;                                 // File descriptor for serial connection with autopilot
    bool debug;

};


#endif // JEVOISSERIALPORT_H
