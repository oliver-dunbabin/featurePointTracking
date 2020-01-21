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

class JevoisSerialPort
{
public:
    JevoisSerialPort(int, std::vector<std::string>, bool);            // Initialise specific parameters
    JevoisSerialPort();                               // Initialise defaults
    ~JevoisSerialPort();

    void init();                                // Initialise serial connection with autopilot
    void deInit();

    void setPortID(const std::vector<std::string>* );     // Set host port to which autopilot is connected
    std::string getPortID(){return port;}  // Retrieve portID

    void setBaudRate(int baud);                 //  Set rate of serial data
    int getBaudRate(){return baudRate;}         // Retrieve baud rate

    int file;                                   // File descriptor for serial connection with autopilot

    int read_msg(char *);
    int read_port(void *pmsg, int numBytes);
    void clear_port(int index);

    int write_msg(const std::string &);

    void start();
    void stop();

    void handle_quit(int);

    int bytesRead;
    int initStatus;
    int received;
    char seekState;
    msg buffer;

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
