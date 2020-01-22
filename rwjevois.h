#ifndef RWJEVOIS_H
#define RWJEVOIS_H

#include "jevoisserialport.h"
#include "circularbuffer.h"
#include <stdint.h>
#include <chrono>

#define SERIAL_SYNC1 0xaa
#define SERIAL_SYNC2 0x44
#define JEVOISMSGID 0x96
#define SERIAL_SEEK_SYNC1 0
#define SERIAL_SEEK_SYNC2 1
#define SERIAL_SEEK_ID 2
#define SERIAL_SEEK_SIZE 3
#define SERIAL_SEEK_DATA 4
#define NUMCORNERS 100
#define NUMPERBIN 1

// Message format
struct msg_header {
    char sync1;                     // First message sync byte
    char sync2;                     // Second message sync byte
    unsigned char messageID;        // ID of Harris Corner message (set to any int)
    unsigned char csum;             // Message checksum
    uint32_t messageSize;           // Size of message being received
}__attribute__((packed));

struct harrisMessageFP {
    harrisMessageFP()   {harrisHeader.sync1 = SERIAL_SYNC1;
                        harrisHeader.sync2 = SERIAL_SYNC2;
                        harrisHeader.messageID = JEVOISMSGID;}
    msg_header harrisHeader;
    uint64_t time;                                     // Time frame was grabbed (since epoch)
    uint16_t imageWidth;                                // Width of input frame
    uint16_t imageHeight;                               // Height of input frame
    uint16_t fpCoord[NUMCORNERS*NUMPERBIN][2];          // Harris feature point coordinate (col * row)
    unsigned char fpVal[NUMCORNERS*NUMPERBIN];          // Harris feature point value
}__attribute__((packed));

// Checksum calculation
unsigned char calculateCheckSum(unsigned char *buf, int byteCount, int index);

class rwJevois
{
public:
    rwJevois();
    rwJevois(JevoisSerialPort *serial_port_);
    ~rwJevois();

    uint64_t write_count;
    CircularBuffer<harrisMessageFP> harrisMsgBuf;

    void read_messages(std::string&);

    void read_messages();

    int write_messages(const std::string &, std::chrono::time_point<std::chrono::system_clock> prevT);

    int write_messages(const std::string &);

    void start();

    void stop();

    bool setJevoisClock();

    void handle_quit();

private:
    bool time_to_exit;
    bool reading_status;
    bool writing_status;
    std::thread readThread;

    JevoisSerialPort *serial_port;

    void init_counters();

    void init_camera_proc();

    void readMsg();

    void read_thread();

};

#endif // RWJEVOIS_H
