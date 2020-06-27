#ifndef RWJEVOIS_H
#define RWJEVOIS_H

#include "jevoisserialport.h"
#include "circularbuffer.h"
#include <stdint.h>
#include <chrono>

// Set message header data
#define SERIAL_SYNC1 0xaa
#define SERIAL_SYNC2 0x44
#define JEVOISMSGID 0x96 // Message ID for Harris Detector
// Set message buffer values
#define NUMCORNERS 100  // Number of corners detected in a frame
#define NUMPERBIN 1
#define CBUFLEN 20      // Circular buffer size

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
    uint64_t time;                                      // Time frame was grabbed (since epoch)
    uint32_t sendT;                                     // Time between grab image and send message
    uint16_t imageWidth;                                // Width of input frame
    uint16_t imageHeight;                               // Height of input frame
    uint16_t fpCoord[NUMCORNERS*NUMPERBIN][2];          // Harris feature point coordinate (col * row)
    uint8_t fpVal[NUMCORNERS*NUMPERBIN];                // Harris feature point value
}__attribute__((packed));

// Checksum calculation of message - used to determine if message sent to serial buffer has been corrupted
unsigned char calculateCheckSum(unsigned char *buf, int byteCount, int index);

class rwJevois
{
public:
    rwJevois();
    rwJevois(JevoisSerialPort *serial_port_);
    ~rwJevois();

    uint64_t write_count;
    CircularBuffer<harrisMessageFP> harrisMsgBuf; // Circular buffer which stores messages from Jevois camera

    void read_messages(std::string&); // Read messages arrived at serial port - stops reading upon EOL character

    void read_messages(); // Read messages arrived at serial port - stops reading upon EOL character

    // Write messages to serial port with time argument - poor measure of message send delay
    int write_messages(const std::string &, std::chrono::time_point<std::chrono::system_clock> prevT);

    // Writes string to serial port
    int write_messages(const std::string &);

    void start(); // Start the read thread to continuously read from serial port

    void stop(); // Stop serial port read thread

    bool setJevoisClock(); // Sends current time to serial port

    void handle_quit(); // Function to handle program exit (e.g. ctrl+c)

private:
    bool time_to_exit;  // Flag to exit read thread
    bool reading_status;
    bool writing_status;
    std::thread readThread;

    JevoisSerialPort *serial_port; // Serial port object

    void init_counters();

    void init_camera_proc(); // Send Jevois initialisation values to serial port

    void readMsg(); // Read Jevois Harris Corner messages from serial port buffer, and package them into harrisMessageFP

    void read_thread(); // While loop to read messages from serial buffer - uses readMsg()

    void dispPreview(const harrisMessageFP msg, std::chrono::milliseconds t); // Helper function to debug - displays fraction of serial port data

};

#endif // RWJEVOIS_H
