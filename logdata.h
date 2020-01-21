#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>

class LogData
{
public:
    LogData(char const *suffix);

    ~LogData();

    void createFilename(char const *suffix);

    void saveData(std::string data);

    void close();

private:
    char *filename;

    std::ofstream fileID;

    std::ofstream *fileID_quit;

};

#endif // LOGDATA_H
