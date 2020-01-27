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
    LogData(char const *suffix, int bytesize, bool createName);

    ~LogData();

    void createFilename(char const *suffix, int bytesize);

    void saveData(std::string data);

    template <typename T>
    void saveBinary(T *data, int bytesize)
    {

        outfile = fopen(filename, "a");
        fwrite(data, bytesize, 1, outfile);
        fclose(outfile);
    }

    void close();

private:

    char *filename;

    FILE* outfile = nullptr;

    std::ofstream fileID;

};

#endif // LOGDATA_H
