#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>

#define READ 1
#define WRITE 0

class LogData
{
public:
    LogData(std::string parentdir, int operation);

    void createFilename(std::string parentdir);

    void saveData(std::string data);

    template <typename T>
    void saveBinary(T *data, int bytesize)
    {

        outfile = fopen(filename.c_str(), "a");
        fwrite(data, bytesize, 1, outfile);
        fclose(outfile);
    }

    const std::string getFilename(){return filename;}

    void close();

private:

    std::string filename;

    FILE* outfile = nullptr;

    std::ofstream fileID;

};

#endif // LOGDATA_H
