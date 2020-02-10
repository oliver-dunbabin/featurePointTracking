#ifndef LOGDATA_H
#define LOGDATA_H

#include <fstream>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>

#define READ 0
#define TXTWRITE 1
#define BINWRITE 2

class LogData
{
public:
    LogData(std::string parentdir, int operation);

    void createFilename(std::string parentdir);

    void saveData(std::string data);

    void openFile();

    template <typename T>
    void saveBinary(T *data, int bytesize)
    {

        outfile = fopen(filepath.c_str(), "a");
        fwrite(data, bytesize, 1, outfile);
        fclose(outfile);
    }

    const std::string getFilename(){return filename;}

    const std::string getFilepath(){return filepath;}

    void setFilename(std::string name){filename = name;}

    void close();

private:

    std::string filename;

    std::string filepath;

    FILE* outfile = nullptr;

    std::ofstream fileID;

};

#endif // LOGDATA_H
