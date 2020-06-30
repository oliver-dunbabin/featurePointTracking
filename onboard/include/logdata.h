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

// Logfile object used to save binary files - this code needs to be overhauled
class LogData
{
public:
    LogData(std::string parentdir, int operation);

    void createFilename(std::string parentdir);     // Creates binary file filename

    void saveData(std::string data);

    void openFile();                                // Opens binary file

    // Function used to save message of type <T> to binary file
    template <typename T>
    void saveBinary(T *data, int bytesize)
    {

        outfile = fopen(filepath.c_str(), "a");
        if (outfile != nullptr){
            fwrite(data, bytesize, 1, outfile);
            fclose(outfile);
        }else {
            printf("Error opening file:\t%s\n",filepath.c_str());
        }
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
