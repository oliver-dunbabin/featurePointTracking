#include "logdata.h"

LogData::LogData(const char *suffix, int bytesize, bool createName)
{

    if (createName){
        createFilename(suffix, bytesize);
    }else{
        filename = (char *)malloc(bytesize);
        strncpy(filename, suffix, bytesize);
    }
}


LogData::~LogData()
{
    free(filename);
}


void LogData::createFilename(char const *suffix, int bytesize)
{

    time_t now = time(NULL);
    struct tm *info;
    info = localtime(&now);
    char date_time[30];
    strftime(date_time,30,"%y-%m-%d_%H-%M-%S_",info);
    char *cwd = getcwd(NULL, 0);
    char filepath[100];
    strncpy(filepath, cwd, strlen(cwd));
    free(cwd);
    filepath[strlen(filepath)] = '/';
    strcat(filepath, "testdata/");
    char folder[30];
    strncpy(folder, suffix, bytesize);
    strcat(folder, "/");
    strcat(filepath, folder);
    strcat(filepath,date_time);
    strcat(filepath, suffix);
    strcat(filepath, ".fpBIN");
    filename = (char *)malloc(strlen(filepath));
    strncpy(filename,filepath, strlen(filepath));

}


void LogData::saveData(std::string data)
{

    fileID.open(filename, std::ofstream::out | std::ofstream::app);

    if(!fileID.is_open()){
        std::cout << "Could not open file" << std::endl;
        exit(1);
    }

    fileID << data;
    fileID << '\n';
    fileID.close();

}


void LogData::close()
{

    if(fileID.is_open())
        fileID.close();

    if (outfile != nullptr)
        fclose(outfile);

}
