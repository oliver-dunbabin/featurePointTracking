#include "logdata.h"

LogData::LogData(const char *suffix)
{
    fileID_quit = &fileID;

    createFilename(suffix);

}


LogData::~LogData()
{
    free(filename);
}


void LogData::createFilename(char const *suffix)
{
    time_t now = time(NULL);
    struct tm *info;
    info = localtime(&now);
    char date_time[30];
    strftime(date_time,30,"%d%m%y-%H%M%S",info);
    char *cwd = getcwd(NULL, 0);
    char filepath[100];
    strncpy(filepath, cwd, strlen(cwd));
    free(cwd);
    filepath[strlen(filepath)] = '/';
    strcat(filepath, "testdata/");
    strcat(filepath,date_time);
    strcat(filepath, suffix);
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
    fileID.close();
}
