#include "logdata.h"

LogData::LogData(std::string parentdir, int operation)
{

    if (operation == WRITE){
        createFilename(parentdir);
    }else{
        filename = parentdir;
    }
    //printf("file created at: %s\n", filename);
}


void LogData::createFilename(std::string parentdir)
{

    time_t now = time(NULL);
    struct tm *info;
    info = localtime(&now);
    char date_time[30];
    strftime(date_time,30,"%y-%m-%d_%H-%M-%S",info);
    std::string filetime(date_time);


    filename +=  parentdir + "/" + filetime + ".fpBIN";
}


void LogData::saveData(std::string data)
{

    fileID.open(filename, std::ofstream::out | std::ofstream::app);

    if(!fileID.is_open()){
        printf("\n\nCould not open file");
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
