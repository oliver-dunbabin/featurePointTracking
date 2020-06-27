#include "shared.h"
#include <stdio.h>
#include <string.h>

int main( int argc, char *argv[] ){
    if ( argc < 3 || argc > 3 ){
        if (!strcmp(argv[1], "-O") || !strcmp(argv[1], "-o") || !strcmp(argv[1], "--options")){
            printf("Valid options for FILETYPE:\n"
                   "\tvehicleState:\tthe state of vehicle (body frame)\n"
                   "\tcamMessage:\tthe raw feature point message from camera\n"
                   "\tfpDatabase:\tthe feature point estimates\n\n");
        }else{
            printf("Invalid Input:\t%s\nExectuable should "
                   "be called as:\tFILENAME FILETYPE "
                   "[eg. convertFPBIN data.fpBin vehicleState]\n"
                   "use option -O for valid FILETYPE\n\n", argv[1]);
        }
    }else{
        char path[200];
        strcpy(path, argv[1]);
        char *tempPath = (char *)calloc(strlen(path)+1, sizeof(char));
        strcpy(tempPath,path);

        if (!strcmp(argv[2], "vehicleState")){
            char *name     = strtok(tempPath, ".");
            char filename[200];
            strcpy(filename, name);
            strcat(filename, ".txt");
            FILE *posefile  = fopen(path, "rb");

            if( posefile != nullptr) {
                fprintf(stderr,"opened file for reading:\t%s\n",path);
                FILE *writefile = fopen(filename, "w");
                if( writefile != nullptr){
                    fprintf(stderr,"opened file for writing:\t%s\n",filename);
                    vehicleState vEst;
                    while( !feof(posefile) ) {
                        fread(&vEst, sizeof(vehicleState), 1, posefile);
                        fprintf(writefile, "%lu,%f,%f,%f,%f,%f,%f,%f\n",
                                vEst.timestamp, vEst.pos.X, vEst.pos.Y, vEst.pos.Z,
                                vEst.quat.Q0, vEst.quat.Q1, vEst.quat.Q2, vEst.quat.Q3);
                    }
                    printf("end of file reached for %s\n",argv[1]);
                    fclose(writefile);
                }else{
                    fprintf(stderr,"could not open write file:\t%s\n"
                                   "**use absolute path to file**\n", filename);
                }
                fclose(posefile);
            }else{
                fprintf(stderr,"could not open read file:\t%s\n"
                               "**use absolute path to file**\n", path);
            }
            free(tempPath);
        }else if (!strcmp(argv[2], "camMessage")){
            char *name      = strtok(tempPath, ".");
            char filename[200];
            strcpy(filename, name);
            strcat(filename, ".txt");
            FILE *measfile  = fopen(path, "rb");

            if( measfile != nullptr){
                fprintf(stderr,"opened file for reading:\t%s\n",path);
                FILE *writefile = fopen(filename, "w");
                if( writefile != nullptr){
                    fprintf(stderr,"opened file for writing:\t%s\n",filename);
                    camMessage fpMsg;
                    while( !feof(measfile) ) {
                        fread(&fpMsg, sizeof(camMessage), 1, measfile);
                        fprintf(writefile, "%lu,%lu,%i,",fpMsg.time_stmp,fpMsg.arrivalT,fpMsg.NUMFPS);
                        for (int i = 0; i < fpMsg.NUMFPS; i++){
                            fprintf(writefile,"%f,%f,",fpMsg.fpLocNorm[i][0],fpMsg.fpLocNorm[i][1]);
                        }
                        fprintf(writefile, "\n");
                    }
                    printf("end of file reached for %s\n",argv[1]);
                    fclose(writefile);
                }else{
                    fprintf(stderr,"could not open write file:\t%s\n"
                                   "**use absolute path to file**\n", filename);
                }
                fclose(measfile);
            }else{
                fprintf(stderr,"could not open read file:\t%s\n"
                               "**use absolute path to file**\n", path);
            }
            free(tempPath);
        }else if (!strcmp(argv[2], "fpDatabase")){
            FILE *estfile = fopen(argv[1], "rb");

            if( estfile != nullptr ){
                fpDatalink *fpData;
                    while( !feof(estfile) ) {
                        fread(fpData, sizeof(fpDatalink), 1, estfile);
                    }
                printf("end of file reached for %s\n",argv[1]);
            }else {
                printf("could not open file %s", argv[1]);
            }

            fclose(estfile);
        }else if (!strcmp(argv[2], "-O") || !strcmp(argv[2], "-o") || !strcmp(argv[2], "--options")){
            printf("Valid options for FILETYPE:\n"
                   "\tvehicleState:\tthe state of vehicle (body frame)\n"
                   "\tcamMessage:\tthe raw feature point message from camera\n"
                   "\tfpDatabase:\tthe feature point estimates\n\n");
        }else
            printf("Invalid Input:\t%s\nExectuable should"
                   "be called as:\tFILENAME FILETYPE "
                   "[eg. convertFPBIN data.fpBin vehicleState]\n"
                   "use option -O (or --options) for valid FILETYPEs\n\n", argv[1]);
    }
}
