#include "onboard.h"
#include <signal.h>
#include <string.h>

void handle_quit(int sig);
Onboard *onboard_quit;

int main( int argc, char *argv[] )
{
    int i;
    for(i = 0; i < argc; i++){
        printf("%s ", argv[i]);
    }
    signal(SIGINT, handle_quit);
    bool badInput = false;
    if (argc == 2){
        if (!(strcmp(argv[1], "sensors")) || !(strcmp(argv[1], "-s"))){

            Onboard onboard(false, 0);
            onboard_quit = &onboard;
            onboard.testSensors();
        }else if (!(strcmp(argv[1], "camera")) || !(strcmp(argv[1], "-c"))){

            Onboard onboard(false, 0);
            onboard_quit = &onboard;
            onboard.testCamera();
        }else{
            badInput = true;
        }
    }else if (argc == 3){
        if (!(strcmp(argv[1], "onboard")) || !(strcmp(argv[1], "-o")) ){

            int filterType = 0;         // 0: EKF; 1: UKF
            if (!strcmp(argv[2], "UKF")){

                filterType = 1;
                printf("\nFilter Type:\tUKF\n");
            }else{

                printf("\nFilter Type:\tEKF\n");
            }

            Onboard onboard(true, filterType);
            onboard_quit = &onboard;
            onboard.updateOnboard();
        }else {
            badInput = true;
        }
    }else if (argc == 5){

        int filterType = 0;         // 0: EKF; 1: UKF
        if ((!strcmp(argv[1], "simulation") || !strcmp(argv[1], "-S"))){
            if (!strcmp(argv[4], "UKF")){

                filterType = 1;
                printf("\nFilter Type:\tUKF\n");
            }else{

                printf("\nFilter Type:\tEKF\n");
            }

            Onboard onboard(argv[2],argv[3], false, filterType);
            onboard_quit = &onboard;
            onboard.simulation();
        }else{
            badInput = true;
        }
    }else {

        badInput = true;
    }
    if (badInput){

        printf("INVALID INPUT! Run executable as\n"
               "\tfeatureTracking <function> <option1> <option2> <option3>\n"
               "function:\n"
               "\tonboard (or -o):\tRuns onboard version of mapper\n\t\twith <option1>:\t\"EKF\" or \"UKF\"\n"
               "\tcamera (or -c):\tTests camera. Accepts no options\n"
               "\tsensors (or -s):\tTests RealSense T265 and camera. Accepts no options\n"
               "\tsimulation (or -S:\tRuns filter from saved binary files\n\t\twith\t<option1>:\t\"filename\"\n"
               "\t\t\t<option2>:\t\"filepath\"\n\t\t\t<option3>:\t\"EKF\" or \"UKF\"\n");
    }

    return 0;
}

void handle_quit(int sig)
{
    fprintf(stderr, "Attempting to close threads\n");
    onboard_quit->time_to_exit = true;
}
