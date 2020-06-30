#include "onboard.h"
#include <signal.h>
#include <string.h>

void handle_quit(int sig);
Onboard *onboard_quit;

int main( int argc, char *argv[] )
{

    signal(SIGINT, handle_quit);
    bool badInput = false;
    if (argc == 3){

        if (!(strcmp(argv[1], "updateOnboard")) ){

            int filterType = 0;         // 0: EKF; 1: UKF
            if (!strcmp(argv[2], "UKF")){

                filterType = 1;
                printf("Filter Type:\tUKF\n");
            }else{

                printf("Filter Type:\tEKF\n");
            }

            Onboard onboard(true, filterType);
            onboard_quit = &onboard;
            onboard.updateOnboard();
        }else if (!(strcmp(argv[1], "testSensors"))){

            Onboard onboard(false, 0);
            onboard_quit = &onboard;
            onboard.testSensors();
        }else if (!(strcmp(argv[1], "testCamera"))){

            Onboard onboard(false, 0);
            onboard_quit = &onboard;
            onboard.testCamera();
        }else {
            badInput = true;
        }
    }else if (argc == 4 && !strcmp(argv[1], "simulation")){

        int filterType = 0;         // 0: EKF; 1: UKF
        if (!strcmp(argv[4], "UKF")){

            filterType = 1;
            printf("Filter Type:\tUKF\n");
        }else{

            printf("Filter Type:\tEKF\n");
        }

        Onboard onboard(argv[2],argv[3], false, filterType);
        onboard_quit = &onboard;
        onboard.simulation();
    }else {

        badInput = true;
    }
    if (badInput){

        printf("Relevant error message");
    }

    return 0;
}

void handle_quit(int sig)
{
    fprintf(stderr, "Attempting to close threads\n");
    onboard_quit->time_to_exit = true;
}
