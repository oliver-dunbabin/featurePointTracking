#include "onboard.h"
#include <signal.h>

void handle_quit(int sig);
Onboard *onboard_quit;

int main()
{
    /*/ Declare sigaction struct for handling exit
    struct s
igaction action;
    // Set the handler function
    action.sa_handler = handle_quit;
    */

    signal(SIGINT, handle_quit);

    Onboard onboard(true);
    onboard_quit = &onboard;
    onboard.updateOnboard();
    //onboard.testSensors();
    //onboard.testCamera();

/*
    Onboard onboard("20-02-04_15-36-12","/home/oliver/MySoftware/featurePointTracking/testdata/", false);
    onboard_quit = &onboard;
    onboard.simulation();
*/
    return 0;
}

void handle_quit(int sig)
{
    onboard_quit->time_to_exit = true;
}
