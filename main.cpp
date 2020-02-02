#include "onboard.h"
#include <signal.h>
#include <iostream>

void handle_quit(int sig);
Onboard *onboard_quit;

int main()
{
    /*/ Declare sigaction struct for handling exit
    struct sigaction action;
    // Set the handler function
    action.sa_handler = handle_quit;
    */

    signal(SIGINT, handle_quit);

    /*Onboard onboard(true);
    onboard_quit = &onboard;
    onboard.updateOnboard();
    //onboard.testSensors();
    //onboard.testCamera();*/

    Onboard onboard("20-01-31_02-11-57","/home/oliver/MySoftware/featurePointTracking/testdata/", true);
    onboard.simulation();

    return 0;
}

void handle_quit(int sig)
{
    onboard_quit->time_to_exit = true;
}
