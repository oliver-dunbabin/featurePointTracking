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

    Onboard onboard(false);
    onboard_quit = &onboard;

    //onboard.updateOnboard();
    onboard.testCamera();


    return 0;
}

void handle_quit(int sig)
{
    onboard_quit->time_to_exit = true;
}
