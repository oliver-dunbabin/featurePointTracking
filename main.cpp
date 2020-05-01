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

    int filterType = 0;     // 0: EKF; 1: UKF

//    Onboard onboard(true, filterType);
//    onboard_quit = &onboard;
//    onboard.updateOnboard();
    //onboard.testSensors();
    //onboard.testCamera();

  Onboard onboard("20-04-04_12-54-11","/home/oliver/MySoftware/featurePointTracking/testdata/", false, filterType);
  onboard_quit = &onboard;
  onboard.simulation();


    return 0;
}

void handle_quit(int sig)
{
    fprintf(stderr, "Attempting to close threads\n");
    onboard_quit->time_to_exit = true;
}
