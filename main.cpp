#include "onboard.h"
#include <signal.h>
#include <iostream>

void handle_quit(int sig);
Onboard *onboard_quit;

int main()
{
    Onboard onboard(false);
    onboard_quit = &onboard;
    signal(SIGINT, handle_quit);

    onboard.updateOnboard();

    return 0;
}

void handle_quit(int sig)
{
    std::cout << "\n\nTERMINATING ALL PROCESSES AT USER REQUEST with code: " << sig << std::endl;

    try {
        onboard_quit->handle_quit(sig);
    }
    catch (int error){
        fprintf(stderr,"\nWarning, could not complete SAFE EXIT procedure\n");
    }
    exit(0);
}
