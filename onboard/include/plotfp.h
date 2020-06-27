#ifndef PLOTFP_H
#define PLOTFP_H

#include "gnuplot-iostream.h"
#include "shared.h"

// Object used to display fp estimates when run on computer using GNU Plot
// TO DO: Pretty primative - can replace with more sophisticated plotting software
class plotFP
{
public:
    plotFP(bool gif);

    void setup_plot();  // Sets GNU Plot and figure parameters

    void plot(shared *data, int frame);     // Plots predicted measurements against actual and vehicle location with fp estimates

private:
    Gnuplot est;    // GNU object to plot fp estimates
    Gnuplot meas;   // GNU object to plot measurements
    bool save_to_gif;   // Flag to record plots to GIF for debugging purposes
};

#endif // PLOTFP_H
