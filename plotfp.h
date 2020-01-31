#ifndef PLOTFP_H
#define PLOTFP_H

#include "gnuplot-iostream.h"
#include "shared.h"

class plotFP
{
public:
    plotFP(bool gif);

    void setup_plot();

    void plot(shared *data, int frame);

private:
    Gnuplot est;
    Gnuplot meas;
    bool save_to_gif;
};

#endif // PLOTFP_H
