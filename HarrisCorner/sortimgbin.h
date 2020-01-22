#ifndef SORTIMGBIN_H
#define SORTIMGBIN_H

#include <opencv2/core.hpp>
#include <iostream>
#include <bits/stdc++.h>

using namespace std;

struct pixel_val {
    unsigned short row;
    unsigned short col;
    unsigned char harrisVal; 
}__attribute__((packed));

inline unsigned int SQ(int x) {return x*x;}

int sqrtFloor(unsigned int x);


class sortImgBin
{
public:
    sortImgBin(cv::Mat& img, unsigned int k, unsigned int m, unsigned int n, int p);

    struct sortPixels
    {
        inline bool operator() (const pixel_val &a, const pixel_val &b)
        {
            return (a.harrisVal > b.harrisVal);
        }
    };

    struct sortBins
    {
        inline bool operator() (const vector<pixel_val> &a, const vector<pixel_val> &b)
        {
            return (a[0].harrisVal > b[0].harrisVal);
        }
    };

    void setNumBin(int num){numPerBin = num;}

    const vector<vector<pixel_val> > getFp();

    void getFp(unsigned short fpCoor[][2], unsigned char *fpVal);

    unsigned int numPerBin;  // Number of fp allowed in each bin

    unsigned int getnumFp(){return numFp;}

private:
    unsigned int imgCols;    // Number of columns of input image
    unsigned int imgRows;    // Number of rows of the input image
    unsigned int boarder;    // Number of pixels to exclude around image boarders
    unsigned int binSize;    // Size of bin
    unsigned int numFp;      // Number of fp extracted from image
    unsigned int imgSize;    // Size of image area of interest
    unsigned int binCol;     // Number of bins in column
    unsigned int binRow;     // Number of bins in row
    vector<vector<pixel_val>> bins;
    vector<vector<pixel_val>> topFp;

    void makeBin(cv::Mat &img,int r, int c);
};

#endif // SORTIMGBIN_H
