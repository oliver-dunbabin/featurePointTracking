#ifndef KLARGESTHEAP_H
#define KLARGESTHEAP_H

#include <opencv2/core.hpp>
#include <limits.h>
#include <random>

using namespace std;

struct loc{
    int row;
    int col;
};

struct pixel_val {
    unsigned char harrisVal;
    loc pixLoc;
};

inline int SQ(int x) {return x*x;}
inline int SQ(double x) {return x*x;}

class kLargestHeap
{
public:
    // Constructor: Builds heap array from image, of specified size
    kLargestHeap(cv::Mat&, int);

    // Constructor: Builds heap array of size k from image, of specified size,
    // neglecting the first m pixels around image boarder
    kLargestHeap(cv::Mat& img, int k, int m);

    // Destructor: Releases memory created in the constructor
    ~kLargestHeap();

    struct sortPixels
    {
        inline bool operator() (const pixel_val &a, const pixel_val &b)
        {
            return (a.harrisVal > b.harrisVal);
        }
    };

    struct lessThanPix
    {
        inline bool operator() (const pixel_val &a, const pixel_val &b)
        {
            return (a.harrisVal < b.harrisVal);
        }
    };

    // Sorts the initial array of k pixel values
    void heapSort();

    // Returns the current heap
    pixel_val* getHeap();

    // Gets the root key of the heap - in this case, the lowest value
    pixel_val getHeapRoot(){return heap_array[0];}

    // Gets the size of heap
    int getHeapSize(){return heap_size;}

    int getIterator(){return iterator;}

    // Remove the root element from the heap
    void swapRoot(cv::Mat& img, int);

    // Swap elements in an array
    void swap(pixel_val*, pixel_val*);

private:
    int heap_size;  // The size of the heap as defined at construction (k)
    int imgCols;    // Number of columns of input image
    int imgRows;    // Number of rows of the input image
    int boarder;    // Number of pixels to exclude around image boarders
    int separation;
    int iterator;
    pixel_val* heap_array;  // Pointer to zeroth element of heap array


    // Heapify the subtree with root at index i.
    // The parent's subtrees are already heapified
    void minHeapify(int);

    bool addToHeap(pixel_val *next_pixel, int it);

    // Returns the parent index for the ith node
    int iParent(int i) {return (i-1)/2;}

    // Returns the left child's index for the ith node
    int iLeft(int i) {return (2*i+1);}

    // Returns the right child's index for the ith node
    int iRight(int i) {return (2*i+2);}
};

#endif // KLARGESTHEAP_H
