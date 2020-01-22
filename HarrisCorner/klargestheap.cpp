#include "klargestheap.h"

// Constructor: Builds heap array from image, of specified size
kLargestHeap::kLargestHeap(cv::Mat& img, int k)
{
    imgCols = img.cols;
    imgRows = img.rows;
    if (k > imgCols * imgRows) { k = imgCols * imgRows;}
    boarder = 0;
    heap_size = k;
    heap_array = new pixel_val[heap_size];

    // Fill heap with k randomly generated pixels
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> disCol(0, imgCols - 1);
    uniform_int_distribution<> disRow(0, imgRows - 1);

    for (int i = 0; i<k;i++)
    {
        int r = disCol(gen);
        int c = disRow(gen);
        heap_array[i].harrisVal     = img.at<uchar>(r,c);
        heap_array[i].pixLoc.col    = c;
        heap_array[i].pixLoc.row    = r;
    }
    iterator = heap_size;
    heapSort();
}


// Constructor: Builds heap array of size k from image, of specified size,
// neglecting the first m pixels around image boarder
kLargestHeap::kLargestHeap(cv::Mat& img, int k, int m)
{
    imgCols = img.cols;
    imgRows = img.rows;
    if (k > imgCols * imgRows) { k = imgCols * imgRows;}
    if ((2*m >= imgCols) || (2*m >= imgRows)) {m = min(imgCols/2 - 1, imgRows/2 - 1);}
    boarder = m;
    heap_size = k;
    heap_array = new pixel_val[heap_size];

    // Fill heap with k randomly generated pixels
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> disCol(boarder, imgCols - boarder - 1);
    uniform_int_distribution<> disRow(boarder, imgRows - boarder - 1);

    for (int i = 0; i<k;i++)
    {
        int r = disCol(gen);
        int c = disRow(gen);
        heap_array[i].harrisVal     = img.at<uchar>(r,c);
        heap_array[i].pixLoc.col    = c;
        heap_array[i].pixLoc.row    = r;
    }
    iterator = heap_size;
    heapSort();
}


// Destructor: Releases memory created in the constructor
kLargestHeap::~kLargestHeap()
{
    delete[] heap_array;
    heap_array = nullptr;
}


// Heapify the subtree with root at index i.
// The parent's subtrees are already heapified
void kLargestHeap::minHeapify(int i)
{
    int l = iLeft(i);
    int r = iRight(i);
    int smallest = i;
    if (l < heap_size && heap_array[l].harrisVal < heap_array[i].harrisVal )
        smallest = l;
    if (r < heap_size && heap_array[r].harrisVal < heap_array[smallest].harrisVal)
        smallest = r;
    if (smallest != i)
    {
        swap(&heap_array[i], &heap_array[smallest]);
        minHeapify(smallest);
    }
}


// Sorts the initial array of k pixel values
void kLargestHeap::heapSort()
{
    for (int i = (heap_size)/2 - 1 ; i>=0; i--)
        minHeapify(i);
}


// Remove the root element from the heap
void kLargestHeap::swapRoot(cv::Mat& img, int k)
{
    int r = k/(imgCols-2*boarder);
    int c = k - (imgCols-2*boarder)*r;
    int row = r + boarder;
    int col = c + boarder;
    uchar img_val = img.at<uchar>(row, col);
    if (img_val  > heap_array[0].harrisVal)
    {
        pixel_val root;
        root.harrisVal = img_val;
        root.pixLoc.col= col;
        root.pixLoc.row= row;
        heap_array[0] = root;
        minHeapify(0);
    }
}


// Swap elements in an array
void kLargestHeap::swap(pixel_val *x, pixel_val *y)
{
    pixel_val temp = *x;
    *x = *y;
    *y = temp;
}


bool kLargestHeap::addToHeap(pixel_val *next_pixel, int it)
{
    pixel_val minValue;
    minValue.harrisVal = 255;
    int minLoc;
    bool append = true;
    for (int i = 0; i < it; i++){
        if (SQ(heap_array[i].pixLoc.col - next_pixel->pixLoc.col) +
                SQ(heap_array[i].pixLoc.row - next_pixel->pixLoc.row) <= SQ(separation)){
            if ((heap_array[i].harrisVal < next_pixel->harrisVal) && (heap_array[i].harrisVal <= minValue.harrisVal)){
                minLoc = i;
                minValue = heap_array[i];
                append = false;
            }
        }
    }
    if(append)
        minLoc = it;
    heap_array[minLoc] = *next_pixel;
    return append;
}


pixel_val* kLargestHeap::getHeap(){
    sort(heap_array, heap_array + heap_size, sortPixels());
    return heap_array;
}

