#include "sortimgbin.h"

sortImgBin::sortImgBin(cv::Mat &img, unsigned int k, unsigned int m, unsigned int n, int p)
{
    imgCols = img.cols;
    imgRows = img.rows;
    numPerBin = p;

    if ((2*m >= imgCols) || (2*m >= imgRows)) {m = min(imgCols/2 - 1, imgRows/2 - 1);}
    boarder = m;
    imgSize = (imgCols - 2*boarder)*(imgRows - 2*boarder);
    if (k > imgSize) { k = imgSize;}
    numFp = k;

    n = (SQ(n) >=imgSize/numFp) ? (sqrtFloor(imgSize/numFp) - 1) : (n < 1) ? 1 : n;
    binSize = n;
    numPerBin = binSize < 2 ? 1 : numPerBin;

    binCol = ((imgCols - 2*boarder) % binSize) ? ((imgCols - 2*boarder)/binSize + 1) : ((imgCols - 2*boarder)/binSize);
    binRow = ((imgRows - 2*boarder) % binSize) ? ((imgRows - 2*boarder)/binSize + 1) : ((imgRows - 2*boarder)/binSize);
    for(unsigned int i = 0; i < binRow; i++){
        for (unsigned int j = 0; j < binCol; j++){
            makeBin(img, i, j);
            int index = i*binCol+j;
            sort(bins[index].begin(),bins[index].end(),sortPixels());
        }
    }
}

void sortImgBin::makeBin(cv::Mat &img,int r, int c)
{
    vector<pixel_val> temp;
    for (unsigned int i = 0; i < binSize; i++){
        for (unsigned int j = 0; j < binSize; j++){
            unsigned int row = r*binSize + i + boarder;
            unsigned int col = c*binSize + j + boarder;
            pixel_val pixel;
            if((row < boarder) || (row > imgRows - boarder) || (col < boarder) || (col > imgCols - boarder)){
                pixel.harrisVal = 0;
                pixel.row = row;
                pixel.col = col;
            }else{
                pixel.harrisVal = img.at<uchar>(row,col);
                pixel.row = row;
                pixel.col = col;
            }
            temp.push_back(pixel);
        }
    }
    bins.push_back(temp);
}


const vector<vector<pixel_val>> sortImgBin::getFp()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(bins.begin(), bins.end(), std::default_random_engine(seed));
    unsigned int numBin = binCol*binRow;
    vector<vector<pixel_val>> maxVal(numBin,vector<pixel_val>(numPerBin));

    for (unsigned int i = 0; i < numBin; i++){
        for (unsigned int j = 0; j < numPerBin; j++)
            maxVal[i][j] = bins[i][j];
    }
    sort(maxVal.begin(), maxVal.end(), sortBins());

    bool exit = false;
    for (unsigned int i = 0; i < numFp; i++){
        vector<pixel_val> temp;
        for (unsigned int j = 0; j < numPerBin; j++){
            if (i*numPerBin + j > numFp){
                exit = true;
                break;
            }
            temp.push_back(maxVal[i][j]);
        }
        topFp.push_back(temp);
        if (exit){break;}
    }
    return topFp;
}


void sortImgBin::getFp(unsigned short fpCoor[][2], unsigned char *fpVal)
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(bins.begin(), bins.end(), std::default_random_engine(seed));
    unsigned int numBin = binCol*binRow;
    vector<vector<pixel_val>> maxVal(numBin,vector<pixel_val>(numPerBin));

    for (unsigned int i = 0; i < numBin; i++){
        for (unsigned int j = 0; j < numPerBin; j++)
            maxVal[i][j] = bins[i][j];
    }
    sort(maxVal.begin(), maxVal.end(), sortBins());

    bool exit = false;
    for (unsigned int i = 0; i < numFp; i++){
        for (unsigned int j = 0; j < numPerBin; j++){
            if (i*numPerBin + j > numFp){
                exit = true;
                break;
            }
            pixel_val temp = maxVal[i][j];
            fpCoor[i*numPerBin+j][0]    = temp.col;
            fpCoor[i*numPerBin+j][1]    = temp.row;
            fpVal[i*numPerBin+j]        = temp.harrisVal;
        }
        if (exit){break;}
    }
}


int sqrtFloor(unsigned int x)
{
    if (x == 0 || x == 1)
        return x;

    int start = 0, end = x/2, ans;
    while (start <= end)
    {
        int mid = (start + end)/2;

        if (SQ(mid) == x)
            return mid;
        if (SQ(mid) < x){
            start = mid+1;
            ans = mid;
        }else
            end = mid - 1;
    }
    return ans;
}


