#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <stdint.h>
#include <stdio.h>
#include <mutex>
#include <vector>
#include <iostream>

template <class T>
class CircularBuffer
{
private:

    std::vector <T> data;
    std::mutex mu;
    int maxLen;
    int f, b;                       // f - front of buffer; b - back of buffer
    unsigned int n;                 // n - size of current buffer

    void countInc(int &i){
        i++;
        if (i == maxLen){
            i = 0;
        }
    }

    void countDec(int &i){
        i--;
        if (i < 0){
            i = maxLen;
        }
    }

public:

    /*CircularBuffer(){
        maxLen = 1;
        data.resize(maxLen);
        n = 0; f = 0; b = 0;
    }*/

    CircularBuffer(int l){
        if (l > 1){
            maxLen = l;
        }else{
            maxLen = 0;
        }

        data.resize(maxLen);
        n = 0; f = 0; b = 0;
    }

    bool push(T element){
        std::lock_guard<std::mutex> lock(mu);
        //mu.lock();
        if (n == 0){
            data[f] = element;
            countInc(f);
            n++;
            //mu.unlock();
            return true;
        }else if (f != b){
            data[f] = element;
            countInc(f);
            n++;
            //mu.unlock();
            return true;
        }else{
            countInc(b);
            data[f] = element;
            countInc(f);
            //mu.unlock();
            return false;
        }
    }

    bool push(T *element, int l){
        bool safe = true;
        for (int i = 0; i < l; i++){
            if(!push(element[i])){
                safe = false;
            }
        }
        return safe;
    }

    bool pop(T *element){
        if(n > 0){
            std::lock_guard<std::mutex> lock(mu);
            *element = data[b];
            if (n > 1){
                countInc(b);
            }
            n--;
            if(n == 0){
                f = b;
            }
            return 1;
        }
        return 0;
    }

    bool pop(T *element, int l){
        bool safe = true;
        for (int i = 0; i < l; i++){
            if (!pop(&element[i])){
                safe = false;
                break;
            }
        }
        return safe;
    }

    bool copy(T &element){
        if (n > 0){
            std::lock_guard<std::mutex> lock(mu);
            element = data[b];
            return 1;
        }
        return 0;
    }

    size_t size(){
        return n;
    }

    void clear(){
        data.clear();
        n = 0; f = 0; b = 0;
    }
};

#endif // CIRCULARBUFFER_H
