#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include <stdint.h>
#include <stdio.h>
#include <mutex>
#include <vector>
#include <iostream>

/**
 * Class which defines a template for a circular buffer
 * Front (f) of buffer is last in element (newest)
 * Back (b) of buffer is first in element (oldest)
 * When size of buffer (n) exceeds maxLen, newer elements write over back (b)
 */
template <class T>
class CircularBuffer
{
private:

    std::vector <T> data;   // Vector of elements of type <T>
    std::mutex mu;
    int maxLen;             // Max allowable size of buffer
    int f, b;               // f - front of buffer; b - back of buffer
    unsigned int n;         // n - size of current buffer

    void countInc(int &i){  // Increments position of counter within circular buffer (i.e. increments counter to position n-1, then returns to position 0)
        i++;
        if (i == maxLen){
            i = 0;
        }
    }

public:

    CircularBuffer(int l){
        if (l > 1){
            maxLen = l;
        }else{
            maxLen = 0;
        }

        data.resize(maxLen);
        n = 0; f = 0; b = 0;
    }

    // Push element into circular buffer
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

    // Push array of elements (of length l) into circular buffer
    bool push(T *element, int l){
        bool safe = true;
        for (int i = 0; i < l; i++){
            if(!push(element[i])){
                safe = false;
            }
        }
        return safe;
    }

    // Remove element from back of circular buffer (and save it to object *element)
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

    // Remove l elements from back of circular buffer (and save them to array *elements)
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

    // Copy element from back of circular buffe
    bool copy(T &element){
        if (n > 0){
            std::lock_guard<std::mutex> lock(mu);
            element = data[b];
            return 1;
        }
        return 0;
    }

    // Return current size of circular buffer
    size_t size(){
        return n;
    }

    // Clear all elements from circular buffer
    void clear(){
        data.clear();
        n = 0; f = 0; b = 0;
    }
};

#endif // CIRCULARBUFFER_H
