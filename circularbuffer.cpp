#include "circularbuffer.h"
/*
template<class T>
CircularBuffer<T>::CircularBuffer()
{
    maxLen = 1;
    data.resize(maxLen);
    n = 0; f = 0; b = 0;
}

template<class T>
CircularBuffer<T>::CircularBuffer(int l){
    if (l > 1){
        maxLen = l;
    }else{
        maxLen = 0;
    }

    data.resize(maxLen);
    n = 0; f = 0; b = 0;
}

template<class T>
void CircularBuffer<T>::countInc(int &i){
    i++;
    if (i == maxLen){
        i = 0;
    }
}

template<class T>
void CircularBuffer<T>::countDec(int &i){
    i--;
    if (i < 0){
        i = maxLen;
    }
}

template<class T>
bool CircularBuffer<T>::push(T element){
    mu.lock();
    if (n == 0){
        data[f] = element;
        countInc(f);
        n++;
        mu.unlock();
        return true;
    }else if (f != b){
        data[f] = element;
        countInc(f);
        n++;
        mu.unlock();
        return true;
    }else{
        countInc(b);
        data[f] = element;
        countInc(f);
        mu.unlock();
        return false;
    }
}

template<class T>
bool CircularBuffer<T>::push(T *element, int l){
    bool safe = true;
    for (int i = 0; i < l; i++){
        if(!push(element[i])){
            safe = false;
        }
    }
    return safe;
}

template<class T>
bool CircularBuffer<T>::pop(T *element){
    if(n > 0){
        mu.lock();
        *element = data[b];
        if (n > 1){
            countInc(b);
        }
        n--;
        if(n == 0){
            f = b;
        }
        mu.unlock();
        return 1;
    }
    return 0;
}

template<class T>
bool CircularBuffer<T>::pop(T *element, int l){
    bool safe = true;
    for (int i = 0; i < l; i++){
        if (!pop(&element[i])){
            safe = false;
            break;
        }
    }
    return safe;
}

template<class T>
bool CircularBuffer<T>::copy(T &element){
    if (n > 0){
        mu.lock();
        element = data[b];
        mu.unlock();
        return 1;
    }
    return 0;
}
*/
