#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>

#define DEBUG_ 0
#define D_COUT(x) \
{\
    if(DEBUG_) \
       std::cout<<x<<std::endl; \
}

#define D_COUT2(x, y) \
{\
    if(DEBUG_) \
       std::cout<<x<<y<<std::endl; \
}

#endif
