#ifndef API_H
#define API_H
#define EV_API

#ifdef WIN32
    #define _USE_MATH_DEFINES
    #undef EV_API
    #ifdef EXPORT 
        #define EV_API __declspec(dllexport)
    #else 
        #define EV_API __declspec(dllimport)
    #endif
#endif

#endif