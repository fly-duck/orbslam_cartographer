#pragma once 

#include <chrono>
#include <iostream>

namespace BenchMark
{
    
class Timer
{


public:

    Timer()
    {
        start_timestamped_=std::chrono::high_resolution_clock::now();

    }


    void Stop () 
    {
        auto end_timestamped=std::chrono::high_resolution_clock::now();
        auto start=std::chrono::time_point_cast<std::chrono::microseconds>(start_timestamped_).time_since_epoch().count();
        auto end=std::chrono::time_point_cast<std::chrono::microseconds>(end_timestamped).time_since_epoch().count();
        auto duration=(end-start);
        auto ms = duration*0.001;
        std::cout<< duration<<"us("<<ms<<"ms)\n";
    }

    ~Timer()
    {
     Stop();
    }




private:

    std::chrono::time_point<std::chrono::high_resolution_clock> start_timestamped_;


};
}