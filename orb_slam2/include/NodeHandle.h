#include <ros/ros.h>
#include <mutex>

namespace Carto
{


    class Node 
    {

        Node() =default ;

        

        private: 

        std::mutex mutex_;


    };



}