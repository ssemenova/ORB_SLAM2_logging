#ifndef THREADLOCAL_H
#define THREADLOCAL_H

#include <fstream>

namespace ORB_SLAM2
{
    thread_local std::ofstream cout_stream;
}

#endif