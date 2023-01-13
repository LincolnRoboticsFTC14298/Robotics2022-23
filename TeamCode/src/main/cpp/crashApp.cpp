//
// Created by fyre on 10/26/22.
//
#include "crashApp.hpp"

void crasher::killApp()
{
    int i = 0;
    while (true)
    {
        void* del = malloc(++i);
        if (i < 0)
            break;
    }
}