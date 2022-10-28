//
// Created by fyre on 10/26/22.
//
#include "crashApp.hpp"

void crasher::killApp()
{
    int i = 0;
    while (true)
    {
        printf("%d\n", i++);
        malloc(i);
    }
}