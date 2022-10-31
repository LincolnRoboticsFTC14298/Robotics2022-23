//
// Created by fyre on 10/31/22.
//

#include "crashApp.hpp"

extern "C"
{
    JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_CPPBridge_breakShit(JNIEnv* env, jclass clazz)
    {
        crasher::killApp();
    }
}