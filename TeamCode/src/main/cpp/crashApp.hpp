//
// Created by fyre on 10/26/22.
//

#pragma once

#include <jni.h>
#include <stdio.h>
#include <stdlib.h>

namespace crasher
{
    void killApp();
}

extern "C"
{
    JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_CPPBridge_breakShit(JNIEnv *env, jclass clazz)
    {
        crasher::killApp();
    }
}