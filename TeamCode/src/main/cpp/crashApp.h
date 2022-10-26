//
// Created by fyre on 10/26/22.
//

#ifndef ROBOTICS2022_23_CRASHAPP_H
#define ROBOTICS2022_23_CRASHAPP_H

#include <jni.h>
#include <stdio.h>
#include <stdlib.h>

namespace crasher
{
    void killApp();
}

extern "C"
{
    JNIEXPORT void JNICALL Java_org_firstinspires_ftc_teamcode_BrokenTeleOp_breakShit(JNIEnv *env, jclass clazz)
    {
        crasher::killApp();
    }
}

#endif //ROBOTICS2022_23_CRASHAPP_H