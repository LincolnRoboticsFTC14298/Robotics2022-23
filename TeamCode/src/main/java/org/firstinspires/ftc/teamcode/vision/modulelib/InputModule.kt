package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat

/**
 * Input module of the pipeline.
 * @author Jared Haertel
 */
class InputModule() : SimpleModule<Mat>({ input: Mat -> input })