package org.firstinspires.ftc.teamcode.vision.modulelib

import org.opencv.core.Mat

/**
 * A [PipelineModule] is an atomic unit of a modular pipeline.
 * Each module outputs an object of a single type when processing the frame.
 * @param T                  Type of output.
 * @author Jared Haertel
*/
interface PipelineModule<T> {

    fun init(input: Mat)

    fun processFrame(input: Mat): T

}