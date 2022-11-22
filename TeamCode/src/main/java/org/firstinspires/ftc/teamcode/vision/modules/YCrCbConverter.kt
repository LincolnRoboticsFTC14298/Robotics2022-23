package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modulelib.PipelineModule
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class YCrCbConverter(private val inputFrame: AbstractPipelineModule<Mat>) : AbstractPipelineModule<Mat>(inputFrame) {

    private lateinit var output: Mat

    override fun init(input: Mat) {
        output = input.clone()
    }

    override fun processFrameForCache(input: Mat): Mat {
        Imgproc.cvtColor(inputFrame.processFrame(input), output, Imgproc.COLOR_RGB2YCrCb)
        return input
    }

}