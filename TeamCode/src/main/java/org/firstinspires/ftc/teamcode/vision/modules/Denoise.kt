package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc


class Denoise (
    private val inputModule: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        Imgproc.morphologyEx(inputModule.processFrame(rawInput), output, Imgproc.MORPH_CLOSE, Mat.ones(5, 5, CvType.CV_32F))
        Imgproc.morphologyEx(output, output, Imgproc.MORPH_OPEN, Mat.ones(3, 3, CvType.CV_32F))
        return output
    }

}