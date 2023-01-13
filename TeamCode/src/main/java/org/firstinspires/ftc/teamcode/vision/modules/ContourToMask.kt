package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.*
import org.opencv.imgproc.Imgproc


class ContourToMask (
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(contourModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = Mat.zeros(input.size(), CvType.CV_8U)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        output.setTo(Scalar.all(0.0)) // Set matrix all to zero
        Imgproc.drawContours(output, contourModule.processFrame(rawInput), -1, Scalar(255.0), -1)
        return output
    }

}