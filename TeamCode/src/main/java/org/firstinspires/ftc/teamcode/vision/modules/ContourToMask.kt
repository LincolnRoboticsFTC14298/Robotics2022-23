package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

class ContourToMask(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>
) : AbstractPipelineModule<Mat>() {

    private var output: Mat = Mat()

    init {
        addParentModules(contourModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = Mat.zeros(input.size(), CvType.CV_8U)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return output.apply {
            setTo(Scalar.all(0.0))
            Imgproc.drawContours(this, contourModule.processFrame(rawInput), -1, Scalar(255.0), -1)
        }
    }
}
