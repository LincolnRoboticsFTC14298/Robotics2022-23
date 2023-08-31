package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class Denoise(
    private val inputModule: AbstractPipelineModule<Mat>,
    morphCloseRows: Int,
    morphCloseCols: Int,
    morphOpenRows: Int,
    morphOpenCols: Int
) : AbstractPipelineModule<Mat>() {

    private val morphCloseMat = Mat.ones(morphCloseRows, morphCloseCols, CvType.CV_32F)
    private val morphOpenMat = Mat.ones(morphOpenRows, morphOpenCols, CvType.CV_32F)

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return Mat().apply {
            Imgproc.morphologyEx(inputModule.processFrame(rawInput), this, Imgproc.MORPH_CLOSE, morphCloseMat)
            Imgproc.morphologyEx(this, this, Imgproc.MORPH_OPEN, morphOpenMat)
        }
    }
}
