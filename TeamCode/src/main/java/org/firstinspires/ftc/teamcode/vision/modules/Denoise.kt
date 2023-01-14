package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc


class Denoise (
    private val inputModule: AbstractPipelineModule<Mat>,
    private val MORPH_CLOSE_ROWS: Int,
    private val MORPH_CLOSE_COLUMNS: Int,
    private val MORPH_OPEN_ROWS: Int,
    private val MORPH_OPEN_COLUMNS: Int,
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        Imgproc.morphologyEx(inputModule.processFrame(rawInput), output, Imgproc.MORPH_CLOSE, Mat.ones(MORPH_CLOSE_ROWS, MORPH_CLOSE_COLUMNS, CvType.CV_32F))
        Imgproc.morphologyEx(output, output, Imgproc.MORPH_OPEN, Mat.ones(MORPH_OPEN_ROWS, MORPH_OPEN_COLUMNS, CvType.CV_32F))
        return output
    }

}