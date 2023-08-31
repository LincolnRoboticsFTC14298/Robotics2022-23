package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc

class Contours(
    private val inputModule: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<List<MatOfPoint>>() {

    private val output: MutableList<MatOfPoint> = mutableListOf()
    private val hierarchy: Mat = Mat()

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<MatOfPoint> {
        return mutableListOf<MatOfPoint>().apply {
            Imgproc.findContours(
                inputModule.processFrame(rawInput),
                this,
                hierarchy,
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
            )
        }
    }
}
