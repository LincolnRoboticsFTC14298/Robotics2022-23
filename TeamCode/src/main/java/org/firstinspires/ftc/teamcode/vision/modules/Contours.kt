package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc


class Contours (
    private val inputModule: AbstractPipelineModule<Mat>
) : AbstractPipelineModule<List<MatOfPoint>>() {

    private var output = mutableListOf<MatOfPoint>()
    private var hierarchy = Mat()

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<MatOfPoint> {
        output.clear()
        Imgproc.findContours(inputModule.processFrame(rawInput), output, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        return output
    }

}