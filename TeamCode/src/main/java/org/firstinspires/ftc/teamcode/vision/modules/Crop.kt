package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc


class Crop (
    private val inputModule: AbstractPipelineModule<Mat>,
    private val x: Int,
    private val y: Int,
    private val width: Int,
    private val height: Int
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    private val rect = Rect(x,y,width,height)

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        output = inputModule.processFrame(rawInput).submat(rect)
        return output
    }

}