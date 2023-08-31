package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.Rect

class Crop(
    private val inputModule: AbstractPipelineModule<Mat>,
    x: Int,
    y: Int,
    width: Int,
    height: Int
) : AbstractPipelineModule<Mat>() {

    private val rect = Rect(x, y, width, height)

    init {
        addParentModules(inputModule)
    }

    override fun processFrameForCache(rawInput: Mat): Mat {
        return inputModule.processFrame(rawInput).submat(rect).clone()
    }
}
