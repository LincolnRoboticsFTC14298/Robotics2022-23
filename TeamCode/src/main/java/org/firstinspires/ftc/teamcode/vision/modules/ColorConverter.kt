package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class ColorConverter(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val colorMode: Int = Imgproc.COLOR_RGB2Lab
) : AbstractPipelineModule<Mat>() {

    private var output: Mat? = null

    init {
        addParentModules(inputModule)
    }

    override fun init(input: Mat) {
        super.init(input)
        output = input.clone()
    }

    override fun processFrameForCache(rawInput: Mat): Mat =
        output?.apply { Imgproc.cvtColor(inputModule.processFrame(rawInput), this, colorMode) }
            ?: Mat().also { output = it }
}
