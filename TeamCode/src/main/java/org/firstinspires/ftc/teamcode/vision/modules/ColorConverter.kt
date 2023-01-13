package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class ColorConverter(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val colorMode: Int = Imgproc.COLOR_RGB2Lab
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
        Imgproc.cvtColor(inputModule.processFrame(rawInput), output, colorMode)
        return output
    }

}