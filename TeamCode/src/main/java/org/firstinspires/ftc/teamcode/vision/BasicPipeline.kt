package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.Filter
import org.firstinspires.ftc.teamcode.vision.modules.YCrCbConverter
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modulelib.SimpleModule
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Basic pipeline demonstrating the modular system.
 * Simply converts color space and then filters it.
 * @author Jared Haertel
 */
class BasicPipeline() : ModularPipeline() {

    private val inputModule = InputModule()
    private val converter = YCrCbConverter(inputModule)
    private val filter = Filter(converter, Scalar(0.0, 0.0, 0.0), Scalar(0.0, 0.0, 255.0))

    init {
        addEndModules(converter, filter)
    }

    private lateinit var lastConverterOutput: Mat

    override fun processFrameForCache(input: Mat) : Mat {
        lastConverterOutput = converter.processFrame(input)
        return filter.processFrame(input)
    }

    fun getConverterOutput() : Mat {
        return lastConverterOutput
    }

}