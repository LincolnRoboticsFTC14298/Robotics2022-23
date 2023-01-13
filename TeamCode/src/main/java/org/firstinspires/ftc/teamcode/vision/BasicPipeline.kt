package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.modules.Filter
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.ColorConverter
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc

/**
 * Basic pipeline demonstrating the modular system.
 * Simply converts color space and then filters it.
 * @author Jared Haertel
 */
class BasicPipeline() : ModularPipeline() {

    private val inputModule = InputModule()
    private val converter = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    private val filter = Filter(converter, Scalar(0.0, 0.0, 0.0), Scalar(255.0, 255.0, 255.0))

    init {
        addEndModules(filter)
    }

    private lateinit var lastConverterOutput: Mat

    override fun processFrameForCache(input: Mat) : Mat {
        //lastConverterOutput = converter.processFrame(input)
        return filter.processFrame(input)
    }

    fun getConverterOutput() : Mat {
        return lastConverterOutput
    }

}