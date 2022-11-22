package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.modules.Filter
import org.firstinspires.ftc.teamcode.vision.modules.YCrCbConverter
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.SimpleModule
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.openftc.easyopencv.OpenCvPipeline

/**
 * Basic pipeline demonstrating the modular system.
 * Simply converts color space and then filters it.
 * @author Jared Haertel
 */
class BasicPipeline() : OpenCvPipeline() {

    private val inputModule = InputModule()
    private val converter = YCrCbConverter(inputModule)
    private val filter = Filter(converter, Scalar(0.0, 0.0, 0.0), Scalar(0.0, 0.0, 255.0))

    private lateinit var lastConverterOutput: Mat

    override fun processFrame(input: Mat): Mat {
        // Clear previously cached values
        // Only needs to be checked for end nodes

        converter.clearCache()
        filter.clearCache()

        lastConverterOutput = converter.processFrame(input)
        return filter.processFrame(input)
    }

    fun getConverterOutput() : Mat {
        return lastConverterOutput
    }

}