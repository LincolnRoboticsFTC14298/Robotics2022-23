package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import kotlin.math.pow

class AspectRatio : Feature {

    private val aspectRatioResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val boundingBox = boundingRect(contour)
        return (boundingBox.height.toDouble() / boundingBox.width.toDouble()).also { aspectRatioResults.add(it) }
    }

    override fun mean(): Double = aspectRatioResults.average()

    override fun variance(): Double {
        val mean = mean()
        return aspectRatioResults.sumOf { (it - mean).pow(2) } / aspectRatioResults.size
    }

    fun min() = aspectRatioResults.minOrNull()

    fun max() = aspectRatioResults.maxOrNull()
}
