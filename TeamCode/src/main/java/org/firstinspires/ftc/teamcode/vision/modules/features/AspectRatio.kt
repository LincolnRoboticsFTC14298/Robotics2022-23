package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect

class AspectRatio() : Feature {

    private val aspectRatioResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val boundingBox = boundingRect(contour)
        val aspectRatio = boundingBox.height.toDouble() / boundingBox.width.toDouble()
        aspectRatioResults.add(aspectRatio)
        return aspectRatio
    }

    override fun mean(): Double {
        return aspectRatioResults.sumOf{it}/aspectRatioResults.size
    }

    override fun variance(): Double {
        return aspectRatioResults.sumOf { (it-mean()) * (it-mean()) } / aspectRatioResults.size
    }

    fun min(): Double? {
        return aspectRatioResults.minOrNull()
    }

    fun max(): Double? {
        return aspectRatioResults.maxOrNull()
    }
}