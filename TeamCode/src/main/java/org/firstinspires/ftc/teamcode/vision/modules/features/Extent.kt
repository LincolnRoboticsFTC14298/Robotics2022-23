package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import org.opencv.imgproc.Imgproc.contourArea
import kotlin.math.pow

class Extent : Feature {

    private val extentResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        val boundingBox = boundingRect(contour)
        val boundingBoxArea = boundingBox.width * boundingBox.height
        if (boundingBoxArea == 0.0) return 0.0
        val extent = contourArea / boundingBoxArea
        extentResults.add(extent)
        return extent
    }

    override fun mean(): Double {
        return if (extentResults.isNotEmpty()) extentResults.sumOf { it } / extentResults.size else 0.0
    }

    override fun variance(): Double {
        val meanValue = mean()
        return if (extentResults.isNotEmpty()) extentResults.sumOf { (it - meanValue).pow(2) } / extentResults.size else 0.0
    }
}
