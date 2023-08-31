package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.contourArea

class Area : Feature {

    private val areaResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        return contourArea(contour).also { areaResults.add(it) }
    }

    override fun mean(): Double = areaResults.average()

    override fun variance(): Double {
        val mean = mean()
        return areaResults.sumOf { (it - mean).pow(2) } / areaResults.size
    }

    fun areaResultsList() = areaResults
}
