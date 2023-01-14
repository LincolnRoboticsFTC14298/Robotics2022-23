package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import org.opencv.imgproc.Imgproc.contourArea

class Area() : Feature {

    private val areaResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val area = contourArea(contour)
        areaResults.add(area)
        return area
    }

    override fun mean(): Double {
        return areaResults.sumOf{it}/areaResults.size
    }

    override fun variance(): Double {
        return areaResults.sumOf { (it-mean()) * (it-mean()) } / areaResults.size
    }

    fun areaResultsList(): MutableList<Double> {
        return areaResults
    }

}