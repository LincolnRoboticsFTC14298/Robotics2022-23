package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import org.opencv.imgproc.Imgproc.contourArea

class Extent() : Feature {

    private val extentResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        val boundingBox = boundingRect(contour)

        val extent = contourArea / (boundingBox.width * boundingBox.height)
        extentResults.add(extent)
        return extent
    }

    override fun mean(): Double {
        return extentResults.sumOf{it}/extentResults.size
    }

    override fun variance(): Double {
        return extentResults.sumOf { (it-mean()) * (it-mean()) } / extentResults.size
    }
}