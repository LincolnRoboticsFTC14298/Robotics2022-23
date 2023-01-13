package org.firstinspires.ftc.teamcode.vision.modules.scorers


import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import org.opencv.imgproc.Imgproc.contourArea
import kotlin.math.pow

class Extent(private val targetExtent: Double) : Scorer {

    override fun score(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        val boundingBox = boundingRect(contour)

        val extent = contourArea / (boundingBox.width * boundingBox.height)

        return (extent - targetExtent)*(extent - targetExtent)
    }
}