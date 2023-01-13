package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import kotlin.math.pow

class AspectRatio(private val targetAspectRatio: Double) : Scorer {

    override fun score(contour: MatOfPoint): Double {
        val boundingBox = boundingRect(contour)

        val aspectRatio = boundingBox.height/boundingBox.width

        return (aspectRatio - targetAspectRatio)*(aspectRatio - targetAspectRatio)
    }
}