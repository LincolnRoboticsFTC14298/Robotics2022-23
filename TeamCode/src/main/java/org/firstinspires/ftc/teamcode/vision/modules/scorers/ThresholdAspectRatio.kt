package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc
import kotlin.math.max

class ThresholdAspectRatio(
    private val minimumAspectRatio: Double,
    private val maximimumAspectRatio: Double
) : Scorer {

    override fun score(contour: MatOfPoint): Double {
        val boundingBox = Imgproc.boundingRect(contour)

        val aspectRatio = boundingBox.height/boundingBox.width

        return if (aspectRatio >= minimumAspectRatio && aspectRatio <= maximimumAspectRatio) 0.0
        else 1.0
    }
}