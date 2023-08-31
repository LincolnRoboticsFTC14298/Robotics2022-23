package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.imgproc.Imgproc.arcLength
import org.opencv.imgproc.Imgproc.convexHull
import kotlin.math.pow

class Convexity : Feature {

    private val convexityResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourPerimeter = arcLength(MatOfPoint2f(*contour.toArray()), true)

        val hull = MatOfInt()
        convexHull(contour, hull)

        val hullPoints = hull.toList().map { contour.toArray()[it] }.toTypedArray()
        val convexHull = MatOfPoint(*hullPoints)

        val hullPerimeter = arcLength(MatOfPoint2f(*convexHull.toArray()), true)

        return (hullPerimeter / contourPerimeter).also { convexityResults.add(it) }
    }

    override fun mean(): Double = convexityResults.average()

    override fun variance(): Double {
        val mean = mean()
        return convexityResults.sumOf { (it - mean).pow(2) } / convexityResults.size
    }
}
