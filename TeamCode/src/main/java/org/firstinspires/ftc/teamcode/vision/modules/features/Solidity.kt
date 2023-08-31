package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.contourArea
import org.opencv.imgproc.Imgproc.convexHull
import kotlin.math.pow

class Solidity : Feature {

    private val solidityResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        val hull = MatOfInt()
        convexHull(contour, hull)

        val hullPoints = hull.toList().map { contour.toArray()[it] }.toTypedArray()
        val convexHull = MatOfPoint(*hullPoints)

        val hullArea = contourArea(convexHull)
        if (hullArea == 0.0) return 0.0

        val solidity = contourArea / hullArea
        solidityResults.add(solidity)
        return solidity
    }

    override fun mean(): Double {
        return if (solidityResults.isNotEmpty()) solidityResults.average() else 0.0
    }

    override fun variance(): Double {
        val meanValue = mean()
        return if (solidityResults.isNotEmpty()) solidityResults.sumOf { (it - meanValue).pow(2) } / solidityResults.size else 0.0
    }
}
