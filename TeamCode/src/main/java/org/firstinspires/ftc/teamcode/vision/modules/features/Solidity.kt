package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*

class Solidity() : Feature {

    private val solidityResults = mutableListOf<Double>()

    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)

        val hullList: ArrayList<MatOfPoint> = ArrayList()

        val hull = MatOfInt()
        convexHull(contour, hull)

        val contourArray = contour.toArray()
        val hullPoints = arrayOfNulls<Point>(hull.rows())
        val hullContourIdxList = hull.toList()
        for (i in hullContourIdxList.indices) {
            hullPoints[i] = contourArray[hullContourIdxList[i]]
        }

        val convexHull = MatOfPoint(*hullPoints)
        hullList.add(convexHull)

        val hullArea = contourArea(convexHull)

        val solidity = contourArea / hullArea
        solidityResults.add(solidity)
        return solidity
    }

    override fun mean(): Double {
        return solidityResults.sumOf{it}/solidityResults.size
    }

    override fun variance(): Double {
        return solidityResults.sumOf { (it-mean()) * (it-mean()) } / solidityResults.size
    }
}