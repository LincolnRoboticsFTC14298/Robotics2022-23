package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*

class Convexity() : Feature {

    private val convexityResults = mutableListOf<Double>()


    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourPerimeter = arcLength(MatOfPoint2f(*contour.toArray()), true)

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

        val hullPerimeter = arcLength(MatOfPoint2f(*convexHull.toArray()), true)

        val convexity = hullPerimeter / contourPerimeter
        convexityResults.add(convexity)
        return convexity
    }

    override fun mean(): Double {
        return convexityResults.sumOf{it}/convexityResults.size
    }

    override fun variance(): Double {
        return convexityResults.sumOf { (it-mean()) * (it-mean()) } / convexityResults.size
    }
}