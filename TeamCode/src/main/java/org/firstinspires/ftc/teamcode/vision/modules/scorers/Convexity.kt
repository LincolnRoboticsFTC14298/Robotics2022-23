package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import kotlin.math.pow

class Convexity(private val targetConvexity: Double) : Scorer {

    override fun score(contour: MatOfPoint): Double {
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


        return (convexity - targetConvexity)*(convexity - targetConvexity)
    }
}