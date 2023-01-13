package org.firstinspires.ftc.teamcode.vision.modules.scorers

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import kotlin.math.pow

class Solidity(private val targetSolidity: Double) : Scorer {

    override fun score(contour: MatOfPoint): Double {
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

        return (solidity - targetSolidity)*(solidity - targetSolidity)
    }
}