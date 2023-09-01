
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.imgproc.Imgproc.*

// Class to calculate the Convexity feature of a contour
class Convexity : Feature {

    // List to store the convexity results
    private val convexityResults = mutableListOf<Double>()

    // Computes the convexity of the given contour
    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourPerimeter = arcLength(MatOfPoint2f(*contour.toArray()), true)
        
        val hull = MatOfInt()
        convexHull(contour, hull)
        
        val hullContour = MatOfPoint(*hull.toArray())
        val hullPerimeter = arcLength(MatOfPoint2f(*hullContour.toArray()), true)
        
        val convexity = contourPerimeter / hullPerimeter
        convexityResults.add(convexity)
        return convexity
    }
    
    // Other methods like mean(), variance() etc. can be added here
}
