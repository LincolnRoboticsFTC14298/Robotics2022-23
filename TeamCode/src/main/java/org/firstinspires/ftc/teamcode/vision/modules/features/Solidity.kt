
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfInt
import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.*

// Class to calculate the Solidity feature of a contour
class Solidity : Feature {

    // List to store the solidity results
    private val solidityResults = mutableListOf<Double>()

    // Computes the solidity of the given contour
    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        
        val hull = MatOfInt()
        convexHull(contour, hull)
        
        val hullContour = MatOfPoint(*hull.toArray())
        val hullArea = contourArea(hullContour)
        
        val solidity = contourArea / hullArea
        solidityResults.add(solidity)
        return solidity
    }
    
    // Other methods like mean(), variance() etc. can be added here
}
