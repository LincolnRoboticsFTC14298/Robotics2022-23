
package org.firstinspires.ftc.teamcode.vision.modules.features

import org.opencv.core.MatOfPoint
import org.opencv.imgproc.Imgproc.boundingRect
import org.opencv.imgproc.Imgproc.contourArea

// Class to calculate the Extent feature of a contour
class Extent : Feature {

    // List to store the extent results
    private val extentResults = mutableListOf<Double>()

    // Computes the extent of the given contour
    override fun featureMeasurement(contour: MatOfPoint): Double {
        val contourArea = contourArea(contour)
        val boundingBox = boundingRect(contour)
        val boundingBoxArea = boundingBox.width * boundingBox.height.toDouble()
        
        val extent = contourArea / boundingBoxArea
        extentResults.add(extent)
        return extent
    }
    
    // Other methods like mean(), variance() etc. can be added here
}
