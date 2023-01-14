package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import java.lang.Math.min
import java.lang.Math.toRadians


/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param cameraHeight must be in same units as returned distance
 */
class ContourResults(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val cameraHeight: Double,
    private val FOVX: Double,
    private val FOVY: Double,
    private val cameraPitch: Double = 0.0,
    private val useDistanceByWidth: Boolean = false
) : AbstractPipelineModule<List<ContourResults.AnalysisResult>>() {

    data class AnalysisResult(val point: Point, val angle: Double, val distance: Double)

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<AnalysisResult> {
        val results = mutableListOf<AnalysisResult>()
        for (contour in contourModule.processFrame(rawInput)) {
            val boundingBox = boundingRect(contour)
            val pixelX = boundingBox.x+(boundingBox.width/2.0)
            val pixelY = (boundingBox.y+boundingBox.height).toDouble()
            val pixelPoint = Point(pixelX, pixelY) // find pixel location of bottom middle of cone

            // adjust to aiming coordinate space
            val Ax = (pixelPoint.x - (rawInput.size().width/2)) / (rawInput.size().width/2.0)
            val Ay = (pixelPoint.y - (rawInput.size().height/2)) / (rawInput.size().height/2.0)
            val aimingPoint = Point(Ax, Ay)

            // calculate angle and distance
            val pitch = (Ay/2.0) * toRadians(FOVY)
            val yaw = (Ax/2.0) * toRadians(FOVX)
            val distance = -cameraHeight/kotlin.math.tan(cameraPitch + pitch)

            /// New variable
            val contour2f = MatOfPoint2f(*contour.toArray())
            val contourMinAreaRect = minAreaRect(contour2f)
            val distanceByWidth = min(contourMinAreaRect.size.width, contourMinAreaRect.size.height) //TODO double check, finish formula

            // add to results
            if(useDistanceByWidth){
                results.add(AnalysisResult(pixelPoint, yaw, distanceByWidth))
            } else {
                results.add(AnalysisResult(pixelPoint, yaw, distance))
            }
        }
        return results
    }


}