package org.firstinspires.ftc.teamcode.vision.modules

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.imgproc.Imgproc.*
import java.lang.Math.toRadians
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.tan


/**
 * Returns contours that pass the scorers by thresholding a weighted sum.
 * @param cameraHeight must be in same units as returned distance
 */
class ContourResults(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val camera: RobotConfig.CameraData,
    private val targetHeight: Double = 0.0,
    private val pitchDistanceOffset: Double = 0.0, // Used incase the center of the object is wanted instead
    private val useDistanceByWidth: Boolean = false
) : AbstractPipelineModule<List<ContourResults.AnalysisResult>>() {

    data class AnalysisResult(val angle: Double, val distance: Double) {

        fun distanceSqrTo(other: AnalysisResult) : Double {
            return distance * distance + other.distance * other.distance - 2 * distance * other.distance * cos(other.angle - angle)
        }

        fun toVector() : Vector2d {
            return Vector2d.polar(distance, angle)
        }
    }

    init {
        addParentModules(contourModule)
    }

    // TODO Use inverse projection on pixelPoint???
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

            // calculate angle and distance
            val pitch = (Ay/2.0) * camera.FOVY
            val yaw = (Ax/2.0) * camera.FOVX
            val distanceByPitch = (targetHeight - camera.height) / tan(camera.pitch + pitch) + pitchDistanceOffset

            /// New variable
            val contour2f = MatOfPoint2f(*contour.toArray())
            val contourMinAreaRect = minAreaRect(contour2f)
            val distanceByWidth = min(contourMinAreaRect.size.width, contourMinAreaRect.size.height) //TODO double check, finish formula

            // add to results
            if (useDistanceByWidth || pitch epsilonEquals camera.FOVY / 2.0) { // TODO not safe for non rectangular outlines
                results.add(AnalysisResult(yaw, distanceByWidth))
            } else {
                results.add(AnalysisResult(yaw, distanceByPitch))
            }
        }
        return results
    }


}