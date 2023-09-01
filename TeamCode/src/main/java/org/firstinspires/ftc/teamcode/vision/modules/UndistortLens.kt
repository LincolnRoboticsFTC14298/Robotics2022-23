
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.calib3d.Calib3d
import org.opencv.core.Mat

// Class to correct lens distortion in an image
class UndistortLens(
    private val inputModule: AbstractPipelineModule<Mat>,
    private val camera: Vision.Companion.CameraData
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    // Undistort the lens using the camera data
    override fun process(input: Mat): Mat {
        Calib3d.undistort(input, output, camera.cameraMatrix, camera.distCoeffs)
        return output
    }
}
