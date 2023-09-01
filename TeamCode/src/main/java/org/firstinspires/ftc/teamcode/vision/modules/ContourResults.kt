
package org.firstinspires.ftc.teamcode.vision.modules

import android.util.Log
import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.LogFiles.log
import org.firstinspires.ftc.teamcode.util.epsilonEquals
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint

// Class to manage the results of contour analysis in a vision pipeline
class ContourResults(
    private val contoursInputModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val fieldConfig: FieldConfig,
    private val vision: Vision
) : AbstractPipelineModule<Mat>() {

    private lateinit var output: Mat

    // Additional logic for contour result processing can be added here
    override fun process(input: Mat): Mat {
        // Your implementation here
        return output
    }
}
