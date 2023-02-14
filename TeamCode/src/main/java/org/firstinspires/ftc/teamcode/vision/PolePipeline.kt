package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Convexity
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Extent
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Solidity
import org.firstinspires.ftc.teamcode.vision.modules.scorers.ThresholdAspectRatio
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.drawContours

open class PolePipeline(
    private var displayMode: DisplayMode = DisplayMode.ALL_CONTOURS,
    camera: RobotConfig.CameraData
) : ModularPipeline() {

    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_MASK,
        DENOISED_MASK,
        ALL_CONTOURS
    }

    // Modules //
    val inputModule = InputModule()
    //val undistort = UndistortLens(input, camera)
    val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    val poleMask = Filter(labColorSpace, Scalar(0.0, 110.0, 150.0), Scalar(255.0, 150.0, 200.0))
    val denoisedMask = Denoise(poleMask, 5, 3, 3, 1)
    val contours = Contours(denoisedMask)

    // Scorer // // RUSH JOB - SCORERS AND WEIGHTS ESTIMATED
    val poleConvexity = Convexity(0.97)
    val poleExtent = Extent(0.4)
    val poleSolidity = Solidity(0.95)
    val poleAspectRatio = ThresholdAspectRatio(3.5, 50.0)
    val poleContours = FilterContours(contours, 0.1, Pair(5.0, poleConvexity), Pair(0.5, poleExtent), Pair(1.0, poleSolidity), Pair(5.0, poleAspectRatio))

    val poleResultsModule = ContourResults(poleContours, camera, true)

    // Data we care about and wish to access
    var poleResults = listOf<ContourResults.AnalysisResult>()

    val conePipeline = GeneralConePipeline(GeneralConePipeline.DisplayMode.ALL_CONTOURS, camera)

    init {
        addEndModules(poleResultsModule)
    }

    override fun processFrameForCache(input: Mat) : Mat {

        conePipeline.processFrame(input)

        // Get the data we want (yipee)
        poleResults = poleResultsModule.processFrame(input)

        // Display
        return when (displayMode) {
            DisplayMode.RAW_CAMERA_INPUT -> input
            DisplayMode.RAW_MASK -> poleMask.processFrame(input)
            DisplayMode.DENOISED_MASK -> denoisedMask.processFrame(input)
            DisplayMode.ALL_CONTOURS -> {
                drawContours(input, contours.processFrame(input), -1, Scalar(255.0, 0.0, 0.0), 1) // red for other contours
                drawContours(input, poleContours.processFrame(input), -1, Scalar(255.0, 255.0, 0.0), 1) // yellow for pole contours
                input
            }
        }
    }

    override fun onViewportTapped() {
        val modes = enumValues<DisplayMode>()
        val nextOrdinal = (displayMode.ordinal + 1) % modes.size
        displayMode = modes[nextOrdinal]
    }

    // TODO methods to check if single cone part of pole or not

}