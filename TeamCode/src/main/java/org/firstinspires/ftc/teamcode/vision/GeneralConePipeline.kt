package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.coneDiameter
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.scorers.*
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.drawContours

open class GeneralConePipeline(
    private var displayMode: DisplayMode = DisplayMode.ALL_CONTOURS,
    camera: RobotConfig.CameraData
) : ModularPipeline() {


    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_RED_MASK,
        RAW_BLUE_MASK,
        RAW_COMBINED_MASK,
        DENOISED_MASK,
        ALL_CONTOURS
    }

    // Modules //
    private val inputModule = InputModule()
    //private val undistort = UndistortLens(input, camera)
    private val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    private val redMask = Filter(labColorSpace, Scalar(0.0, 150.0, 100.0), Scalar(255.0, 200.0, 175.0))
    private val blueMask = Filter(labColorSpace, Scalar(0.0, 130.0, 30.0), Scalar(255.0, 180.0, 120.0))
    private val combinedMask = CombineMasks(redMask, blueMask)
    private val denoisedMask = Denoise(combinedMask, 5, 5, 3, 3)
    private val contours = Contours(denoisedMask)

    private val coneAspectRatio = 1.33

    // Stacked scorer //
    private val stackConvexity = Convexity(0.96)
    private val stackExtent = Extent(0.78) //TODO Make range?
    private val stackSolidity = Solidity(0.93)
    private val stackAspectRatio = ThresholdAspectRatio(coneAspectRatio + 0.4, 10.0)
    private val stackContours = FilterContours(contours, 0.1, Pair(1.0, stackConvexity), Pair(1.0, stackExtent), Pair(1.0, stackSolidity), Pair(5.0, stackAspectRatio)) //TODO change weights

    // Single cone scorer //
    private val singleConeConvexity = Convexity(0.93)
    private val singleConeExtent = Extent(0.68)
    private val singleConeSolidity = Solidity(0.9)
    private val singleConeAspectRatio = ThresholdAspectRatio(coneAspectRatio - 0.3, coneAspectRatio + 0.3)
    private val singleConeContours = FilterContours(contours, 0.1, Pair(1.0, singleConeConvexity), Pair(1.0, singleConeExtent), Pair(1.0, singleConeSolidity), Pair(10.0, singleConeAspectRatio))

    //Single color single cone mask overlap //
    private val redOverlapMask = MaskOverlap(ContourToMask(singleConeContours), redMask)
    private val redSingleConeContours = Contours(redOverlapMask)
    private val blueOverlapMask = MaskOverlap(ContourToMask(singleConeContours), blueMask)
    private val blueSingleConeContours = Contours(blueOverlapMask)


    private val stackResultsModule = ContourResults(stackContours, camera, pitchDistanceOffset = coneDiameter/2.0)
    private val singleConeResultsModule = ContourResults(singleConeContours, camera, pitchDistanceOffset = coneDiameter/2.0)
    private val redSingleConeResultsModule = ContourResults(redSingleConeContours, camera, pitchDistanceOffset = coneDiameter/2.0)
    private val blueSingleConeResultsModule = ContourResults(blueSingleConeContours, camera, pitchDistanceOffset = coneDiameter/2.0)

    // Data we care about and wish to access
    var stackResults = listOf<ContourResults.AnalysisResult>()
    var singleConeResults = listOf<ContourResults.AnalysisResult>()
    var redSingleConeResults = listOf<ContourResults.AnalysisResult>()
    var blueSingleConeResults = listOf<ContourResults.AnalysisResult>()

    init {
        addEndModules(stackResultsModule, singleConeResultsModule, redSingleConeResultsModule, blueSingleConeResultsModule)
    }

    override fun processFrameForCache(input: Mat) : Mat {

        // Get the data we want (yipee)
        stackResults = stackResultsModule.processFrame(input)
        singleConeResults = singleConeResultsModule.processFrame(input)
        redSingleConeResults = redSingleConeResultsModule.processFrame(input)
        blueSingleConeResults = blueSingleConeResultsModule.processFrame(input)


        // Display
        return when (displayMode) {
            DisplayMode.RAW_CAMERA_INPUT -> input
            DisplayMode.RAW_RED_MASK -> redMask.processFrame(input)//redMask.processFrame(input)
            DisplayMode.RAW_BLUE_MASK -> blueMask.processFrame(input)
            DisplayMode.RAW_COMBINED_MASK -> combinedMask.processFrame(input)
            DisplayMode.DENOISED_MASK -> denoisedMask.processFrame(input)
            DisplayMode.ALL_CONTOURS -> {
                drawContours(input, contours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1) //red for other contours
                drawContours(input, stackContours.processFrame(input), -1, Scalar(255.0, 0.0, 255.0), 1) //Purple for stack contours
                drawContours(input, redSingleConeContours.processFrame(input), -1, Scalar(255.0, 0.0, 0.0), 1) //Red for single red cone contours
                drawContours(input, blueSingleConeContours.processFrame(input), -1, Scalar(0.0, 0.0, 255.0), 1) //Blue for single blue cone contours
                input
            }
        }
    }

    override fun onViewportTapped() {
        val modes = enumValues<DisplayMode>()
        val nextOrdinal = (displayMode.ordinal + 1) % modes.size
        displayMode = modes[nextOrdinal]
    }

}