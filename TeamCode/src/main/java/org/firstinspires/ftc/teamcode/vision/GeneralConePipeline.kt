package org.firstinspires.ftc.teamcode.vision

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
    private val FOVX: Double,
    private val FOVY: Double
) : ModularPipeline() {

//    val camMat = Mat()
//    val distCoeffs = Mat()

    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_RED_MASK,
        RAW_BLUE_MASK,
        RAW_COMBINED_MASK,
        DENOISED_MASK,
        ALL_CONTOURS
    }

    // Modules //
    val inputModule = InputModule()
    //val undistort = UndistortLens(input, camMat, distCoeffs)
    val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    val redMask = Filter(labColorSpace, Scalar(0.0, 150.0, 100.0), Scalar(255.0, 200.0, 175.0))
    val blueMask = Filter(labColorSpace, Scalar(0.0, 130.0, 30.0), Scalar(255.0, 180.0, 120.0))
    val combinedMask = CombineMasks(redMask, blueMask)
    val denoisedMask = Denoise(combinedMask)
    val contours = Contours(denoisedMask)

    val coneAspectRatio = 1.33

    // Stacked scorer //
    val stackConvexity = Convexity(0.96)
    val stackExtent = Extent(0.78) //TODO Make range?
    val stackSolidity = Solidity(0.93)
    val stackAspectRatio = ThresholdAspectRatio(coneAspectRatio + 0.4, 10.0)
    val stackContours = FilterContours(contours, 0.1, Pair(1.0, stackConvexity), Pair(1.0, stackExtent), Pair(1.0, stackSolidity), Pair(5.0, stackAspectRatio)) //TODO change weights

    // Single cone scorer //
    val singleConeConvexity = Convexity(0.93)
    val singleConeExtent = Extent(0.68)
    val singleConeSolidity = Solidity(0.9)
    val singleConeAspectRatio = ThresholdAspectRatio(coneAspectRatio - 0.3, coneAspectRatio + 0.3)
    val singleConeContours = FilterContours(contours, 0.1, Pair(1.0, singleConeConvexity), Pair(1.0, singleConeExtent), Pair(1.0, singleConeSolidity), Pair(10.0, singleConeAspectRatio))

    //Single color single cone mask overlap //
    val test = ContourToMask(singleConeContours)
    val redOverlap = MaskOverlap(test, redMask)
    val redSingleConeContours = Contours(redOverlap)
    val blueOverlap = MaskOverlap(ContourToMask(singleConeContours), blueMask)
    val blueSingleConeContours = Contours(blueOverlap)


    val stackResultsModule = ContourResults(stackContours, 0.0, FOVX, FOVY)
    val singleConeResultsModule = ContourResults(singleConeContours, 0.0, FOVX, FOVY)
    val redSingleConeResultsModule = ContourResults(redSingleConeContours, 0.0, FOVX, FOVY)
    val blueSingleConeResultsModule = ContourResults(blueSingleConeContours, 0.0, FOVX, FOVY)

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
                drawContours(input, contours.processFrame(input), -1, Scalar(255.0, 0.0, 255.0), 1) //Purple for stack contours
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