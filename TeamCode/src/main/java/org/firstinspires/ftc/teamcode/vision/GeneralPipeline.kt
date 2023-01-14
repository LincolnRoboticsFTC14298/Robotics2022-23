package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.features.*
import org.firstinspires.ftc.teamcode.vision.modules.scorers.*
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.opencv.imgproc.Imgproc.drawContours

open class GeneralPipeline(
    private var displayMode: DisplayMode = DisplayMode.ALL_CONTOURS,
    private val FOVX: Double,
    private val FOVY: Double,
    private val cameraHeight: Double,
    private val cameraPitch: Double = 0.0,
    val telemetry: Telemetry
) : ModularPipeline() {

    //private val camMat = Mat()
    //private val distCoeffs = Mat()

    enum class DisplayMode {
        RAW_CAMERA_INPUT,
        RAW_RED_MASK,
        RAW_BLUE_MASK,
        RAW_RED_OVERLAP,
        RAW_BLUE_OVERLAP,
        RAW_COMBINED_MASK,
        DENOISED_MASK,
        UNFILTERED_CONTOURS,
        CONE_FILTERED_CONTOURS,
        ALL_CONTOURS
    }

    // Modules //
    private val inputModule = InputModule()
    //private val undistort = UndistortLens(inputModule, camMat, distCoeffs)
    private val labColorSpace = ColorConverter(inputModule, Imgproc.COLOR_RGB2Lab)
    private val redMask = Filter(labColorSpace, Scalar(0.0, 154.0, 110.0), Scalar(255.0, 210.0, 190.0))
    private val blueMask = Filter(labColorSpace, Scalar(0.0, 120.0, 30.0), Scalar(255.0, 180.0, 110.0)) //very last value varies greatly depending on light level
    private val poleMask = Filter(labColorSpace, Scalar(0.0, 110.0, 150.0), Scalar(255.0, 150.0, 200.0))
    private val combinedMask = CombineMasks(redMask, blueMask)
    //private val combinedMask = redMask //TODO change back once done testing
    private val denoisedConeMask = Denoise(combinedMask, 5, 5, 3, 3)
    private val denoisedPoleMask = Denoise(poleMask, 5, 5, 3, 3)
    private val rawConeContours = Contours(denoisedConeMask)
    private val rawPoleContours = Contours(denoisedPoleMask)

    // Pole Scorer //
    private val poleConvexity = Convexity(); private val poleConvexityScorer = DiffSquaredScorer(poleConvexity, 0.988, 11.09)
    private val poleExtent = Extent(); private val poleExtentScorer = DiffSquaredScorer(poleExtent, 0.661, 0.036)
    private val poleSolidity = Solidity(); private val poleSolidityScorer = DiffSquaredScorer(poleSolidity, 0.909, 0.227)
    private val poleAspectRatio = AspectRatio(); private val poleAspectRatioScorer = ThresholdScorer(poleAspectRatio, Pair(1.0, 30.0), 20.0)
    private val poleArea = Area(); private val poleAreaScorer = ThresholdScorer(poleArea, Pair(50.0, 100000.0), 20.0)
    private val poleContours = FilterContours(rawPoleContours, 0.05, telemetry, poleConvexityScorer + poleExtentScorer + poleSolidityScorer + poleAspectRatioScorer + poleAreaScorer)

    // Single Cone Scorer //
    private val singleConeConvexity = Convexity(); private val singleConeConvexityScorer = DiffSquaredScorer(singleConeConvexity, 0.942, 13.3)
    private val singleConeExtent = Extent(); private val singleConeExtentScorer = DiffSquaredScorer(singleConeExtent, 0.668, 4.06)
    private val singleConeSolidity = Solidity(); private val singleConeSolidityScorer = DiffSquaredScorer(singleConeSolidity, 0.901, 38.9)
    private val singleConeAspectRatio = AspectRatio(); private val singleConeAspectRatioScorer = DiffSquaredScorer(singleConeAspectRatio, 1.369, 0.23)
    private val singleConeContours = FilterContours(rawConeContours, 0.05, telemetry, singleConeConvexityScorer + singleConeExtentScorer + singleConeSolidityScorer + singleConeAspectRatioScorer)

    // Stacked Scorer //
    private val stackConvexity = Convexity(); private val stackConvexityScorer = DiffSquaredScorer(stackConvexity, 0.894, 0.25)
    private val stackExtent = Extent(); private val stackExtentScorer = DiffSquaredScorer(stackExtent, 0.712, 0.28)
    private val stackSolidity = Solidity(); private val stackSolidityScorer = DiffSquaredScorer(stackSolidity, 0.903, 18.8)
    private val stackAspectRatio = AspectRatio(); private val stackAspectRatioScorer = ThresholdScorer(stackAspectRatio, Pair(1.0, 15.0), 25.0)
    private val stackContours = FilterContours(rawConeContours, 0.04, telemetry, stackConvexityScorer + stackExtentScorer + stackSolidityScorer + stackAspectRatioScorer)

    // Single Color Mask and Single Cone Overlap //
    private val redOverlap = MaskOverlap(ContourToMask(singleConeContours), redMask)
    private val redSingleConeContours = Contours(redOverlap)
    private val blueOverlap = MaskOverlap(ContourToMask(singleConeContours), blueMask)
    private val blueSingleConeContours = Contours(blueOverlap)

    // Results Modules //
    private val stackResultsModule = ContourResults(stackContours, cameraHeight, FOVX, FOVY, cameraPitch)
    private val singleConeResultsModule = ContourResults(singleConeContours, cameraHeight, FOVX, FOVY, cameraPitch)
    private val redSingleConeResultsModule = ContourResults(redSingleConeContours, cameraHeight, FOVX, FOVY, cameraPitch)
    private val blueSingleConeResultsModule = ContourResults(blueSingleConeContours, cameraHeight, FOVX, FOVY, cameraPitch)
    private val poleResultsModule = ContourResults(poleContours, cameraHeight, FOVX, FOVY, cameraPitch)

    // Data we care about and wish to access
    var stackResults = listOf<ContourResults.AnalysisResult>()
    var singleConeResults = listOf<ContourResults.AnalysisResult>()
    var redSingleConeResults = listOf<ContourResults.AnalysisResult>()
    var blueSingleConeResults = listOf<ContourResults.AnalysisResult>()
    var poleResults = listOf<ContourResults.AnalysisResult>()

    init {
        addEndModules(stackResultsModule, singleConeResultsModule, redSingleConeResultsModule, blueSingleConeResultsModule, poleResultsModule)
    }

    override fun processFrameForCache(input: Mat) : Mat {

        // Get the data we want (yipee)
        stackResults = stackResultsModule.processFrame(input)
        singleConeResults = singleConeResultsModule.processFrame(input)
        redSingleConeResults = redSingleConeResultsModule.processFrame(input)
        blueSingleConeResults = blueSingleConeResultsModule.processFrame(input)
        poleResults = poleResultsModule.processFrame(input)

        // Telemetry for Testing //
        telemetry.addData("displaymode", displayMode)

        telemetry.addData("mean, variance", null)
        telemetry.addData("aspectRatio", poleAspectRatio.mean().toString() + ", " + poleAspectRatio.variance().toString())
        telemetry.addData("convexity", poleConvexity.mean().toString() + ", " + poleConvexity.variance().toString())
        telemetry.addData("extent", poleExtent.mean().toString() + ", " + poleExtent.variance().toString())
        telemetry.addData("solidity", poleSolidity.mean().toString() + ", " + poleSolidity.variance().toString())
        telemetry.addData("aspectRatio min, max", poleAspectRatio.min().toString() + ", " + poleAspectRatio.max().toString())
        //telemetry.addData("areaResults", poleArea.areaResultsList().sorted().toString())
        telemetry.update()

        // Display //
        return when (displayMode) {
            DisplayMode.RAW_CAMERA_INPUT -> input
            DisplayMode.RAW_RED_MASK -> redMask.processFrame(input)
            DisplayMode.RAW_BLUE_MASK -> blueMask.processFrame(input)
            DisplayMode.RAW_RED_OVERLAP -> redOverlap.processFrame(input)
            DisplayMode.RAW_BLUE_OVERLAP -> blueOverlap.processFrame(input)
            DisplayMode.RAW_COMBINED_MASK -> combinedMask.processFrame(input)
            DisplayMode.DENOISED_MASK -> denoisedConeMask.processFrame(input)
            DisplayMode.UNFILTERED_CONTOURS ->{
                drawContours(input, rawConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1)
                input
            }
            DisplayMode.CONE_FILTERED_CONTOURS ->{
                drawContours(input, singleConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1)
                input
            }

            DisplayMode.ALL_CONTOURS -> {
                drawContours(input, rawConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1) //Green for other cone contours
                drawContours(input, stackContours.processFrame(input), -1, Scalar(255.0, 0.0, 255.0), 2) //Pink for stack contours

                drawContours(input, blueSingleConeContours.processFrame(input), -1, Scalar(0.0, 0.0, 255.0), 1) //Blue for single blue cone contours
                drawContours(input, redSingleConeContours.processFrame(input), -1, Scalar(255.0, 0.0, 0.0), 1) //Red for single red cone contours
                //drawContours(input, singleConeContours.processFrame(input), -1, Scalar(0.0, 0.0, 255.0), 2) //Blue for single cone contours

                drawContours(input, rawPoleContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1) //Green for other pole contours
                drawContours(input, poleContours.processFrame(input), -1, Scalar(255.0, 255.0, 0.0), 2) //Yellow for pole contours



                //drawContours(input, singleConeContours.processFrame(input), -1, Scalar(0.0, 255.0, 0.0), 1) //green for other contours


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