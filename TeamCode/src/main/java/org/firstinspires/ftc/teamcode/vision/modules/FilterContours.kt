package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Scorer
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint

class FilterContours(
    private val contourModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val threshold: Double,
    private val scorer: Scorer
) : AbstractPipelineModule<List<MatOfPoint>>() {

    init {
        addParentModules(contourModule)
    }

    override fun processFrameForCache(rawInput: Mat): List<MatOfPoint> {
        return contourModule.processFrame(rawInput).filter { contour ->
            scorer.score(contour) <= threshold
        }
    }
}
