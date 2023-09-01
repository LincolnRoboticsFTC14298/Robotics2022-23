
package org.firstinspires.ftc.teamcode.vision.modules

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.modulelib.AbstractPipelineModule
import org.firstinspires.ftc.teamcode.vision.modules.scorers.Scorer
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint

// Class to filter contours based on scoring modules
class FilterContours(
    private val contoursInputModule: AbstractPipelineModule<List<MatOfPoint>>,
    private val scorers: List<Pair<Double, Scorer>>
) : AbstractPipelineModule<List<MatOfPoint>>() {

    private var output = mutableListOf<MatOfPoint>()

    // Filter contours based on the scores calculated by the scorers
    override fun process(input: List<MatOfPoint>): List<MatOfPoint> {
        output.clear()
        for (contour in input) {
            var score = 0.0
            for ((weight, scorer) in scorers) {
                score += weight * scorer.score(contour)
            }
            if (score < 1.0) {
                output.add(contour)
            }
        }
        return output
    }
}
