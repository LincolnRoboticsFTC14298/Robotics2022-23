package org.firstinspires.ftc.teamcode.subsystems.drive.localization

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.particleFilter.ProbabilisticMeasurementModel
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.matrixToPose
import org.firstinspires.ftc.teamcode.util.vectorToDoubleArray
import org.opencv.core.Point

class VisionMeasurementModel(private val vision: Vision) : ProbabilisticMeasurementModel {

    override fun predictObservation(state: SimpleMatrix): SimpleMatrix {
        TODO("Not yet implemented")
    }

    override fun calculateProbability(z: SimpleMatrix, state: SimpleMatrix): Double {
        val probability = 1.0

        // Each column of z is an observation of a pole
        val expected = predictObservation(state)
        return 0.0
    }

    fun getCorrespondingLandmarkPosition(z: SimpleMatrix, state: SimpleMatrix) {
        val landmarks = Array<SimpleMatrix>(z.numCols()) { i ->
            val pixel = Point(z.get(0, i), z.get(1, i))
            val realHeight = z.get(2, i)
            val globalVector = vision.pixelToGlobalPosition(pixel, realHeight, matrixToPose(state))
            val globalZ = SimpleMatrix(arrayOf(vectorToDoubleArray(globalVector)))
            globalZ
        }
    }

}