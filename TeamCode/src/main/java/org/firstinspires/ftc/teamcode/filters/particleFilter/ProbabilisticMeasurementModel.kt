package org.firstinspires.ftc.teamcode.filters.particleFilter

import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.MeasurementModel

interface ProbabilisticMeasurementModel : MeasurementModel {
    fun calculateProbability(z: SimpleMatrix, state: SimpleMatrix) : Double
}