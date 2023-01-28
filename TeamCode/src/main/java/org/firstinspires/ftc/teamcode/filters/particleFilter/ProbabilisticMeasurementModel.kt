package org.firstinspires.ftc.teamcode.filters.particleFilter

import org.ejml.simple.SimpleMatrix

interface ProbabilisticMeasurementModel {
    fun calculateProbability(z: SimpleMatrix, state: SimpleMatrix) : Double
}