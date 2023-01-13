package org.firstinspires.ftc.teamcode.filters

import org.ejml.simple.SimpleMatrix

interface MeasurementModel {
    fun predictObservation(state: SimpleMatrix): SimpleMatrix
}