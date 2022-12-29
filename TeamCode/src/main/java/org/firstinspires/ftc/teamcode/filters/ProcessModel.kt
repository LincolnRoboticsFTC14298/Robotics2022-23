package org.firstinspires.ftc.teamcode.filters

import org.ejml.simple.SimpleMatrix

interface ProcessModel {
    fun predictState(previousState: SimpleMatrix, u: SimpleMatrix?, dt: Double): SimpleMatrix
}