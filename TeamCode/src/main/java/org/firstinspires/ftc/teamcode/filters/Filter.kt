package org.firstinspires.ftc.teamcode.filters

interface Filter<StateType, MeasurementType> {
    var stateEstimate: StateType
    fun update(z: MeasurementType)
}