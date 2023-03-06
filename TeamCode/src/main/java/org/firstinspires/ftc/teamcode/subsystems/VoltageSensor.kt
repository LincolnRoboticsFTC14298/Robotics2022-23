package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap

class VoltageSensor(hardwareMap: HardwareMap) : SubsystemBase() {

    private val voltageSensor = hardwareMap.voltageSensor.iterator().next()

    var voltage: Double = 0.0

    override fun periodic() {
        voltage = voltageSensor.voltage
    }

}