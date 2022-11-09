package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Claw subsystem consists of a servo with a touch sensor that picks up cones.
 * @param hardwareMap HardwareMap.
 * @param servoName Claw's servo's name.
 */
class ClawSubsystem(hardwareMap: HardwareMap, servoName: String) : SubsystemBase() {

    /**
     * TODO: Set min and max degrees.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servo: ServoEx = SimpleServo(hardwareMap, servoName, 0.0, 1.0)
    // TODO: add touch sensor

    /**
     * Rotate servo to drop cone.
     */
    fun drop() {

    }

    /**
     * Rotate servo to pick up cone.
     */
    fun pickUp() {

    }

    /**
     * Check if there is a cone based on touch sensor.
     * @return [Boolean] of state.
     */
    fun isConeTouched(): Boolean {
        return false
    }

}