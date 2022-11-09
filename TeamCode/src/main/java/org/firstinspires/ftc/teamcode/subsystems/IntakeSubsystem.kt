package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Intake subsystem consists of a continuous rotating servo
 * that picks up cones and a touch sensor to detect it.
 * @param hardwareMap HardwareMap.
 * @param crServoName Intake's crservo's name.
 */
class IntakeSubsystem(hardwareMap: HardwareMap, crServoName: String) : SubsystemBase() {

    /**
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val crServo = CRServo(hardwareMap, crServoName)

    // TODO: add touch sensor

    /**
     * Turn servo on to a constant speed.
     */
    fun turnOn() {

    }

    /**
     * Turn servo off.
     */
    fun turnOff() {

    }

    /**
     * Check if there is a cone based on touch sensor.
     * @return [Boolean] of state.
     */
    fun isConeInside(): Boolean {
        return false
    }

}