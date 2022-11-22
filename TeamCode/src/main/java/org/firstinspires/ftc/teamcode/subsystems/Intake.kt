package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

/**
 * Intake subsystem consists of a continuous rotating servo
 * that picks up cones and a touch sensor to detect it.
 * @param hwMap HardwareMap.
 * @param servoName Intake continuous servo's name.
 * @param sensorName Touch Sensor Name
 */
class Intake(hwMap: HardwareMap, servoName: String, sensorName: String) : SubsystemBase() {

    /**
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val servo: CRServo = CRServo(hwMap, servoName)
    private val sensor: TouchSensor = hwMap.get(TouchSensor::class.java, sensorName)

    private val speed = 0.5

    /**
     * Turn servo on at a constant speed to pick up cone.
     */
    fun suckIn() {
        servo.set(speed)
    }

    /**
     * Turn servo on at a constant speed to spit out cone.
     */
    fun spitOut() {
        servo.set(-speed)
    }

    /**
     * Turn servo off.
     */
    fun off() {
        servo.stop()
    }

    /**
     * Check if there is a cone based on touch sensor.
     * @return [Boolean] of state.
     */
    fun isConeInside(): Boolean {
        return sensor.isPressed
    }
}