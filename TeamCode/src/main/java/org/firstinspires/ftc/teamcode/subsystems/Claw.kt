package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor

/**
 * Claw subsystem consists of a servo with a touch sensor that picks up cones.
 * @param hwMap HardwareMap.
 * @param servoName Claw's servo's name.
 * @param sensorName Name of the touch sensor
 */
class Claw(hwMap: HardwareMap, servoName: String, sensorName: String) : SubsystemBase() {

    /**
     * TODO: Set min and max degrees.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servo: ServoEx = SimpleServo(hwMap, servoName, 0.0, 1.0)
    private val sensor: TouchSensor = hwMap.get(TouchSensor::class.java, sensorName)

    private val MIN = 0.0
    private val MAX = 0.7

    /**
     * Rotate servo to drop cone.
     */
    fun drop() {
        servo.position = MAX
    }

    /**
     * Rotate servo to pick up cone.
     */
    fun grab() {
        servo.position = MIN
    }

    /**
     * Check if there is a cone based on touch sensor.
     * @return [Boolean] of state.
     */
    fun isConeTouched(): Boolean {
        return sensor.isPressed
    }

}