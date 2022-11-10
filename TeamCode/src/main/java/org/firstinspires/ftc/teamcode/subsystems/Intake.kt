package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Intake subsystem consists of a continuous rotating servo
 * that picks up cones and a touch sensor to detect it.
 * @param hwMap HardwareMap.
 * @param servoName Intake continuous servo's name.
 * @param sensorName Touch Sensor Name
 */
class Intake(hwMap: HardwareMap, servoName: String, sensorName: String) : SubsystemBase()
{

    /**
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val servo = CRServo(hwMap, servoName)
    private val sensor = hwMap.touchSensor.get(sensorName)

    private val SPD = 0.5

    /**
     * Turn servo on to a constant speed.
     */
    fun on()
    {
        servo.set(SPD)
    }

    /**
     * Turn servo off.
     */
    fun off()
    {
        servo.stop()
    }

    /**
     * Check if there is a cone based on touch sensor.
     * @return [Boolean] of state.
     */
    fun isConeInside(): Boolean
    {
        return sensor.isPressed
    }
}