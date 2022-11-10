package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Lift consists of two multistage slides powered by a motor which pulls a string.
 * @param hardwareMap HardwareMap.
 * @param leftMotorName Lift's left motor's name.
 * @param rightMotorName Lift's right motor's name.
 */
class Lift(hardwareMap: HardwareMap, leftMotorName: String, rightMotorName: String)  : SubsystemBase()
{

    /**
     * Avoid using the individual motors, it's best to use the group.
     * TODO: reverse motor if appropriate
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val leftMotor  = Motor(hardwareMap, leftMotorName)
    private val rightMotor = Motor(hardwareMap, rightMotorName)
    private val motorGroup = MotorGroup(leftMotor, rightMotor) // TODO: Set zero power behavior to break

    /**
     * Set power of lift.
     * @param power Percentage of the maximum speed of the lift.
     */
    fun setPower(power: Double)
    {
        motorGroup.set(power)
    }

    /**
     * Stop lift.
     */
    fun stop()
    {
        motorGroup.stopMotor()
    }

    /**
     * Get vertical component of the lift extension using encoder ticks.
     * @return Height of the lift in cm.
     */
    fun getHeight(): Double
    {
        return 0.0
    }
}