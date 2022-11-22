package org.firstinspires.ftc.teamcode.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Lift consists of two multistage slides powered by a motor which pulls a string.
 * @param hardwareMap        HardwareMap.
 * @param leftMotorName      Lift's left motor's name.
 * @param rightMotorName     Lift's right motor's name.
 */
class Lift(hwMap: HardwareMap, leftMotorName: String, rightMotorName: String)  : SubsystemBase() {

    /**
     * Avoid using the individual motors, it's best to use the group.
     * TODO: reverse motor if appropriate.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val leftMotor  = Motor(hwMap, leftMotorName)
    private val rightMotor = Motor(hwMap, rightMotorName)
    private val motorGroup = MotorGroup(leftMotor, rightMotor)

    init {
        motorGroup.setDistancePerPulse(1.0) // TODO: Experimentally find value.
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE) // TODO: is this desirable w/ motion profiling?
    }

    /**
     * Set power of lift.
     * TODO: Account for voltage of the robot
     * @param power Percentage of the maximum speed of the lift.
     */
    fun setPower(power: Double) {
        motorGroup.set(power)
    }

    /**
     * Stop lift.
     */
    fun stop() {
        motorGroup.stopMotor()
    }

    /**
     * Get vertical component of the lift extension using encoder ticks.
     * @return Height of the lift in cm.
     */
    fun getHeight(): Double {
        return motorGroup.distance
    }

    /**
     * Get vertical component of the velocity using encoder ticks.
     * @return Velocity of the lift in cm / s.
     */
    fun getVelocity(): Double {
        return motorGroup.rate
    }

}