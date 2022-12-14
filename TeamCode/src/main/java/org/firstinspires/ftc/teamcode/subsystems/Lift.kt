package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.leftLiftName
import org.firstinspires.ftc.teamcode.RobotConfig.liftCoeffs
import org.firstinspires.ftc.teamcode.RobotConfig.liftDPP
import org.firstinspires.ftc.teamcode.RobotConfig.liftHeightOffset
import org.firstinspires.ftc.teamcode.RobotConfig.liftKA
import org.firstinspires.ftc.teamcode.RobotConfig.liftKStatic
import org.firstinspires.ftc.teamcode.RobotConfig.liftKV
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxAccel
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxHeight
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxVel
import org.firstinspires.ftc.teamcode.RobotConfig.liftTargetErrorTolerance
import org.firstinspires.ftc.teamcode.RobotConfig.poleLiftOffset
import org.firstinspires.ftc.teamcode.RobotConfig.rightLiftName
import kotlin.math.abs

/**
 * Lift consists of two multistage slides powered by a motor which pulls a string.
 * @param hwMap        HardwareMap.
 */
class Lift(hwMap: HardwareMap) : SubsystemBase() {

    /**
     * Avoid using the individual motors, it's best to use the group.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val leftMotor  = Motor(hwMap, leftLiftName)
    private val rightMotor = Motor(hwMap, rightLiftName)
    private val motorGroup = MotorGroup(leftMotor, rightMotor)

    private val batteryVoltageSensor = hwMap.voltageSensor.iterator().next()


    private val controller = PIDFController(
        liftCoeffs,
        liftKStatic,
        liftKV,
        liftKA
    )

    private lateinit var motionProfile: MotionProfile

    /**
     * @return Target height off the ground in cm.
     */
    var setpoint = 0.0
        private set

    private val timer = ElapsedTime()

    init {
        // TODO: reverse motor if appropriate.
        motorGroup.setDistancePerPulse(liftDPP)
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)

        retract()
    }

    override fun periodic() {
        val state = motionProfile[timer.seconds()]

        controller.apply {
            targetPosition = state.x
            targetVelocity = state.v
            targetAcceleration = state.a + 981.0 // gravity term
        }

        setPower(controller.update(getCurrentHeight(), getCurrentVelocity()))
    }

    /**
     * @return Height of the lift relative to the ground in cm.
     */
    fun getCurrentHeight(): Double {
        return motorGroup.distance + liftHeightOffset
    }


    /**
     * @return Velocity of the lift in cm / s.
     */
    fun getCurrentVelocity(): Double {
        return motorGroup.rate
    }

    /**
     * @return Acceleration of the lift in cm / s2.
     */
    fun getCurrentAcceleration(): Double {
        return liftDPP * motorGroup.encoder.acceleration
    }

    /**
     * Sets the target height of the lift and constructs an optimal motion profile for it.
     * @param height        Target height off the ground in cm.
     */
    fun setSetpoint(height: Double) {
        timer.reset()
        setpoint = Range.clip(height, liftHeightOffset, liftMaxHeight)
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(getCurrentHeight(), getCurrentVelocity(), getCurrentAcceleration()),
            MotionState(setpoint, 0.0, 0.0),
            liftMaxVel,
            liftMaxAccel
        )
    }

    /**
     * Sets the target height of the lift and constructs an optimal motion profile for it.
     * @param pole          Based on [PoleType] heights.
     */
    fun setSetpoint(pole: RobotConfig.PoleType) {
        setSetpoint(pole.height + poleLiftOffset)
    }

    fun retract() {
        setSetpoint(liftHeightOffset) // The zero point
    }

    /**
     * Set power of lift.
     * @param power         Percentage of the maximum speed of the lift.
     */
    private fun setPower(power: Double) {
        motorGroup.set(power * 12.0 / batteryVoltageSensor.voltage)
    }

    /**
     * @return True if the controller has reached the target with some tolerance.
     */
    fun atTarget(): Boolean {
        return abs(controller.lastError) < liftTargetErrorTolerance
    }

    /**
     * @return Time remaining from reaching the target.
     */
    fun timeFromTarget(): Double {
        return motionProfile.duration() - timer.seconds()
    }

}