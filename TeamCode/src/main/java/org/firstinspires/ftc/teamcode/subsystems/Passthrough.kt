package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.ServoEx
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughDepositAngle
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMaxDegree
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMinDegree
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughPickUpAngle
import org.firstinspires.ftc.teamcode.RobotConfig.leftPassthroughName
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughJunctionAngle
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMaxAccel
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughMaxVel
import org.firstinspires.ftc.teamcode.RobotConfig.passthroughTimeTolerance
import org.firstinspires.ftc.teamcode.RobotConfig.rightPassthroughName
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos


/**
 * Passthrough subsystem consist of a servo that rotates
 * the mechanism containing the claw from pick up to drop off.
 * The angle is that made with the heading vector (i.e. w/ the front of the intake side)
 * @param hwMap             HardwareMap
 */
class Passthrough(hwMap: HardwareMap) : SubsystemBase() {

    /**
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware">FTCLib Docs: Hardware</a>
     */
    private val servoLeft: ServoEx = SimpleServo(
        hwMap,
        leftPassthroughName,
        passthroughMinDegree,
        passthroughMaxDegree
    )
    private val servoRight: ServoEx = SimpleServo(
        hwMap,
        rightPassthroughName,
        passthroughMinDegree,
        passthroughMaxDegree
    )

    private var timer = ElapsedTime()
    private lateinit var motionProfile: MotionProfile

    var setpoint: Double = 0.0
        set(angle) {
            field = angle
            timer.reset()
            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(getAngleEstimate(), 0.0, 0.0),
                MotionState(angle, 0.0, 0.0),
                passthroughMaxVel,
                passthroughMaxAccel
            )
            Log.i("Passthrough desired angle", setpoint.toString())
        }

    init {
        pickUp()
    }

    // TODO: check direction of servo

    override fun periodic() {
        Log.v("Passthrough angle estimate", getAngleEstimate().toString())
        if (!atTarget()) {
            val state = motionProfile[timer.seconds()]
            servoLeft.turnToAngle(state.x)
            servoRight.turnToAngle(state.x)
        }
    }

    fun junctionDeposit() {
        setpoint = passthroughJunctionAngle
    }

    /**
     * Rotate servo to drop off position.
     */
    fun deposit() {
        setpoint = passthroughDepositAngle
    }

    /**
     * Rotate servo to pick up cone.
     */
    fun pickUp() {
        setpoint = passthroughPickUpAngle
    }

    fun setPosition(position: Double) {
        servoRight.position = position
        servoLeft.position = position
    }

    /**
     * @return The position of the passthrough relative to where it is attached on the lift.
     */
    fun getRelativePosition(): Pose2d {
        val currAngle = getAngleEstimate()
        val heading = if (currAngle <= 90.0) -180.0 else 0.0
        return Pose2d(-cos(toRadians(currAngle)), 0.0, heading) // TODO: Include offset and passthrough length
    }

    /**
     * @return Angle estimate based on motion profiling in degrees.
     */
    fun getAngleEstimate() : Double {
        return servoLeft.angle
    }

    /**
     * @return If at desired angle based on time.
     */
    fun atTarget(): Boolean {
        return timer.seconds() - motionProfile.duration() > passthroughTimeTolerance
    }

    fun timeToTarget(angle: Double): Double {
        val testProfile = MotionProfileGenerator.generateSimpleMotionProfile(
            MotionState(getAngleEstimate(), 0.0, 0.0),
            MotionState(angle, 0.0, 0.0),
            passthroughMaxVel,
            passthroughMaxAccel
        )
        return testProfile.duration()
    }

    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(telemetry: Telemetry) {
        telemetry.addData("Passthrough desired angle", setpoint)
    }

}