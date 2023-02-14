package org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator
import com.acmerobotics.roadrunner.profile.MotionState
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.gravityFeedforward
import org.firstinspires.ftc.teamcode.RobotConfig.leftLiftName
import org.firstinspires.ftc.teamcode.RobotConfig.liftCoeffs
import org.firstinspires.ftc.teamcode.RobotConfig.liftDPP
import org.firstinspires.ftc.teamcode.RobotConfig.liftHeightOffset
import org.firstinspires.ftc.teamcode.RobotConfig.liftKA
import org.firstinspires.ftc.teamcode.RobotConfig.liftKStatic
import org.firstinspires.ftc.teamcode.RobotConfig.liftKV
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxAccel
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxExtension
import org.firstinspires.ftc.teamcode.RobotConfig.liftMaxVel
import org.firstinspires.ftc.teamcode.RobotConfig.liftTargetErrorTolerance
import org.firstinspires.ftc.teamcode.RobotConfig.magnetLimitName
import org.firstinspires.ftc.teamcode.RobotConfig.poleLiftOffset
import org.firstinspires.ftc.teamcode.RobotConfig.rightLiftName
import org.firstinspires.ftc.teamcode.RobotConfig.withinSwitchRange
import org.firstinspires.ftc.teamcode.filters.kalmanFilter.*
import org.firstinspires.ftc.teamcode.util.arrayToColumnMatrix
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

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

    private val limit = hwMap.get(TouchSensor::class.java, magnetLimitName)

    private val controller = PIDFController(liftCoeffs, liftKStatic, liftKV, liftKA)
    private lateinit var motionProfile: MotionProfile

    private var filter: KalmanFilter
    private var state: DoubleArray = doubleArrayOf()

    private val profileTimer = ElapsedTime()
    private val timer = ElapsedTime()

    /**
     * @return Target height off the ground in cm.
     */
    var setpoint: Double = 0.0
        /**
         * Sets the target extension length and constructs an optimal motion profile.
         * @param height        Target length in cm.
         */
        set(length) {
            profileTimer.reset()
            field = Range.clip(length, 0.0, liftMaxExtension)
            motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                MotionState(getExtensionLength(), getVelocity(), getAcceleration()),
                MotionState(field, 0.0, 0.0),
                liftMaxVel,
                liftMaxAccel
            )
            Log.i("Lift setpoint", length.toString())
        }

    init {

        motorGroup.encoder.reset()

        val processModel = ConstantAccelerationProcessModel()

        val H = SimpleMatrix(arrayOf(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0)))
        val R = SimpleMatrix(arrayOf(doubleArrayOf(1.0, 0.0), doubleArrayOf(0.0, 2.0)))
        val measurementModel = LinearMeasurementModel(H, R)

        // Retracted and stationary
        val initialState = SimpleMatrix(arrayOf(doubleArrayOf(0.0, 0.0, 0.0))).transpose()
        // Completely sure that it is retracted and stationary
        val initialCovariance = SimpleMatrix(arrayOf(doubleArrayOf(0.0)))
        filter = KalmanFilter(processModel, measurementModel, initialState, initialCovariance)

        motorGroup.setDistancePerPulse(liftDPP)
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

        retract()
    }

    lateinit var desiredState: MotionState
    override fun periodic() {
        // TODO: Maybe only set power if it has actually changed!! Do this through thresholding
        //  integrating current power with desired power. Write wrappers for automatic voltage
        //  compensation.
        //  Naive optimization would only write when motion profiling is active; heavily trusts ff

        desiredState = motionProfile[profileTimer.seconds()]

        controller.apply {
            targetPosition = desiredState.x
            targetVelocity = desiredState.v
            targetAcceleration = desiredState.a + gravityFeedforward
        }

        checkEncoder()

        // Get current state estimates using kalman filter//
        val u = doubleArrayOf(desiredState.x-state[0], desiredState.v-state[1], desiredState.a-state[2]) // Subtract desired state with previous state estimate
        updateFilter(arrayToColumnMatrix(u))

        setPower(controller.update(getExtensionLength(), getVelocity()))

        timer.reset()
    }

    fun updateFilter(u: SimpleMatrix?) {
        val dt = timer.time()
        filter.predict(u, dt)

        val z = doubleArrayOf(getRawExtensionLength(), getRawVelocity())
        Log.v("Raw state", z.toString())
        filter.update(arrayToColumnMatrix(z))

        state = filter.stateEstimate.ddrm.data
        Log.v("Estimated state", state.toString())

        timer.reset()
    }

    /**
     * Resets encoder position if necessary
     */
    private var checkLimit = false
    fun checkEncoder() {
        if (checkLimit && getExtensionLength() <= withinSwitchRange) {
            if (limit.isPressed) {
                motorGroup.encoder.reset()
                checkLimit = false // Only need to check limit switch once
                Log.i("Limit Switch", "Resetting")
            }
        }
    }

    fun setHeight(height: Double) {
        setpoint = (height - liftHeightOffset) / sin(toRadians(60.0)) // It will be extending upwards so no need to check
        checkLimit = false
    }
    /**
     * Sets the target height of the lift and constructs an optimal motion profile for it.
     * @param pole          Based on [PoleType] heights.
     */
    fun setHeight(pole: RobotConfig.PoleType) {
        setHeight(pole.height + poleLiftOffset)
    }

    /**
     * Retracts the lift to the starting height.
     */
    fun retract() {
        setpoint = 0.0
        checkLimit = true
    }

    /**
     * Set power of lift.
     * @param power         Percentage of the maximum speed of the lift.
     */
    fun setPower(power: Double) {
        motorGroup.set(power * 12.0 / batteryVoltageSensor.voltage)
    }

    /**
     * Returns the position of where the passthrough is attached in tangent space.
     * TODO: Add starting x
     */
    fun getRelativePosition(): Pose2d {
        return Pose2d(getExtensionLength() * cos(toRadians(60.0)), 0.0, 0.0)
    }

    /**
     * @return Distance the lift has extended relative to retracted state in cm.
     */
    fun getExtensionLength(): Double {
        return state[0]
    }

    /**
     * @return Velocity of the lift in cm / s.
     */
    fun getVelocity(): Double {
        return state[1]
    }

    /**
     * @return Acceleration of the lift in cm / s2.
     */
    fun getAcceleration(): Double {
        return state[2]
    }

    /**
     * @return Raw lift position has extended relative to retracted state in cm.
     */
    fun getRawExtensionLength(): Double {
        return motorGroup.distance
    }

    /**
     * @return Raw velocity of the lift in cm / s.
     */
    fun getRawVelocity(): Double {
        return motorGroup.correctedVelocity * liftDPP
    }


    /**
     * @return True if the controller has reached the target with some tolerance.
     */
    fun atTarget(): Boolean {
        return abs(getExtensionLength() - setpoint) < liftTargetErrorTolerance
    }

    /**
     * @return Time remaining from reaching the target.
     */
    fun timeFromTarget(): Double {
        return motionProfile.duration() - profileTimer.seconds()
    }

    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(telemetry: Telemetry) {
        telemetry.addData("Lift raw length", getRawExtensionLength())
        telemetry.addData("Lift raw velocity", getRawVelocity())
        telemetry.addData("Lift raw acceleration", motorGroup.encoder.acceleration * liftDPP)
        telemetry.addData("Lift estimated length", getExtensionLength())
        telemetry.addData("Lift estimated velocity", getVelocity())
        telemetry.addData("Lift estimated acceleration", getAcceleration())
        telemetry.addData("Target velocity", desiredState.v)
        telemetry.addData("Velocity Error", desiredState.v - getVelocity())
    }

}