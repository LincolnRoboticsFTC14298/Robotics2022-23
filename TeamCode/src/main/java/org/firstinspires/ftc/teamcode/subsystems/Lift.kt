package org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.util.PIDFController
import android.util.Log
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.*
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorGroup
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.filters.kalmanFilter.*
import org.firstinspires.ftc.teamcode.util.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.arrayToColumnMatrix
import java.lang.Math.toRadians
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

/**
 * Lift consists of two multistage slides powered by a motor which pulls a string.
 * @param hwMap        HardwareMap.
 */
@Config
class Lift(hwMap: HardwareMap, private val voltageSensor: VoltageSensor) : SubsystemBase() {

    /**
     * Avoid using the individual motors, it's best to use the group.
     * @see <a href="https://docs.ftclib.org/ftclib/features/hardware/motors">FTCLib Docs: Motors</a>
     */
    private val leftMotor  = Motor(hwMap, leftLiftName)
    private val rightMotor = Motor(hwMap, rightLiftName)
    private val motorGroup = MotorGroup(leftMotor, rightMotor)

    private val limit = hwMap.get(TouchSensor::class.java, magnetLimitName)

    private val controller = PIDFController(liftCoeffs, liftKStatic, liftKV, liftKA)
    private lateinit var motionProfile: TimeProfile

    private var filter: KalmanFilter
    private var state: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0)

    private val profileTimer = ElapsedTime()
    private val timer = ElapsedTime()

    /**
     * @return Target height off the ground in in.
     */
    var setpoint: Double = 0.0
        /**
         * Sets the target extension length and constructs an optimal motion profile.
         * @param height        Target length in inches.
         */
        set(length) {
            profileTimer.reset()
            field = Range.clip(length, 0.0, liftMaxExtension)
            motionProfile = TimeProfile(constantProfile(field - getExtensionLength(), 0.0, liftMaxVel, -liftMaxAccel, liftMaxAccel).baseProfile)
            Log.i("Lift setpoint", length.toString())
        }

    init {

        val processModel = ConstantAccelerationProcessModel()

        val H = SimpleMatrix(arrayOf(doubleArrayOf(1.0, 0.0, 0.0), doubleArrayOf(0.0, 1.0, 0.0)))
        val R = SimpleMatrix(arrayOf(doubleArrayOf(0.5, 0.0), doubleArrayOf(0.0, 2.0)))
        val measurementModel = LinearMeasurementModel(H, R)

        // Retracted and stationary
        val initialState = SimpleMatrix(arrayOf(doubleArrayOf(0.0, 0.0, 0.0))).transpose()
        // Completely sure that it is retracted and stationary
        val initialCovariance = SimpleMatrix(arrayOf(doubleArrayOf(0.0)))
        filter = KalmanFilter(processModel, measurementModel, initialState, initialCovariance)

        motorGroup.resetEncoder()

        motorGroup.setDistancePerPulse(liftDPP)
        motorGroup.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT)

        retract()
    }

    lateinit var desiredState: DoubleArray
    override fun periodic() {
        // TODO: Maybe only set power if it has actually changed!! Do this through thresholding
        //  integrating current power with desired power. Write wrappers for automatic voltage
        //  compensation.
        //  Naive optimization would only write when motion profiling is active; heavily trusts ff

        desiredState = motionProfile[profileTimer.seconds()].values().toDoubleArray()

        controller.apply {
            targetPosition = desiredState[0]
            targetVelocity = desiredState[1]
            targetAcceleration = desiredState[2] + gravityFeedforward
        }

        checkEncoder()

        // Get current state estimates using kalman filter//
        val u = desiredState.zip(state).map { it.first - it.second }.toDoubleArray()
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
    var checkLimit = false
    fun checkEncoder() {
        if (checkLimit && getExtensionLength() <= withinSwitchRange) {
            if (limit.isPressed) {
                motorGroup.resetEncoder()
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
    fun setHeight(pole: FieldConfig.PoleType) {
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
        motorGroup.set(power * 12.0 / voltageSensor.voltage)
    }

    /**
     * Returns the position of where the passthrough is attached in tangent space.
     */
    fun getRelativePosition(): Pose2d {
        return Pose2d(getExtensionLength() * cos(toRadians(60.0)) + liftOffsetDistanceFromCenter, 0.0, 0.0)
    }

    fun getFutureRelativePosition(): Pose2d {
        return Pose2d(setpoint * cos(toRadians(60.0)) + liftOffsetDistanceFromCenter, 0.0, 0.0)
    }

    /**
     * @return Distance the lift has extended relative to retracted state in in.
     */
    fun getExtensionLength(): Double {
        return state[0]
    }

    /**
     * @return Velocity of the lift in in / s.
     */
    fun getVelocity(): Double {
        return state[1]
    }

    /**
     * @return Acceleration of the lift in in / s2.
     */
    fun getAcceleration(): Double {
        return state[2]
    }

    /**
     * @return Raw lift position has extended relative to retracted state in in.
     */
    fun getRawExtensionLength(): Double {
        return motorGroup.distance
    }

    /**
     * @return Raw velocity of the lift in in / s.
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
        return motionProfile.duration - profileTimer.seconds()
    }

    /**
     * For debugging/tuning purposes
     */
    fun fetchTelemetry(packet: TelemetryPacket) {
        packet.put("Lift raw length", getRawExtensionLength())
        packet.put("Lift raw velocity", getRawVelocity())
        packet.put("Lift raw acceleration", motorGroup.encoder.acceleration * liftDPP)
        packet.put("Lift estimated length", getExtensionLength())
        packet.put("Lift estimated velocity", getVelocity())
        packet.put("Lift estimated acceleration", getAcceleration())
        packet.put("Target velocity", desiredState[1])
        packet.put("Velocity Error", desiredState[1] - getVelocity())
    }

    companion object {
        const val leftLiftName = "leftLift"
        const val rightLiftName = "rightLift"
        const val magnetLimitName = "magnet"

        const val liftHeightOffset = 0.0 // in The raw height of zero is off the ground

        const val liftMaxExtension = 0.0 // in Max allowable extension height
        const val poleLiftOffset = 5.0 // in above the pole the lift should be at

        const val liftDPP = 1.0 // TODO: Find experimentally

        const val liftOffsetDistanceFromCenter = 0.0

        @JvmField
        var liftMaxVel = 20.0 // in / s  // TODO: Find max values
        @JvmField
        var liftMaxAccel = 20.0 // in / s2

        @JvmField
        var liftKStatic = 0.0
        @JvmField
        var liftKV = 1.0 / liftMaxVel
        @JvmField
        var liftKA = 0.0
        @JvmField
        var gravityFeedforward = 0.0


        //@JvmField
        var liftCoeffs = PIDCoefficients(0.0, 0.0, 0.0) // TODO: Calculate from kV and kA

        val liftTargetErrorTolerance = 0.5 // in

        const val withinSwitchRange = 3.0 // in from the bottom to check magnet switch for reset
    }

}