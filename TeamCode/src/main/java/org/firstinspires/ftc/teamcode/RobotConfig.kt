package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.localization.OdometryLocalizer
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import java.lang.Math.toRadians

@Config
object RobotConfig {

    /****************************************************
     * Field                                            *
     ****************************************************/

    const val tileSize = 23.5 // in

    const val coneDiameter = 5 // in TODO measure
    const val poleDiameter = 2 // in TODO measure
    const val poleBaseHeight = 5 //in TODO measure

    /**
     * Different types of poles. Height in inches.
     */
    enum class PoleType(val height: Double) {
        LOW(0.0),
        MEDIUM(0.0),
        HIGH(0.0)
    }

    /**
     * All poles on the field.
     */
    enum class Pole(x: Int, y: Int, val type: PoleType) {
        TWO_ONE(2, 1, PoleType.LOW),
        TWO_NEG_ONE(2, -1, PoleType.LOW),

        ONE_TWO(1, 2, PoleType.LOW),
        ONE_ONE(1, 1, PoleType.MEDIUM),
        ONE_ZERO(1, 0, PoleType.HIGH),
        ONE_NEG_ONE(1, -1, PoleType.MEDIUM),
        ONE_NEG_TWO(1, -2, PoleType.LOW),

        ZERO_ONE(0, 1, PoleType.HIGH),
        ZERO_NEG_ONE(0, -1, PoleType.HIGH),

        NEG_ONE_TWO(-1, 2, PoleType.LOW),
        NEG_ONE_ONE(-1, 1, PoleType.MEDIUM),
        NEG_ONE_ZERO(-1, 0, PoleType.HIGH),
        NEG_ONE_NEG_ONE(-1, -1, PoleType.MEDIUM),
        NEG_ONE_NEG_TWO(-1, -2, PoleType.LOW),

        NEG_TWO_ONE(-2, 1, PoleType.LOW),
        NEG_TWO_NEG_ONE(-2, -1, PoleType.LOW);

        val vector = Vector2d(x * tileSize, y * tileSize)

        companion object {
            fun getPole(vector: Vector2d): Pole? {
                values().forEach { pole ->
                    if (pole.vector epsilonEquals vector) {
                        return pole
                    }
                }
                return null
            }
        }
    }

    enum class Junction(x: Int, y: Int) {
        TWO_TWO(2, 2),
        TWO_ZERO(2, 0),
        TWO_NEG_TWO(2, -2),

        ZERO_TWO(0, 2),
        ZERO_ZERO(0, 0),
        ZERO_NEG_TWO(0,-2),

        NEG_TWO_TWO(-2, 2),
        NEG_TWO_ZERO(-2, 0),
        NEG_TWO_NEG_TWO(-2, -2);

        val vector = Vector2d(x * tileSize, y * tileSize)

        companion object {
            fun getJunction(vector: Vector2d): Junction? {
                values().forEach { junction ->
                    if (junction.vector.epsilonEquals(vector)) {
                        return junction
                    }
                }
                return null
            }
        }
    }

    /****************************************************
     * Subsystems                                       *
     ****************************************************/

    /**
     * [SampleMecanumDrive]
     */
    @JvmField
    var driveLeftFront: String = "leftFront"

    @JvmField
    var driveLeftRear: String = "leftRear"

    @JvmField
    var driveRightRear: String = "rightRear"

    @JvmField
    var driveRightFront: String = "rightFront"

    /**
     * [OdometryLocalizer]
     */
    const val leftEncoderName = "leftEncoder" // Must be ports 0 and 3
    const val rightEncoderName = "rightEncoder"
    const val frontEncoderName = "frontEncoder" // port 1 or 2

    var TICKS_PER_REV = 0.0
    var WHEEL_RADIUS = 2.0 // in
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed

    @JvmField
    var X_MULTIPLIER = 1.0 // TODO multipliers for each individual encoder
    @JvmField
    var Y_MULTIPLIER = 1.0
    @JvmField
    var LATERAL_DISTANCE = 10.0 // in; distance between the left and right wheels
    @JvmField
    var FORWARD_OFFSET = 4.0 // in; offset of the lateral wheel

    /**
     * [Lift]
     */
    const val leftLiftName = "leftLift"
    const val rightLiftName = "rightLift"
    const val magnetLimitName = "magnet"

    const val liftHeightOffset = 0.0 // in The raw height of zero is off the ground
    const val liftMaxExtension = 0.0 // in Max allowable extension height
    const val poleLiftOffset = 20.0 // in above the pole the lift should be at

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


    @JvmField
    var liftCoeffs = PIDCoefficients(0.0, 0.0, 0.0) // TODO: Calculate from kV and kA

    val liftTargetErrorTolerance = 0.5 // in

    const val withinSwitchRange = 3.0 // in from the bottom to check magnet switch for reset

    /**
     * [Claw]
     */
    const val clawServoName = "claw"
    const val colorSensorName = "color"

    @JvmField
    var clawClosedPosition = 0.85
    @JvmField
    var clawOpenedPosition = 0.73
    @JvmField
    var clawPartiallyOpenedPosition = 0.80


    @JvmField
    var clawMaxVel = 0.7
    @JvmField
    var clawMaxAccel = 4.5

    @JvmField
    var colorGain = 20.0
    @JvmField
    var valueThreshold = 0.15




    /**
     * Passthrough
     */
    const val leftPassthroughName = "leftPassthrough"
    const val rightPassthroughName = "rightPassthrough"

    const val passthroughMinDegree = -45.0 // degrees
    const val passthroughMaxDegree = 180.0 // degrees

    const val passthroughOffsetDistanceFromLift = 0.0

    @JvmField
    var passthroughPickUpAngle = -45.0 // degrees
    @JvmField
    var passthroughDepositAngle = 180.0 // degrees
    @JvmField
    var passthroughJunctionAngle = -35.0

    @JvmField
    var passthroughMaxVel = 0.0
    @JvmField
    var passthroughMaxAccel = 0.0

    @JvmField
    var passthroughTimeTolerance = 0.2 // Seconds to wait after motion profile supposedly complete

    /**
     * Vision
     */
    enum class CameraData(val pitch: Double, val height: Double, val relativePosition: Vector2d, val FOVX: Double, val FOVY: Double, val fx: Double, val fy: Double, val cx: Double, val cy: Double, val distCoeffs: MatOfDouble) {
        PHONECAM(0.0, 2.0, Vector2d(-5.0, 0.0), toRadians(60.0), toRadians(60.0), 0.0, 0.0, 0.0, 0.0, MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)),
        LOGITECH_C920(0.0, 3.0, Vector2d(5.0, 0.0), toRadians(60.0), toRadians(60.0), 578.272, 578.272, 402.145, 221.506, MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0));

        fun getCameraMatrix(): Mat {
            val cameraMat = Mat(3, 3, CvType.CV_64FC1)
            val cameraData = doubleArrayOf(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
            cameraMat.put(0, 0, *cameraData)
            return cameraMat
        }
    }

    const val stackToPoleMaximumDistance = 5.0






    /****************************************************
     * Commands                                         *
     ****************************************************/

    // Time in seconds the passthrough starts moving before the lift reaches its target height.
    @JvmField
    val poleDepositAnticipationTime = 0.5


    /****************************************************
     * Driver Controls                                  *
     ****************************************************/

    const val teleOpSetPointAdj = 1.0
    const val teleOpDepositAdj = 5.0
}