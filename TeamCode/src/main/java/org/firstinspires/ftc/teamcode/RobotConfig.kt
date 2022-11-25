package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.drive.SampleMecanumDrive

object RobotConfig {

    /****************************************************
     * Field                                            *
     ****************************************************/

    const val tileSize = .595 // m


    /**
     * Different types of poles.
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

        val vector = Vector2d(x.toDouble() * tileSize, y.toDouble() * tileSize)

        companion object {
            fun getPole(vector: Vector2d): Pole? {
                values().forEach { pole ->
                    if (pole.vector.epsilonEquals(vector)) {
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

        val vector = Vector2d(x.toDouble() * tileSize, y.toDouble() * tileSize)

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
     * [Mecanum]
     */
    val trackingCoeffs = PIDCoefficients(0.0, 0.0, 0.0) // For rotation

    /**
     * [Lift]
     */
    const val liftLeftMotorName = "liftLeft"
    const val liftRightMotorName = "liftRight"

    const val liftHeightOffset = 0.0 // cm The raw height of zero is off the ground
    const val liftMaxHeight = 0.0 // cm Max allowable extension height

    const val liftDPP = 1.0 // TODO: Find experimentally

    const val liftMaxVel = 25.0 // cm / s  // TODO: Find max values
    const val liftMaxAccel = 40.0 // cm / s2

    const val liftKStatic = 0.0
    const val liftKV = 1.0 / liftMaxVel
    const val liftKA = 0.0

    val liftCoeffs = PIDCoefficients(0.0, 0.0, 0.0) // TODO: Calculate from kV and kA

    const val liftTargetErrorTolerance = 0.5 // cm

    const val poleLiftOffset = 10.0 // cm above the pole the lift should be at

    /**
     * [Intake]
     */
    const val intakeServoName = "intake"
    const val intakeTouchSensorName = "intakeTouch"
    val intakePosition = Pose2d(0.0, 0.0, 0.0) // Position of the center of the intake during pick up position

    const val intakeTimeToSpitOut = 500 // milliseconds

    const val intakeSpeed = 0.5

    /**
     * Passthrough
     */
    const val passthroughServoLeftName = "passthroughLeft"
    const val passthroughServoRightName = "passthroughRight"
    const val potentiometerName = "potentiometer"

    const val passthroughMinDegree = 0.0
    const val passthroughMaxDegree = 0.0
    // TODO: Make degrees?
    const val passthroughRetractedAngle = -15.0
    const val passthroughDepositAngle = 180.0 // degrees

    const val potentiometerOffset = 0.0 // degrees




    /****************************************************
     * Commands                                         *
     ****************************************************/

    const val poleDepositAnticipationTime: Double = 2.0 // Time in seconds the passthrough starts moving before the lift reaches its target height.

}