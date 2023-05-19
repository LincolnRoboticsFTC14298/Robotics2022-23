package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dIncrDual
import com.acmerobotics.roadrunner.Vector2dDual
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.filters.ProcessModel
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.teamcode.util.Encoder.PositionVelocityPair

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class OdometryLocalizer(
    hardwareMap: HardwareMap
) : ProcessModel, Localizer {

    val par0: Encoder = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, leftEncoderName)))
    val par1: Encoder = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, rightEncoderName)))
    val perp: Encoder = OverflowEncoder(RawEncoder(hardwareMap.get(DcMotorEx::class.java, perpendicularEncoderName)))

    var lastPar0Pos: Int = par0.positionAndVelocity.position
    var lastPar1Pos: Int = par1.positionAndVelocity.position
    var lastPerpPos: Int = perp.positionAndVelocity.position

    var currPar0PosVel: PositionVelocityPair = par0.positionAndVelocity
    var currPar1PosVel: PositionVelocityPair = par1.positionAndVelocity
    var currPerpPosVel: PositionVelocityPair = perp.positionAndVelocity

    fun getIncr(par0PosVel: PositionVelocityPair, par1PosVel: PositionVelocityPair, perpPosVel: PositionVelocityPair): Twist2dIncrDual<Time> {
        val par0PosDelta = par0PosVel.position - lastPar0Pos
        val par1PosDelta = par1PosVel.position - lastPar1Pos
        val perpPosDelta = perpPosVel.position - lastPerpPos

        val twistIncr = Twist2dIncrDual(
            Vector2dDual(
                DualNum<Time>(
                    listOf(
                        (PAR0_Y_TICKS * par1PosDelta - PAR1_Y_TICKS * par0PosDelta) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                        (PAR0_Y_TICKS * par1PosVel.velocity - PAR1_Y_TICKS * par0PosVel.velocity) / (PAR0_Y_TICKS - PAR1_Y_TICKS)
                    )
                ) * MecanumDrive.IN_PER_TICK,
                DualNum<Time>(
                    listOf(
                        PERP_X_TICKS / (PAR0_Y_TICKS - PAR1_Y_TICKS) * (par1PosDelta - par0PosDelta) + perpPosDelta,
                        PERP_X_TICKS / (PAR0_Y_TICKS - PAR1_Y_TICKS) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity
                    )
                ) * MecanumDrive.IN_PER_TICK
            ),
            DualNum(
                listOf(
                    (par0PosDelta - par1PosDelta) / (PAR0_Y_TICKS - PAR1_Y_TICKS),
                    (par0PosVel.velocity - par1PosVel.velocity) / (PAR0_Y_TICKS - PAR1_Y_TICKS)
                )
            )
        )

        return twistIncr
    }

    /**
     * Only used for normal localizer
     */
    override fun updateAndGetIncr(): Twist2dIncrDual<Time> {
        updateReadings()
        return getIncr(currPar0PosVel, currPar1PosVel, currPerpPosVel)
    }

    fun updateReadings() {
        lastPar0Pos = currPar0PosVel.position
        lastPar1Pos = currPar1PosVel.position
        lastPerpPos = currPerpPosVel.position

        currPar0PosVel = par0.positionAndVelocity
        currPar1PosVel = par1.positionAndVelocity
        currPerpPosVel = perp.positionAndVelocity
    }

    fun getVelocity() = getIncr(currPar0PosVel, currPar1PosVel, currPerpPosVel).velocity().value()

    /**
     * Used for particle filter.
     */
    override fun predictState(previousState: SimpleMatrix, u: SimpleMatrix?, dt: Double): SimpleMatrix {
        return if (u != null) {
            val pose = matrixToPose(previousState)
            val noisyWheels = matrixToPostionVelocityPair(u)
            val incr = getIncr(noisyWheels[0], noisyWheels[1], noisyWheels[2])
            val newPose = pose.plus(incr.value())
            poseToMatrix(newPose)
        } else {
            previousState
        }
    }


    fun getReadings() = listOf(currPar0PosVel, currPar1PosVel, currPerpPosVel)
    fun getReadingsMatrix() = positionVelocityPairsToMatrix(getReadings())

    fun positionVelocityPairsToMatrix(pairs: List<PositionVelocityPair>) =
        SimpleMatrix(pairs.map { doubleArrayOf(it.position.toDouble(), it.velocity.toDouble()) }.toTypedArray()).transpose()

    // TODO TEST
    fun matrixToPostionVelocityPair(matrix: SimpleMatrix) : List<PositionVelocityPair> {
        val data = matrix.ddrm.data.toList()
        val pos0PosVel = PositionVelocityPair(data[0].toInt(), data[3].toInt())
        val pos1PosVel = PositionVelocityPair(data[1].toInt(), data[4].toInt())
        val perpPosVel = PositionVelocityPair(data[2].toInt(), data[5].toInt())
        return listOf(pos0PosVel, pos1PosVel, perpPosVel)
    }

    companion object {
        const val leftEncoderName = "leftEncoder" // Must be port 0 and 3
        const val rightEncoderName = MecanumDrive.rightRearName // Must be port 0 and 3
        const val perpendicularEncoderName = "frontEncoder" // port 1 or 2

        @JvmField
        var X_MULTIPLIER = 1.0 // TODO multipliers for each individual encoder
        @JvmField
        var Y_MULTIPLIER = 1.0
        @JvmField
        var LATERAL_DISTANCE = 12.0 // in; distance between the left and right wheels
        @JvmField
        var FORWARD_OFFSET = 12.5 / 2.0 // in; offset of the lateral wheel

        @JvmField
        var PAR0_Y_TICKS = -9664.594625447902
        @JvmField
        var PAR1_Y_TICKS = 12265.310512671498
        @JvmField
        var PERP_X_TICKS = 12705.066008931835
    }

}