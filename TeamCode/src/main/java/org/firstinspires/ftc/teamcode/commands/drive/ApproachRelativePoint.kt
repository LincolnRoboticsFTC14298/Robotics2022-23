package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.purepursuit.PurePursuitUtil.angleWrap
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * Approaches a relative point while accepting an input controlling speed or aggressiveness.
 * Driver cannot control rotation, only how fast it moves towards the point.
 * @param targetPoint                   Target point in robot tangent space.
 * @param offsetPose                    Offset vector in robot tangent space. This offset vector
 *                                      will approach the target instead.
 * @param speed                         Controls how fast the robot drives forward towards the point.
 * @param maxTolerableDistance          Maximum distance it can be from the target in m.
 * @param maxTolerableAngleDifference   Maximum angle it can face away from the target in radians.
 * @param timeout                       Timeout after that many seconds if no target provided.
 */
class ApproachRelativePoint(
    private val mecanum: Mecanum,
    private val targetPoint: () -> Vector2d?,
    private val offsetPose: Pose2d = Pose2d(0.0, 0.0, 0.0),
    private val speed: () -> Double,
    private val maxTolerableDistance: Double = 0.005, // m
    private val maxTolerableAngleDifference: Double = 0.05, // radians
    private val timeout: Double = 2.0
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    private val controller = PIDFController(RobotConfig.trackingCoeffs)

    private var distanceFromTarget: Double = Double.MAX_VALUE

    private var target: Vector2d? = null

    private val timeoutTimer = ElapsedTime()

    override fun initialize() {
        controller.setInputBounds(-Math.PI, Math.PI) // Input and target must be an angle in [-pi, pi]
        controller.targetPosition = offsetPose.heading
    }

    override fun execute() {
        val possibleTarget = targetPoint.invoke()
        if (possibleTarget != null) {
            target = possibleTarget // Saves target in case the targetPoint goes offline
            timeoutTimer.reset() // Resets timeout timer because the target is fresh
        }

        if (target != null) {
            // Calculate error taking into account the offsetPose space
            val errorVec = target!!.minus(offsetPose.headingVec())
            distanceFromTarget = errorVec.norm()

            // TODO: Dynamic "motion profiler"???
            // Turning
            val turn = controller.update(errorVec.angle())

            // Translating
            val v = mecanum.getVelocityEstimate()!!.vec().norm()
            val distTillDeaccel = v * v / (2 * DriveConstants.MAX_ACCEL) // Distance required to go from current velocity to zero
            val maxPower = min(distanceFromTarget / distTillDeaccel, 1.0) // Max allowable power taking into account need for de-acceleration
            val forward = errorVec.times(speed.invoke() * maxPower / distanceFromTarget)

            val power = Pose2d(forward, turn)
            mecanum.setWeightedDrivePower(power)
        }
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget <= maxTolerableDistance
                && abs(controller.lastError) <= maxTolerableAngleDifference
                || timeoutTimer.seconds() > timeout
    }

}
