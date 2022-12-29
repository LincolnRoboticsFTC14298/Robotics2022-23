package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.PathFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.path.Path
import com.acmerobotics.roadrunner.util.Angle
import com.acmerobotics.roadrunner.util.GuidingVectorField
import com.acmerobotics.roadrunner.util.NanoClock
import kotlin.math.atan2
import kotlin.math.sqrt
import com.acmerobotics.roadrunner.followers.GVFFollower

/**
 * State-of-the-art path follower based on the [GuidingVectorField].
 *
 * @param maxVel maximum velocity
 * @param maxAccel maximum acceleration
 * @param admissibleError admissible/satisfactory pose error at the end of each move
 * @param kN normal vector weight (see [GuidingVectorField])
 * @param kOmega proportional heading gain
 * @param errorMapFunc error map function (see [GuidingVectorField])
 * @param clock clock
 *
 *  @see [GVFFollower]
 */
class HolonomicGVFFollower @JvmOverloads constructor(
    private val maxVel: Double,
    private val maxAccel: Double,
    admissibleError: Pose2d,
    private val kN: Double,
    private val kOmega: Double,
    private val errorMapFunc: (Double) -> Double = { it },
    clock: NanoClock = NanoClock.system()
) : PathFollower(admissibleError, clock) {

    private lateinit var gvf: GuidingVectorField
    private var lastUpdateTimestamp: Double = 0.0
    private var lastSpeed: Double = 0.0
    private var lastProjDisplacement: Double = 0.0

    override var lastError: Pose2d = Pose2d()

    override fun followPath(path: Path) {
        gvf = GuidingVectorField(path, kN, errorMapFunc)
        lastUpdateTimestamp = clock.seconds()
        lastSpeed = 0.0
        lastProjDisplacement = 0.0
        super.followPath(path)
    }

    override fun internalUpdate(currentPose: Pose2d): DriveSignal {
        val gvfResult = gvf.getExtended(currentPose.x, currentPose.y, lastProjDisplacement)

        val desiredDirection = gvfResult.vector.rotated(currentPose.heading) // Tangent space
        val desiredHeading = atan2(gvfResult.vector.y, gvfResult.vector.x)
        val headingError = Angle.normDelta(desiredHeading - currentPose.heading)

        val omega = kOmega * headingError

        // basic online motion profiling
        val timestamp = clock.seconds()
        val dt = timestamp - lastUpdateTimestamp
        val remainingDistance = currentPose.vec() distTo path.end().vec()
        val maxSpeedToStop = sqrt(2 * maxAccel * remainingDistance)
        val maxSpeedFromLast = lastSpeed + maxAccel * dt
        val speed = minOf(maxSpeedToStop, maxSpeedFromLast, maxVel)

        lastUpdateTimestamp = timestamp
        lastSpeed = speed
        lastProjDisplacement = gvfResult.displacement

        val targetPose = path[gvfResult.displacement]

        lastError = Kinematics.calculateRobotPoseError(targetPose, currentPose)

        return DriveSignal(Pose2d(desiredDirection*speed, omega))
    }
}