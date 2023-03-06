package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.Twist2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Input in robot (tangent) frame
 */
class PseudoMotionProfiledDrive(
    private val mecanum: MecanumDrive,
    private val input: () -> Twist2d,
    private val isInputVelocityNormalized: Boolean,
    private val isInputRotationNormalized: Boolean,
    private val distanceFromTarget: () -> Double = { Double.MAX_VALUE },
    private val maxTolerableDistance: Double = 1.0, // in
) : CommandBase() {

    val timer = ElapsedTime()

    init {
        timer.reset()
        addRequirements(mecanum)
    }

    override fun execute() {
        val currentVel = mecanum.robotVelRobot.transVel.norm()

        var desiredInput = input.invoke()
        if (isInputVelocityNormalized) desiredInput = Twist2d(desiredInput.transVel * MecanumDrive.MAX_WHEEL_VEL, desiredInput.rotVel)
        if (isInputRotationNormalized) desiredInput = Twist2d(desiredInput.transVel, desiredInput.rotVel * MecanumDrive.MAX_ANG_VEL)

        // Get the time from the last loop
        val dt = timer.seconds()

        // Calculate the maximum velocity it can drive before needing to stop
        val maxVelToStop = sqrt(2 * MecanumDrive.MIN_PROFILE_ACCEL * distanceFromTarget.invoke()) // TODO update to account for tangential driving

        // Calculate the maximum and minimum velocity that the drivetrain could possibly travel
        val maxVelFromLast = currentVel + MecanumDrive.MAX_PROFILE_ACCEL * dt
        val minVelFromLast = currentVel - MecanumDrive.MIN_PROFILE_ACCEL * dt

        // Choose the minimum possible speed and obtain the max of the physically possible velocity
        val velocity = max(minVelFromLast, minOf(maxVelFromLast, desiredInput.transVel.norm(), maxVelToStop))

        val maxRotFromLast = mecanum.robotVelRobot.rotVel + MecanumDrive.MAX_ANG_ACCEL * dt
        val minRotFromLast = mecanum.robotVelRobot.rotVel - MecanumDrive.MAX_ANG_ACCEL * dt

        val rotation = max(minRotFromLast, min(maxRotFromLast, desiredInput.rotVel)) // May cause issues w/ [ApproachRelativePoint] and [ApproachAngle]

        mecanum.setDriveSignal(Twist2d(desiredInput.transVel.normalize() * velocity, rotation))
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return distanceFromTarget.invoke() <= maxTolerableDistance
    }

}
