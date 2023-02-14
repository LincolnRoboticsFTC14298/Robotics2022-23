package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.sqrt


// TODO add distance
class ApproachAngle(
    private val mecanum: Mecanum,
    private val targetAngle: () -> Double?,
    private val speed: () -> Double,
    private val timeout: Double = 2.0
) : CommandBase() {

    init {
        addRequirements(mecanum)
    }

    private val controller = PIDFController(SampleMecanumDrive.HEADING_PID)

    private var target: Double? = null

    private val timeoutTimer = ElapsedTime()

    private val loopTimer = ElapsedTime()

    override fun initialize() {
        // Automatically handles overflow
        controller.setInputBounds(-Math.PI, Math.PI)
    }

    override fun execute() {
        val possibleTarget = targetAngle.invoke()
        if (possibleTarget != null) {
            // Save target in case the targetPoint goes offline
            target = possibleTarget
            // Reset timeout timer because the target is fresh
            timeoutTimer.reset()
        }

        if (target != null) {
            // Turning //
            // github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java
            // Set desired angular velocity to the heading controller output
            // TODO CHECK
            val omega = controller.update(target!!) * DriveConstants.kV * DriveConstants.TRACK_WIDTH

            val dt = loopTimer.seconds()
            val currentVel = mecanum.getVelocityEstimate()!!.vec().norm()
            val maxVelFromLast = currentVel + DriveConstants.MAX_ACCEL * dt
            val minVelFromLast = currentVel - DriveConstants.MAX_ACCEL * dt
            // Choose the minimum possible speed and obtain the max of the physically possible velocity
            val velocity = max(minVelFromLast, minOf(maxVelFromLast, speed.invoke() * DriveConstants.MAX_VEL))

            mecanum.drive.setDriveSignal(DriveSignal(Pose2d(velocity, 0.0, omega)))
        }

        loopTimer.reset()
    }

    override fun isFinished(): Boolean {
        return timeoutTimer.seconds() > timeout
    }

}