package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import kotlin.math.abs


class ApproachAngle(
    private val mecanum: Mecanum,
    private val targetAngle: () -> Double?,
    private val input: () -> Vector2d,
    private val maxTolerableAngleDifference: Double = 0.05 // radians
) : SequentialCommandGroup() {

    init {
        addRequirements(mecanum)
    }

    private var lastTargetAngleSeen: Double? = null

    private val controller = PIDFController(SampleMecanumDrive.HEADING_PID)
    init {
        // Automatically handles overflow
        controller.setInputBounds(-Math.PI, Math.PI)

        val fieldInput = { input.invoke().rotated(mecanum.getPoseEstimate().heading) }

        val rotation = {
            lastTargetAngleSeen = targetAngle.invoke() ?: lastTargetAngleSeen
            // Set desired angular velocity to the heading controller output
            controller.update(lastTargetAngleSeen!!)
        }

        addCommands(
            ParallelDeadlineGroup(
                WaitUntilCommand { targetAngle.invoke() != null },
                mecanum.defaultCommand
            ),
            PseudoMotionProfiledDrive(
                mecanum,
                fieldInput,
                rotation
            )
        )

        addRequirements(mecanum)
    }

    override fun isFinished(): Boolean {
        return super.isFinished() && abs(controller.lastError) <= maxTolerableAngleDifference
    }

}