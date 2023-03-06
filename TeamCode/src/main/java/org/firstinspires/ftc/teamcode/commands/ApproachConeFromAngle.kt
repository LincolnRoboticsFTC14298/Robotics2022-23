package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Vision

class ApproachConeFromAngle(
    mecanum: MecanumDrive,
    vision: Vision,
    claw: Claw,
    input: () -> Twist2d
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingRearCamera),
            ParallelDeadlineGroup(
                // Waiting for intake to get a cone
                WaitUntilCommand { claw.isConeInside() },
                // Driving
                ApproachAngle(
                    mecanum,
                    { vision.getClosestCone(mecanum.pose)?.angle },
                    { Twist2d(-input.invoke().transVel, input.invoke().rotVel) }
                )
            ),
            ClawPickUp(claw),
            InstantCommand(vision::stopStreamingRearCamera)
        )
        addRequirements(mecanum, vision, claw)
    }
}