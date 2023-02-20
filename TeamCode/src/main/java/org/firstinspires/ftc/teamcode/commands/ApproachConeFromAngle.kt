package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.Vision

class ApproachConeFromAngle(
    mecanum: Mecanum,
    vision: Vision,
    claw: Claw,
    input: () -> Vector2d
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
                    { vision.getClosestConeAngle(mecanum.getPoseEstimate()) },
                    input
                )
            ),
            ClawPickUp(claw),
            InstantCommand(vision::stopStreamingRearCamera)
        )
        addRequirements(mecanum, vision, claw)
    }
}