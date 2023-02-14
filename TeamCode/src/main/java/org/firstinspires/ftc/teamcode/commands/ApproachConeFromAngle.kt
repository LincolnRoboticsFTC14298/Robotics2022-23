package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.Claw
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.Vision

class ApproachConeFromAngle(
    mecanum: Mecanum,
    vision: Vision,
    claw: Claw,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingRearCamera),
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    // Waiting for intake to get a cone
                    WaitUntilCommand { claw.isConeInside() },
                    // Driving
                    SequentialCommandGroup(
                        // Drive normally until a cone has been detected
                        ParallelDeadlineGroup(
                            WaitUntilCommand { vision.getConeAngle() != null },
                            mecanum.defaultCommand
                        ),
                        // Switch to auto approach the cone once a cone has been detected
                        WaitUntilCommand{ claw.isConeInside() },
                        ApproachAngle(
                            mecanum,
                            vision::getConeAngle,
                            speed
                        )
                    ),
                ),
                ClawPickUp(claw)
            ),
            InstantCommand(vision::stopStreamingRearCamera)
        )
        addRequirements(mecanum, vision, claw)
    }
}