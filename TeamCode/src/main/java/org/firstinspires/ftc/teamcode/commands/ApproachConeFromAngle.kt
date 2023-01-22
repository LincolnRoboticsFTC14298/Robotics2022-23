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
        val result = null //vision.getNearestCone()
        if (result != null) {
            addCommands(
                InstantCommand(vision::startStreamingFrontCamera),
                SequentialCommandGroup(
                    ParallelDeadlineGroup(
                        // Waiting for intake to get a cone
                        WaitUntilCommand { claw.isConeInside() },
                        // Driving
                        SequentialCommandGroup(
                            // Drive normally until a cone has been detected
                            ParallelDeadlineGroup(
                                WaitUntilCommand { false },//vision.getNearestCone() != null },
                                mecanum.defaultCommand
                            ),
                            // Switch to auto approach the cone once a cone has been detected
                            WaitUntilCommand{ claw.isConeInside() },
                            ApproachAngle(
                                mecanum,
                                {0.0},//vision::getNearestCone,
                                speed
                            )
                        ),
                    ),
                    ClawPickUp(claw)
                ),
                InstantCommand(vision::stopStreamingFrontCamera)
            )
        }
        addRequirements(mecanum, vision, claw)
    }
}