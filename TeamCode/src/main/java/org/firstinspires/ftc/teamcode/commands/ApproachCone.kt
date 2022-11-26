package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.Vision

/**
 * Use vision to align robot and precisely pick up the cone with input from driver.
 * Uses vision because environment is much more dynamic.
 * TODO: Test!! What happens when the cone tips? When it is lifted? When it moves? What if its too close? How will it react?
 * TODO: this may not change if you change the default command, probably not a problem
 */
class ApproachCone(
    mecanum: Mecanum,
    vision: Vision,
    intake: Intake,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        addCommands(
            InstantCommand(vision::startStreamingFrontCamera),
            ParallelDeadlineGroup(
                // Waiting for intake to get a cone
                IntakePickUp(intake),
                // Driving
                SequentialCommandGroup(
                    // Drive normally until a cone has been detected
                    ParallelDeadlineGroup(
                        WaitUntilCommand { vision.getConeRelativePosition() != null },
                        mecanum.defaultCommand
                    ),
                    // Switch to auto approach the cone once the cone has been detected
                    ApproachRelativePoint(
                        mecanum,
                        vision::getConeRelativePosition,
                        RobotConfig.intakePosition,
                        speed
                    )
                )
            ),
            InstantCommand(vision::stopStreamingFrontCamera)
        )
        addRequirements(mecanum, vision, intake)
    }

}