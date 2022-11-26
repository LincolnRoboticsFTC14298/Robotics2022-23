package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.commands.drive.ApproachRelativePoint
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.localization.Localizer

/**
 * Uses localizer to align with largest pole "in sight" for depositing the cone.
 * Also deposits.
 * TODO: Should it deposit and reset? What if the odometry error is too high?
 */
class ApproachPole(
    mecanum: Mecanum,
    localizer: Localizer,
    lift: Lift,
    passthrough: Passthrough,
    intake: Intake,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        val nearestPole = localizer.getFacingPole()
        if (nearestPole != null) {
            addCommands(
                ParallelCommandGroup(
                    // TODO: Maybe activate lift/passthrough based on the distance to the pole
                    ReadyPoleDeposit(nearestPole.type, lift, passthrough),
                    ApproachRelativePoint(
                        mecanum,
                        nearestPole::vector,
                        RobotConfig.intakePosition,
                        speed
                    )
                ),
                WaitUntilCommand { lift.atTarget() && passthrough.atTargetAngle() },
                IntakeDeposit(intake),
                ReadyPickUp(lift, passthrough)
            )
        }
        addRequirements(mecanum, localizer, lift, passthrough, intake)
    }

}