package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.commands.drive.ApproachPoint
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.drive.Mecanum

/**
 * Uses localizer to align with pole most "in sight" of the robot for depositing
 * the cone while accepting driver input. It then deposits the cone and
 * resets. If there is no pole detected the command does nothing.
 * TODO: Reconsider purpose
 * @author Jared Haertel
 */
class ApproachPoleAndDeposit(
    mecanum: Mecanum,
    lift: Lift,
    passthrough: Passthrough,
    intake: Intake,
    speed: () -> Double
) : SequentialCommandGroup() {

    init {
        val nearestPole = mecanum.getFacingPole()
        if (nearestPole != null) {
            addCommands(
                ParallelCommandGroup(
                    // Start the lift and extend passthrough once appropriate
                    // TODO: Maybe activate lift/passthrough based on the distance to the pole
                    ReadyPoleDeposit(nearestPole.type, lift, passthrough),
                    // Approach pole
                    ApproachPoint(
                        mecanum,
                        nearestPole::vector,
                        RobotConfig.intakePosition, // TODO: Correct
                        speed
                    )
                ),
                // Once lift and passthrough are done and at the pole, deposit
                IntakeDeposit(intake)
            )
        }
        addRequirements(mecanum, lift, passthrough, intake)
    }

}