package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.poleDepositAnticipationTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Readies cone deposit at specified height.
 * The [Passthrough] only rotates once the lift is a certain time
 * away from the target height to prevent hitting the pole.
 * TODO: Reconsider this commands purpose
 * @author Jared Haertel
 */
class ReadyPoleDeposit(
    pole: RobotConfig.PoleType,
    lift: Lift, passthrough: Passthrough
) : SequentialCommandGroup() {

    init{
        addCommands(
            // Set height to the right pole
            InstantCommand({ lift.setHeight(pole) }, lift),
            // Ensures passthrough and lift finished simultaneously by
            // starting passthrough deposit at time from finish
            WaitUntilCommand { lift.timeFromTarget() <= poleDepositAnticipationTime },
            // Rotate passthrough to deposit position
            InstantCommand(passthrough::deposit, passthrough),
            // Wait until lift and passthrough are done
            WaitUntilCommand { lift.atTarget() && passthrough.atTarget() }
        )
        addRequirements(lift, passthrough)
    }

}