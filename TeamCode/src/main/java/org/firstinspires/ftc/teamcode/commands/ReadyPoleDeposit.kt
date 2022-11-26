package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Readies cone deposit at specified height.
 * The [Passthrough] only rotates once the lift is a certain time
 * away from the target height to prevent hitting the pole.
 * @author Jared Haertel
 */
class ReadyPoleDeposit(pole: RobotConfig.PoleType, lift: Lift, passthrough: Passthrough) : SequentialCommandGroup() {


    init{
        addCommands(
            InstantCommand({ lift.setSetpoint(pole) }, lift),
            WaitUntilCommand { lift.timeFromTarget() <= RobotConfig.poleDepositAnticipationTime }, // Ensures passthrough and lift finished simultaneously
            PassthroughDeposit(passthrough),
        )
        addRequirements(lift, passthrough)
    }

}