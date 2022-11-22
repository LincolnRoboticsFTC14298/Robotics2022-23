package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Deposits cone at specified height from [ReadyPickUp] state.
 * The [Passthrough] only rotates once the lift is a certain time from the target
 * away from the target height to prevent hitting the pole.
 * TODO: May want to check logic to minimize lift vibration/strain
 * @author Jared Haertel
 */
class PoleDeposit(height: LiftHeight.Height, lift: Lift, intake: Intake, passthrough: Passthrough) : ParallelCommandGroup() {

    private val startDepositTime: Double = 2.0

    init{
        val liftCommand = LiftHeight(height, lift)
        addCommands(
            liftCommand,
            SequentialCommandGroup(
                WaitUntilCommand { liftCommand.timeFromFinished() <= startDepositTime }, // Ensures passthrough and lift finished simultaneously
                PassthroughDeposit(passthrough),
                ConeDeposit(intake)
            ),
        )
        addRequirements(lift, intake, passthrough)
    }

}