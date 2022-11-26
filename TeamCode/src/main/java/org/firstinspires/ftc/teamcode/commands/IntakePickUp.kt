package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Intake

/**
 * It turns on the intake and is finished when it detects the cone is inside.
 * Assumes passthrough is in correct position.
 * @author Jared Haertel
 */
class IntakePickUp(private val intake: Intake) : CommandBase() {

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        intake.suckIn()
    }

    override fun isFinished(): Boolean {
        return intake.isConeInside()
    }

    override fun end(interrupted: Boolean) {
        intake.off()
    }

}