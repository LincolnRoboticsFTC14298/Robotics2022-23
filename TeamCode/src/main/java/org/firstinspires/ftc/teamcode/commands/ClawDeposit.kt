package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.Claw

/**
 * Time based outake command that spits out the cone and stops after a certain amount of time.
 * @author Jared Haertel
 */
class ClawDeposit(private val claw: Claw) : CommandBase() {

    init {
        addRequirements(claw)
    }

    override fun initialize() {
        claw.partiallyOpen()
    }

    override fun isFinished(): Boolean {
        return claw.atTarget()
    }

}