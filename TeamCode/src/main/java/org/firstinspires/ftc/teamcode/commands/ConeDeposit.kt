package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.util.Timing.Timer
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.Passthrough

/**
 * Reverses intake to spit out cone and stops after predetermined amount of time.
 * @author Jared Haertel
 */
class ConeDeposit(private val intake: Intake) : CommandBase() {

    private val timeToSpitOut = 500 // milliseconds
    private val timer: ElapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        timer.reset()
        intake.spitOut()
    }

    override fun isFinished(): Boolean {
        return !intake.isConeInside() && timer.time() >= timeToSpitOut
    }

    override fun end(interrupted: Boolean) {
        intake.off()
    }
}