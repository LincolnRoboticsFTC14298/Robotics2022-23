package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.roadrunner.Twist2d
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.util.PIDCoefficients
import org.firstinspires.ftc.teamcode.util.PIDFController
import kotlin.math.abs


class ApproachAngle(
    private val mecanum: MecanumDrive,
    private val targetAngle: () -> Double?,
    private val input: () -> Twist2d,
    private val maxTolerableAngleDifference: Double = 0.05 // radians
) : SequentialCommandGroup() {

    init {
        addRequirements(mecanum)
    }

    private var lastTargetAngleSeen: Double? = null

    private val controller = PIDFController(PIDCoefficients(MecanumDrive.HEADING_GAIN, 0.0, MecanumDrive.HEADING_VEL_GAIN))

    init {
        // Automatically handles overflow
        controller.setInputBounds(-Math.PI, Math.PI)

        val localInput = {
            lastTargetAngleSeen = targetAngle.invoke() ?: lastTargetAngleSeen
            // Set desired angular velocity to the heading controller output
            val rotation = controller.update(lastTargetAngleSeen!!)

            Twist2d(input.invoke().transVel, rotation)
        }

        addCommands(
            ParallelDeadlineGroup(
                WaitUntilCommand { targetAngle.invoke() != null },
                mecanum.defaultCommand
            ),
            PseudoMotionProfiledDrive(
                mecanum,
                localInput,
                isInputVelocityNormalized = true,
                isInputRotationNormalized = false
            )
        )

        addRequirements(mecanum)
    }

    override fun isFinished(): Boolean {
        return super.isFinished() && abs(controller.lastError) <= maxTolerableAngleDifference
    }

}