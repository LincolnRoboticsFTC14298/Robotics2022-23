package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.Twist2d
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.commands.drive.ApproachAngle
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Vision

class ApproachPoleFromAngle(
    mecanum: MecanumDrive,
    vision: Vision,
    input: () -> Twist2d
) : SequentialCommandGroup() {

    init {
        addCommands(
            // Drive normally until a cone has been detected
            ApproachAngle(
                mecanum,
                { vision.getClosestPole()?.angle },
                input
            ),
        )
        addRequirements(mecanum, vision)
    }
}