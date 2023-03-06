package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d

interface GlobalLocalizer {
    var pose: Pose2d
    var robotVelRobot: Twist2d
}