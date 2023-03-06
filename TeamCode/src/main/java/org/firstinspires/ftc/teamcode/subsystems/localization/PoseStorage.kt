package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.Pose2d

object PoseStorage {
    @JvmStatic
    var currentPose: Pose2d = Pose2d(0.0, 0.0, 0.0)
}