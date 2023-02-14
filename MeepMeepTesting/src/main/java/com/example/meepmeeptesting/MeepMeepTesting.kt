package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim


fun main() {
    System.setProperty("sun.java2d.opengl", "true")

    val tileSize = 23.5
    val width = 14.0
    val height = 14.0

    val startPose = Pose2d(-1.5 * tileSize, -3.0 * tileSize + 7, Math.toRadians(90.0))
    val firstDeposit = pointToPose(Vector2d(-1.5 * tileSize, -0.9 * tileSize), Vector2d(-tileSize, 0.0))
    val pickUp = Pose2d(-3.0 * tileSize + height/2.0 + 4.0, -0.5 * tileSize, Math.toRadians(0.0))
    val subsequentDeposit = pointToPose(Vector2d(-1.8 * tileSize, -0.5 * tileSize), Vector2d(-tileSize, 0.0))


    val meepMeep = MeepMeep(800)
    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(30.0, 30.0, Math.toRadians(180.0), Math.toRadians(180.0), 12.0)
            .setDimensions(width, height)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(startPose)
                    .forward(1.2*tileSize-7)
                    .lineToSplineHeading(firstDeposit)
                    .setTangent(Math.toRadians(90.0))
                    .waitSeconds(0.25) // Deposit
                    .splineToSplineHeading(pickUp + Pose2d(x = 0.7*tileSize), Math.toRadians(-180.0))
                    .lineTo(pickUp.vec())
                    .waitSeconds(0.25) // Pick Up
                    .forward(0.4*tileSize)
                    .lineToSplineHeading(subsequentDeposit)
                    .waitSeconds(0.25) // Deposit
                    .lineToSplineHeading(pickUp + Pose2d(x = 0.4*tileSize))
                    .lineTo(pickUp.vec())
                    .waitSeconds(0.25) // Pick Up
                    .forward(0.4*tileSize)
                    .lineToSplineHeading(subsequentDeposit)
                    .waitSeconds(0.25) // Deposit
                    .lineToSplineHeading(pickUp + Pose2d(x = 0.4*tileSize))
                    .lineTo(pickUp.vec())
                    .waitSeconds(0.25) // Pick Up
                    .forward(0.4*tileSize)
                    .lineToSplineHeading(subsequentDeposit)
                    .waitSeconds(0.25) // Deposit
                    .lineToSplineHeading(pickUp + Pose2d(x = 0.4*tileSize))
                    .lineTo(pickUp.vec())
                    .waitSeconds(0.25) // Pick Up
                    .forward(0.4*tileSize)
                    .lineToSplineHeading(subsequentDeposit)
                    .waitSeconds(0.25) // Deposit
                    .lineToSplineHeading(pickUp + Pose2d(x = 0.4*tileSize))
                    .lineTo(pickUp.vec())
                    .waitSeconds(0.25) // Pick Up
                    .forward(0.4*tileSize)
                    .lineToSplineHeading(subsequentDeposit)
                    .waitSeconds(0.25) // Deposit
                    .build()
            }
    meepMeep.setBackground(Background.FIELD_POWERPLAY_KAI_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}

fun pointToPose(vector: Vector2d, target: Vector2d): Pose2d {
    val angle = (target-vector).angle()
    return Pose2d(vector, angle)
}