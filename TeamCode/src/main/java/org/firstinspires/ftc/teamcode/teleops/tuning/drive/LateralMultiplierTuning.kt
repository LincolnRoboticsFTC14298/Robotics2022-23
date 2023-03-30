package org.firstinspires.ftc.teamcode.teleops.tuning.drive

import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.FieldConfig.tileSize
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.teleops.ActionOpMode
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister
import java.lang.Math.toRadians


// TODO: Make like the other tuners
class LateralMultiplierTuning : ActionOpMode() {

    override fun runOpMode() {
        if (OpModeRegister.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive = MecanumDrive(hardwareMap, startingPose, OdometryLocalizer(hardwareMap),VoltageSensor(hardwareMap))
            waitForStart()
            while (opModeIsActive()) {
                drive.voltageSensor.periodic()
                drive.periodic()

                val endPose = startingPose.times(Pose2d(0.0, DISTANCE, 0.0))

                runBlocking(
                    drive.actionBuilder(drive.pose)
                        .lineToY(endPose.trans.y)
                        .lineToY(startingPose.trans.y)
                        .build()
                )
            }
        } else {
            throw AssertionError()
        }
    }

    companion object {
        @JvmStatic
        var DISTANCE = 64.0

        @JvmStatic
        var startingPose = Pose2d(2.0 * tileSize, 2.0 * tileSize,  toRadians(-90.0))
    }
}