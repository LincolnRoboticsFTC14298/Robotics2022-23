package org.firstinspires.ftc.teamcode.teleops.tuning.drive

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.FieldConfig.tileSize
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.util.Action.ActionOpMode
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister
import org.firstinspires.ftc.teamcode.util.Action.ParallelDeadlineAction
import java.lang.Math.toRadians


// TODO: Make like the other tuners
class LateralMultiplierTuning : ActionOpMode() {

    override fun runOpMode() {
        if (OpModeRegister.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive = MecanumDrive(hardwareMap, startingPose, OdometryLocalizer(hardwareMap),VoltageSensor(hardwareMap))
            waitForStart()
            while (opModeIsActive()) {
                val endPose = startingPose.times(Pose2d(0.0, DISTANCE, 0.0))

                runBlocking(ParallelDeadlineAction(
                    drive.actionBuilder(drive.pose)
                        .lineToY(endPose.trans.y)
                        .lineToY(startingPose.trans.y)
                        .build(),
                    object : Action {
                        override fun run(telemetryPacket: TelemetryPacket): Boolean {
                            drive.voltageSensor.periodic()
                            telemetryPacket.put("voltage", drive.voltageSensor.voltage)
                            return true
                        }

                        override fun preview(canvas: Canvas) {}
                    }
                ))
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