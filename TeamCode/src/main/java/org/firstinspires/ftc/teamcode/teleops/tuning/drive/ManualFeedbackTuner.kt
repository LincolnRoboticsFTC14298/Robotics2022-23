package org.firstinspires.ftc.teamcode.teleops.tuning.drive

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.Pose2d
import org.firstinspires.ftc.teamcode.FieldConfig.tileSize
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.teleops.OpModeRegister
import org.firstinspires.ftc.teamcode.util.Action.ActionOpMode
import org.firstinspires.ftc.teamcode.util.Action.ParallelDeadlineAction

@Config
class ManualFeedbackTuner : ActionOpMode() {

    override fun runOpMode() {
        if (OpModeRegister.DRIVE_CLASS == MecanumDrive::class.java) {
            val drive = MecanumDrive(
                hardwareMap,
                startingPose,
                OdometryLocalizer(hardwareMap),
                VoltageSensor(hardwareMap)
            )
            waitForStart()
            while (opModeIsActive()) {
                runBlocking(ParallelDeadlineAction(
                    drive.actionBuilder(drive.pose)
                        .lineToX(startingPose.trans.x + DISTANCE)
                        .lineToX(startingPose.trans.x)
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
        var DISTANCE = 64.0
        var startingPose = Pose2d(-2.0 * tileSize, 2.0 * tileSize, 0.0)
    }
}