package org.firstinspires.ftc.teamcode.teleops.tuning.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer

class SimpleLocalizationTest : OpMode() {

    lateinit var voltageSensor: VoltageSensor
    lateinit var mecanumDrive: MecanumDrive

    override fun init() {
        voltageSensor = VoltageSensor(hardwareMap)
        mecanumDrive = MecanumDrive(hardwareMap, startingPose, OdometryLocalizer(hardwareMap), voltageSensor)

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    }

    override fun loop() {
        voltageSensor.periodic()
        mecanumDrive.periodic()

        val pose = mecanumDrive.pose

        val p = TelemetryPacket()
        p.put("x", pose.trans.x)
        p.put("y", pose.trans.y)
        p.put("heading (deg)", Math.toDegrees(pose.rot.log()))

        mecanumDrive.drawRobot(p.fieldOverlay())

        FtcDashboard.getInstance().sendTelemetryPacket(p)
        telemetry.update()
    }

    @Config
    companion object {
        @JvmStatic
        var startingPose = Pose2d(0.0, 0.0, 0.0)
    }

}