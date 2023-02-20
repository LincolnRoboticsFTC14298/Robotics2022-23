package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.drive.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.Vision

@TeleOp
@Disabled
class VisionTest : CommandOpMode() {

    private lateinit var vision: Vision
    private lateinit var localizer: OdometryLocalizer

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        vision = Vision(hardwareMap)
        val localizer = OdometryLocalizer(hardwareMap)
        localizer.poseEstimate = Pose2d(0.0, 0.0, Math.toRadians(90.0))

        val mecanum = Mecanum(hardwareMap, localizer, vision)

        register(mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Vector2d(driver1.leftY, -driver1.leftX) }
        val rotation = { -driver1.rightX }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }

        mecanum.defaultCommand = JoystickDrive(mecanum, input, rotation, fieldCentricProvider) //obstacleAvoidanceProvider)
    }

    override fun run() {
        super.run()
        vision.fetchTelemetry(telemetry, localizer.poseEstimate)
        telemetry.update()
    }

}