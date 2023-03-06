package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor

@TeleOp
@Disabled
class VisionTest : CommandOpMode() {

    private lateinit var vision: Vision
    private lateinit var mecanum: MecanumDrive

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        vision = Vision(hardwareMap)
        val localizer = OdometryLocalizer(hardwareMap)
        mecanum = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, Math.toRadians(90.0)), localizer, VoltageSensor(hardwareMap))

        register(mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Twist2d(Vector2d(driver1.leftY, -driver1.leftX), -driver1.rightX) }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }

        mecanum.defaultCommand = JoystickDrive(mecanum, input, fieldCentricProvider) //obstacleAvoidanceProvider)
    }

    override fun run() {
        super.run()
        vision.fetchTelemetry(telemetry, mecanum.pose)
        telemetry.update()
    }

}