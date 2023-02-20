package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.ApproachCone
import org.firstinspires.ftc.teamcode.commands.ApproachConeFromAngle
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.drive.PoseStorage
import org.firstinspires.ftc.teamcode.drive.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.*

@TeleOp
@Disabled
class ConeAutoTest : CommandOpMode() {

    private lateinit var vision: Vision
    private lateinit var lift: Lift
    private lateinit var passthrough: Passthrough
    private lateinit var localizer: OdometryLocalizer

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val passthrough = Passthrough(hardwareMap)
        vision = Vision(hardwareMap)
        val localizer = OdometryLocalizer(hardwareMap)
        localizer.poseEstimate = Pose2d(0.0, 0.0, Math.toRadians(90.0))

        val mecanum = Mecanum(hardwareMap, localizer, vision)

        register(lift, claw, passthrough, mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Vector2d(driver1.leftY, -driver1.leftX) }
        val rotation = { -driver1.rightX }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }

        mecanum.defaultCommand = JoystickDrive(mecanum, input, rotation, fieldCentricProvider) //obstacleAvoidanceProvider)

        val coneInput = { Vector2d(-driver1.leftY, driver1.leftX) }
        driver1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                ApproachCone(mecanum, vision, lift, passthrough, claw, coneInput)
            )

        driver1.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(
                ApproachConeFromAngle(mecanum, vision, claw, coneInput)
            )

    }

    override fun run() {
        super.run()
        vision.fetchTelemetry(telemetry, localizer.poseEstimate)
        telemetry.addData("Lift position", lift.getRelativePosition())
        telemetry.addData("Passthrough position", lift.getRelativePosition() + passthrough.getRelativePosition())
        telemetry.update()
    }

}