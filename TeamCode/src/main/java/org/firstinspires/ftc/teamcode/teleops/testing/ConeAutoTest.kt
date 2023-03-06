package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commands.ApproachCone
import org.firstinspires.ftc.teamcode.commands.ApproachConeFromAngle
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.*

@TeleOp
@Disabled
class ConeAutoTest : CommandOpMode() {

    private lateinit var vision: Vision
    private lateinit var lift: Lift
    private lateinit var passthrough: Passthrough
    private lateinit var mecanum: MecanumDrive

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val voltageSensor = VoltageSensor(hardwareMap)
        lift = Lift(hardwareMap, voltageSensor)
        val claw = Claw(hardwareMap)
        passthrough = Passthrough(hardwareMap)
        vision = Vision(hardwareMap)
        //val localizer = MecanumMonteCarloLocalizer(hardwareMap, vision, Pose2d(), arrayToRowMatrix(doubleArrayOf()))
        val localizer = OdometryLocalizer(hardwareMap)
        mecanum = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, Math.toRadians(90.0)), localizer, voltageSensor)

        register(lift, claw, passthrough, mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Twist2d(Vector2d(driver1.leftY, -driver1.leftX), -driver1.rightX) }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }

        mecanum.defaultCommand = JoystickDrive(mecanum, input, fieldCentricProvider) //obstacleAvoidanceProvider)

        driver1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                ApproachCone(mecanum, vision, lift, passthrough, claw, input)
            )

        driver1.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(
                ApproachConeFromAngle(mecanum, vision, claw, input)
            )

    }

    override fun run() {
        super.run()
        vision.fetchTelemetry(telemetry, mecanum.pose)
        telemetry.addData("Lift position", lift.getRelativePosition())
        telemetry.addData("Passthrough position", lift.getRelativePosition() * passthrough.getRelativePosition())
        telemetry.update()
    }

}