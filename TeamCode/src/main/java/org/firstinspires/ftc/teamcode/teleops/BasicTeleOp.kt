package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.*

@TeleOp
class BasicTeleOp : CommandOpMode() {

    override fun initialize() {
        PhotonCore.enable()

        /****************************************************
         * Initialize hardware                              *
         ****************************************************/

        val voltageSensor = VoltageSensor(hardwareMap)
        val lift = Lift(hardwareMap, voltageSensor)
        val claw = Claw(hardwareMap)
        val passthrough = Passthrough(hardwareMap)
        val vision = Vision(hardwareMap)
        //val localizer = MecanumMonteCarloLocalizer(hardwareMap, vision, Pose2d(), arrayToRowMatrix(doubleArrayOf()))
        val localizer = OdometryLocalizer(hardwareMap)
        val mecanum = MecanumDrive(hardwareMap, Pose2d(0.0, 0.0, Math.toRadians(90.0)), localizer, voltageSensor)

        register(lift, claw, passthrough, mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Twist2d(Vector2d(driver1.leftY, -driver1.leftX), -driver1.rightX) }

        var fieldCentric = false

        mecanum.defaultCommand = JoystickDrive(
            mecanum,
            input,
            { fieldCentric }
        )

        val approachPoleCommand = InstantCommand({})//ApproachPoleFromAngle(mecanum, vision, input)


        // Deposit settings
        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(
                InstantCommand(passthrough::junctionDeposit, passthrough)
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPoleDeposit(FieldConfig.PoleType.LOW, lift, passthrough),
                    approachPoleCommand
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPoleDeposit(FieldConfig.PoleType.MEDIUM, lift, passthrough),
                    approachPoleCommand
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPoleDeposit(FieldConfig.PoleType.HIGH, lift, passthrough),
                    approachPoleCommand
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPickUp(lift, claw, passthrough),
                    WaitUntilCommand(claw::isConeInside),
                    InstantCommand(claw::close, claw)
                )

            )

        // Reset to pickup and keep claw open until a cone comes in
//        driver1
//            .getGamepadButton(GamepadKeys.Button.A)
//            .whenPressed(
//                //ApproachConeFromAngle(mecanum, vision, claw, input)
//            )

        // Manually close claw
        driver1
            .getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand(claw::close, claw)
            )


        // Press trigger to deposit cone
        Trigger(TriggerReader(driver1, GamepadKeys.Trigger.RIGHT_TRIGGER)::isDown)
            .whenActive(InstantCommand(claw::partiallyOpen, claw))

        driver1
            .getGamepadButton(GamepadKeys.Button.X)
            .cancelWhenActive(approachPoleCommand)
    }

    override fun run() {
        super.run()
        telemetry.update()
    }
}

