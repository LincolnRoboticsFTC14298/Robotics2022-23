package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.TriggerReader
import com.outoftheboxrobotics.photoncore.PhotonCore
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.drive.localization.MecanumMonteCarloLocalizer
import org.firstinspires.ftc.teamcode.drive.localization.OdometryLocalizer
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.util.arrayToRowMatrix
import java.time.Instant

@TeleOp
class BasicTeleOp : CommandOpMode() {

    override fun initialize() {
        PhotonCore.enable()

        /****************************************************
         * Initialize hardware                              *
         ****************************************************/

        val lift = Lift(hardwareMap)
        val claw = Claw(hardwareMap)
        val passthrough = Passthrough(hardwareMap)
        val vision = Vision(hardwareMap)
        //val localizer = MecanumMonteCarloLocalizer(hardwareMap, vision, Pose2d(), arrayToRowMatrix(doubleArrayOf()))
        val mecanum = Mecanum(hardwareMap)

        register(lift, claw, passthrough, mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val forward = { driver1.leftY }
        // Negate left x because left is positive
        val strafe = { -driver1.leftX }
        // Negate right x because ccw is positive
        val rotation = { -driver1.rightX }

        var fieldCentric = false

        mecanum.defaultCommand = JoystickDrive(
            mecanum,
            forward,
            strafe,
            rotation,
            { fieldCentric }
        )

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
                    ReadyPoleDeposit(RobotConfig.PoleType.LOW, lift, passthrough),
                    ApproachPoleFromAngle(mecanum, vision, forward)
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPoleDeposit(RobotConfig.PoleType.MEDIUM, lift, passthrough),
                    ApproachPoleFromAngle(mecanum, vision, forward)
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPoleDeposit(RobotConfig.PoleType.HIGH, lift, passthrough),
                    ApproachPoleFromAngle(mecanum, vision, forward)
                )
            )

        driver1
            .getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                InstantCommand(claw::partiallyOpen, claw)
            )

        // Reset to pickup and keep claw open until a cone comes in
        driver1
            .getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(
                SequentialCommandGroup(
                    ReadyPickUp(lift, claw, passthrough),
                    WaitUntilCommand(claw::isConeInside),
                    InstantCommand(claw::close, claw),
                )
            )

        // Manually close claw
        driver1
            .getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(
                InstantCommand(claw::close, claw)
            )


        // Press trigger to deposit cone
        Trigger(TriggerReader(driver1, GamepadKeys.Trigger.RIGHT_TRIGGER)::isDown)
            .whenActive(InstantCommand(claw::partiallyOpen, claw))

//        driver1
//            .getGamepadButton(GamepadKeys.Button.X)
//            .cancelWhenActive(semiAutoDepositLow)
//            .cancelWhenActive(semiAutoDepositMedium)
//            .cancelWhenActive(semiAutoDepositHigh)
//            .cancelWhenActive(approachCone)
    }

    override fun run() {
        super.run()
        telemetry.update()
    }
}

