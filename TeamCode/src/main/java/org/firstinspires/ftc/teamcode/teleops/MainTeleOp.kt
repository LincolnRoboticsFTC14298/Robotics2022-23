package org.firstinspires.ftc.teamcode.teleops

import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.gamepad.TriggerReader
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.RobotConfig.teleOpDepositAdj
import org.firstinspires.ftc.teamcode.RobotConfig.teleOpSetPointAdj
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.ApproachCone
import org.firstinspires.ftc.teamcode.commands.ApproachPoleAndDeposit
import org.firstinspires.ftc.teamcode.commands.drive.modes.AssistedDrive
import org.firstinspires.ftc.teamcode.commands.drive.modes.ManualDrive
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.drive.Mecanum
import org.firstinspires.ftc.teamcode.subsystems.drive.localization.MecanumMonteCarloLocalizer


class MainTeleOp : CommandOpMode() {

    override fun initialize() {

        /****************************************************
         * Initialize hardware                              *
         ****************************************************/

        val lift = Lift(hardwareMap)
        val intake = Intake(hardwareMap)
        val passthrough = Passthrough(hardwareMap)
        val mecanum = Mecanum(hardwareMap)
        val vision = Vision(hardwareMap)
        val localizer = MecanumMonteCarloLocalizer(hardwareMap, vision)


        register(lift, intake, passthrough, mecanum, vision)

        /****************************************************
         * Driver 1 Controls                                *
         * Driving and semi autonomous control              *
         ****************************************************/

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val forward  = { driver1.leftY }
        // Negate left x because left is positive
        val strafe   = { -driver1.leftX }
        // Negate right x because ccw is positive
        val rotation = { -driver1.rightX }

        var fieldCentric = false
        val fieldCentricProvider = { fieldCentric }
        val assistedDrive = AssistedDrive(mecanum, localizer, forward, strafe, rotation, fieldCentricProvider)
        val manualDrive   = ManualDrive(mecanum, localizer, forward, strafe, rotation, fieldCentricProvider)

        mecanum.defaultCommand = manualDrive // TODO: Check if this should be default

        // Press DPAD_UP to turn off field centrism
        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenActive(Runnable{ fieldCentric = false })
        // Press DPAD_DOWN to turn on field centrism
        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenActive(Runnable{ fieldCentric = true })
        // Press DPAD_LEFT to activate assisted driving
        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(Runnable{ mecanum.defaultCommand = assistedDrive })
        // Press DPAD_RIGHT to activate manual driving
        driver1
            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(Runnable{ mecanum.defaultCommand = manualDrive   })

        /**
         * Lift
         */
        // Deposit and then retract lift and passthrough
        val semiAutoDeposit = SequentialCommandGroup(
            ApproachPoleAndDeposit(mecanum, lift, passthrough, intake, forward),
            ReadyPickUp(lift, passthrough)
        )

        // Press A to semi auto deposit the lift
        driver1
            .getGamepadButton(GamepadKeys.Button.A)
            .whenActive(semiAutoDeposit)
        // Press B to cancel auto deposit and reset just in case
        // TODO: is resetting desired?
        driver1
            .getGamepadButton(GamepadKeys.Button.B)
            .cancelWhenActive(semiAutoDeposit)
            .whenActive(ReadyPickUp(lift, passthrough))

        /**
         * Intake
         */
        val approachCone = ApproachCone(mecanum, vision, intake, forward)
        // Press X to approach cone
        driver1
            .getGamepadButton(GamepadKeys.Button.X)
            .whenActive(approachCone)
        // Press Y to cancel cone approach
        driver1
            .getGamepadButton(GamepadKeys.Button.Y)
            .cancelWhenActive(approachCone)

        /****************************************************
         * Driver 2 Controls                                *
         * Manual control and adjustments                   *
         ****************************************************/

        val driver2 = GamepadEx(gamepad2)

        /**
         * Drive
         */
        // Press DPAD_DOWN to retract lift
        driver2
            .getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(InstantCommand(lift::retract, lift))
        // Press DPAD_LEFT to go to low pole and extend passthrough
        driver2
            .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(ReadyPoleDeposit(RobotConfig.PoleType.LOW, lift, passthrough))
        // Press DPAD_RIGHT to go to medium pole and extend passthrough
        driver2
            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(ReadyPoleDeposit(RobotConfig.PoleType.MEDIUM, lift, passthrough))
        // Press DPAD_UP to go to high pole and extend passthrough
        driver2
            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(ReadyPoleDeposit(RobotConfig.PoleType.HIGH, lift, passthrough))

        // Adjust lift height
        // TODO: this doesn't remember changes to height
        // Press LEFT_BUMPER to set lower height setpoint
        driver2
            .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(InstantCommand({ lift.setpoint -= teleOpSetPointAdj }, lift))
        // Press RIGHT_BUMPER to set lower height setpoint
        driver2
            .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(InstantCommand({ lift.setpoint += teleOpSetPointAdj }, lift))


        /**
         * Passthrough
         */
        // Press A to manually set passthrough to deposit
        driver2
            .getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(InstantCommand(passthrough::deposit, passthrough))
        driver2
            .getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(InstantCommand(passthrough::deposit, passthrough))

        // Adjust passthrough deposit location
        Trigger(TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER)::isDown)
            .whenActive(Runnable{ passthrough.depositOffset -= teleOpDepositAdj })
        Trigger(TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER)::isDown)
            .whenActive(Runnable{ passthrough.depositOffset += teleOpDepositAdj })


        /**
         * Intake
         */
        driver2
            .getGamepadButton(GamepadKeys.Button.X)
            .toggleWhenActive(IntakePickUp(intake))
        driver2
            .getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(IntakeDeposit(intake))

        /****************************************************
         * Commands                                         *
         ****************************************************/

        // TODO: Give current pose estimate stuff
        // TODO: Implement
        // TODO: telemetry

        /****************************************************
         * Telemetry                                        *
         ****************************************************/
        // Uses callbacks to update properly
        telemetry.speak("Good luck and have fun")
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML) // TODO DO
        telemetry.addData("Drive mode", "Mode: %s | Field Centric: %s", mecanum::getDefaultCommand.name, fieldCentricProvider)
        telemetry.addData("Lift setpoint", lift::setpoint)
        telemetry.addData("", 0)
        telemetry.update()

    }

    override fun run() {
        super.run()
        telemetry.update()
    }

}