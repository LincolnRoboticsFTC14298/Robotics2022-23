package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.outoftheboxrobotics.photoncore.PhotonCore
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.drive.JoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.PoseStorage
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer


class MainTeleOp : CommandOpMode() {

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

        /****************************************************
         * Driver 1 Controls                                *
         * Driving and semi autonomous control              *
         ****************************************************/

        val driver1 = GamepadEx(gamepad1)

        /**
         * Drive
         */
        val input = { Twist2d(Vector2d(driver1.leftY, -driver1.leftX), -driver1.rightX) }

        var fieldCentric = true
        val fieldCentricProvider = { fieldCentric }
//        var obstacleAvoidance = true
//        val obstacleAvoidanceProvider = { obstacleAvoidance }

        mecanum.defaultCommand = JoystickDrive(mecanum, input, fieldCentricProvider) //obstacleAvoidanceProvider)

        /**
         * Lift
         */
//        // Deposit and then retract lift and passthrough
//        val semiAutoDepositLow = SequentialCommandGroup(
//                ReadyPoleDeposit(RobotConfig.PoleType.LOW, lift, passthrough),
//                ApproachPoleFromAngle(mecanum, vision, forward)
//            )
//
//        val semiAutoDepositMedium = SequentialCommandGroup(
//            ReadyPoleDeposit(RobotConfig.PoleType.MEDIUM, lift, passthrough),
//            ApproachPoleFromAngle(mecanum, vision, forward)
//        )
//
//        val semiAutoDepositHigh = SequentialCommandGroup(
//            ReadyPoleDeposit(RobotConfig.PoleType.HIGH, lift, passthrough),
//            ApproachPoleFromAngle(mecanum, vision, forward)
//        )

//        driver1
//            .getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//            .whenActive(semiAutoDepositLow)
//
//        driver1
//            .getGamepadButton(GamepadKeys.Button.DPAD_UP)
//            .whenActive(semiAutoDepositMedium)
//
//        driver1
//            .getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//            .whenActive(semiAutoDepositHigh)

//        driver1
//            .getGamepadButton(GamepadKeys.Button.Y)
//            .whenActive(SequentialCommandGroup(InstantCommand::))


        /**
         * Claw
         */
//        val approachCone = ApproachCone(mecanum, vision, lift, passthrough, claw, forward)
//        // Press X to approach cone
//        driver1
//            .getGamepadButton(GamepadKeys.Button.A)
//            .whenPressed(approachCone)
//
//        driver1
//            .getGamepadButton(GamepadKeys.Button.X)
//            .cancelWhenActive(semiAutoDepositLow)
//            .cancelWhenActive(semiAutoDepositMedium)
//            .cancelWhenActive(semiAutoDepositHigh)
//            .cancelWhenActive(approachCone)

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
        // Press LEFT_BUMPER to set lower height setpoint TODO FIX
//        driver2
//            .getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//            .whenPressed(InstantCommand({ lift.setpoint -= teleOpSetPointAdj }, lift))
//        // Press RIGHT_BUMPER to set lower height setpoint
//        driver2
//            .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//            .whenPressed(InstantCommand({ lift.setpoint += teleOpSetPointAdj }, lift))


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

//        // Adjust passthrough deposit location
//        Trigger(TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER)::isDown)
//            .whenActive(Runnable{ passthrough.depositOffset -= teleOpDepositAdj })
//        Trigger(TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER)::isDown)
//            .whenActive(Runnable{ passthrough.depositOffset += teleOpDepositAdj })


        /**
         * Claw
         */
        driver2
            .getGamepadButton(GamepadKeys.Button.X)
            .toggleWhenActive(InstantCommand(claw::close, claw))
        driver2
            .getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(InstantCommand(claw::open, claw))

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
        //telemetry.addData("Lift setpoint", lift::setpoint) // TODO FIX
        telemetry.addData("", 0)
    }

    override fun run() {
        super.run()
        telemetry.update()
    }

}