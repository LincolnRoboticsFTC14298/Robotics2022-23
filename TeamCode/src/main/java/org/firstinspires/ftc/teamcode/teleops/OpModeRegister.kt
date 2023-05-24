package org.firstinspires.ftc.teamcode.teleops

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.button.GamepadButton
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.teleops.testing.*
import org.firstinspires.ftc.teamcode.teleops.tuning.*
import org.firstinspires.ftc.teamcode.teleops.tuning.drive.*

object OpModeRegister {
    @JvmField
    val DRIVE_CLASS: Class<*> = MecanumDrive::class.java
    var currentBatch = BATCH.VISION

    enum class BATCH(vararg val opModes: Class<out OpMode>) {
        ROAD_RUNNER_TUNING(
            MecanumMotorDirectionDebugger::class.java,
            ForwardPushTest::class.java,
            LateralPushTest::class.java,
            SimpleLocalizationTest::class.java,
            ForwardRampLogger::class.java,
            AngularRampLogger::class.java,
            ManualFeedforwardTuner::class.java,
            LateralMultiplierTuning::class.java,
            LateralRampLogger::class.java,
            ManualFeedbackTuner::class.java,
            SplineTest::class.java,
        ),
        VISION(
            SimpleWebcamTest::class.java,
            AprilTagDemo::class.java,
            VisionTest::class.java
        ),
        CLAW(
            ClawTuner::class.java,
            ClawTest::class.java
        ),
        PASSTHROUGH(
            PassthroughTuner::class.java
        ),
        LIFT(
            LiftDistancePerPulseTuner::class.java,
            AutomaticLiftFeedforwardTuner::class.java,
            ManualLiftFeedforwardTuner::class.java,
            LiftManualTest::class.java
        ),
        DRIVE(
            SimpleMecanumTeleOp::class.java,
            JoystickDrivingTest::class.java,
            ConeAutoTest::class.java
        ),
        FINAL_MODES(
            BasicTeleOp::class.java,
            MainTeleOp::class.java,
            SimpleTeleOp::class.java
        ),
        MISCELLANEOUS(
            PhotonTest::class.java,
            SensorTest::class.java
        ),
        SPECIAL_EVENT(
            CooperativeDrive::class.java
        )
    }

    @OpModeRegistrar
    @JvmStatic
    fun register(manager: OpModeManager) {

        val selector = ModeSelector::class.java

        manager.register(
            OpModeMeta.Builder()
                .setName(selector.simpleName)
                .setGroup(currentBatch.name)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build(), selector
        )

        for (o in currentBatch.opModes) {
            manager.register(
                OpModeMeta.Builder()
                    .setName(o.simpleName)
                    .setGroup(currentBatch.name)
                    .setFlavor(OpModeMeta.Flavor.TELEOP)
                    .build(), o
            )
        }
    }

    class ModeSelector() : OpMode() {

        val values = enumValues<BATCH>()
        lateinit var gamepad: GamepadEx

        override fun init() {
            gamepad = GamepadEx(gamepad1)
            telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        }

        override fun loop() {
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                currentBatch = values[(currentBatch.ordinal + 1) % values.size]
                telemetry.addLine("YES")
            } else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                currentBatch = values[(currentBatch.ordinal - 1 + values.size) % values.size]
                telemetry.addLine("YES BUT NO")
            }

            telemetry.addData("Current batch", currentBatch.name)
            telemetry.update()
        }

    }

}