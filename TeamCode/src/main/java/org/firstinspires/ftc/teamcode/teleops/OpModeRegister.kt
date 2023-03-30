package org.firstinspires.ftc.teamcode.teleops

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
    private val currentBatch = BATCH.ROAD_RUNNER_TUNING

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
}