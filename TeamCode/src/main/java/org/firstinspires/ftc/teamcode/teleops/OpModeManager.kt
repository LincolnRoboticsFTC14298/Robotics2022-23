package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.teleops.tuning.drive.*
import java.util.*

object OpModeManager {
    @JvmField
    val DRIVE_CLASS: Class<*> = MecanumDrive::class.java
    const val GROUP = "quickstart"
    const val DISABLED = false

    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (DISABLED) return
        val rrOpModes = arrayOf<Class<out OpMode?>>(
            AngularRampLogger::class.java,
            ForwardPushTest::class.java,
            ForwardRampLogger::class.java,
            LateralPushTest::class.java,
            ManualFeedbackTuner::class.java,
            ManualFeedforwardTuner::class.java,
            SplineTest::class.java,
            MecanumMotorDirectionDebugger::class.java
        )
        //val liftOpModes =
        for (o in arrayOf(*rrOpModes)) {
            manager.register(
                OpModeMeta.Builder()
                    .setName(o.simpleName)
                    .setGroup(GROUP)
                    .setFlavor(OpModeMeta.Flavor.TELEOP)
                    .build(), o
            )
        }
    }
}