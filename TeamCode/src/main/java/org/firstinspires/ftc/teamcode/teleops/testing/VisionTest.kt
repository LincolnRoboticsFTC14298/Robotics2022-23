package org.firstinspires.ftc.teamcode.teleops.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Twist2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.FieldConfig.tileSize
import org.firstinspires.ftc.teamcode.commands.drive.SimpleJoystickDrive
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer

@Config
class VisionTest : CommandOpMode() {

    private lateinit var vision: Vision
    private lateinit var mecanum: MecanumDrive

    private var fieldCentric = true
    private var usePose = false

    override fun initialize() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        vision = Vision(hardwareMap)
        val localizer = OdometryLocalizer(hardwareMap)
        mecanum = MecanumDrive(hardwareMap, startPose, localizer, VoltageSensor(hardwareMap))

        register(mecanum, vision)

        val driver1 = GamepadEx(gamepad1)

        val input = { Twist2d(Vector2d(driver1.leftX, driver1.leftY), -driver1.rightX) }


        val fieldCentricProvider = { fieldCentric }
        driver1.getGamepadButton(GamepadKeys.Button.A).whenPressed( Runnable { fieldCentric = !fieldCentric } )

        driver1.getGamepadButton(GamepadKeys.Button.B).whenPressed( Runnable { usePose = !usePose } )

        mecanum.defaultCommand = SimpleJoystickDrive(mecanum, input, fieldCentricProvider)
    }

    override fun run() {
        super.run()
        val p = TelemetryPacket()
        mecanum.drawRobot(p.fieldOverlay())
        p.put("Field Centric", fieldCentric)
        p.put("Use Pose", usePose)
        vision.fetchTelemetry(p, if (usePose) mecanum.pose else null)
        FtcDashboard.getInstance().sendTelemetryPacket(p)
    }

    companion object {
        val startPose = Pose2d(-1.5 * tileSize, -1.5 * tileSize, Math.toRadians(90.0))
    }

}