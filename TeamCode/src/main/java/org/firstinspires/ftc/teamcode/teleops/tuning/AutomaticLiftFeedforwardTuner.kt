package org.firstinspires.ftc.teamcode.teleops.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.internal.system.Misc
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.subsystems.VoltageSensor
import org.firstinspires.ftc.teamcode.util.RegressionUtil
import kotlin.math.sqrt


@TeleOp
@Disabled
class AutomaticLiftFeedforwardTuner : LinearOpMode() {

    var height = 80.0
    var maxPower = 0.5

    lateinit var lift: Lift
    override fun runOpMode() {
        lift = Lift(hardwareMap, VoltageSensor(hardwareMap))
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val timer = ElapsedTime()

        waitForStart()

        telemetry.addLine(Misc.formatInvariant("Manually extend the lift %.2f cm.", height))
        telemetry.addLine("To find gravity term, press a.")
        telemetry.update()

        while(!isStopRequested) {
            if (gamepad1.a) {
                break
            }
        }

        var gravityAcceleration = 0.0
        while(!isStopRequested) {
            lift.updateFilter(null)
            lift.checkEncoder()
            lift.fetchTelemetry(telemetry)
            telemetry.update()

            if (lift.getRawExtensionLength() < 0.1) {
                gravityAcceleration = -lift.getAcceleration()
            }
        }

        telemetry.addData("Downward acceleration = %.5f", gravityAcceleration)
        telemetry.addLine("Keep the lift lowered. Press a when ready to tune feedforward.")
        telemetry.update()

        while(!isStopRequested) {
            if (gamepad1.a) {
                break;
            }
        }

        val maxVel = 0.0 // TODO
        val finalVel = maxPower * maxVel
        val accel = (finalVel*finalVel)/(2.0 * height)
        val rampTime = sqrt(2.0 * height / accel)

        val timeSamples = mutableListOf<Double>()
        val powerSamples = mutableListOf<Double>()
        val velocitySamples = mutableListOf<Double>()
        val accelerationSamples = mutableListOf<Double>()

        timer.reset()
        while(!isStopRequested) {
            if (timer.time() > rampTime) {
                lift.setPower(0.0)
                break
            }

            val vel = accel*timer.time()
            val power = vel / maxVel
            lift.setPower(power)

            lift.updateFilter(null)

            timeSamples.add(timer.time())
            powerSamples.add(power)
            velocitySamples.add(lift.getVelocity())
            accelerationSamples.add(lift.getAcceleration())

            lift.fetchTelemetry(telemetry)
            telemetry.update()
        }

        telemetry.addLine("Lower the lift. Press a when ready to continue.")
        telemetry.update()

        while(!isStopRequested) {
            if (gamepad1.a) {
                break;
            }
        }

        val powerTime = height / maxVel

        timer.reset()
        while(!isStopRequested) {
            if (timer.time() > powerTime) {
                break
            }

            lift.setPower(maxPower)

            lift.updateFilter(null)

            timeSamples.add(timer.time())
            powerSamples.add(maxPower)
            velocitySamples.add(lift.getVelocity())
            accelerationSamples.add(lift.getAcceleration())

            lift.fetchTelemetry(telemetry)
            telemetry.update()
        }

        val results = RegressionUtil.fitStateData(timeSamples, velocitySamples, accelerationSamples, powerSamples, null)

        val kStatic = results.kStatic - results.kA*gravityAcceleration

        lift.setPower(results.kA * gravityAcceleration)

        telemetry.addLine("The lift should be able to hold its position now")
        telemetry.addLine(
            Misc.formatInvariant(
                "kV = %.5f, kA = %.5f, kStatic = %.5f (R^2 = %.2f)",
                results.kV, results.kA, kStatic, results.rSquare
            )
        )
        telemetry.update()

    }
}