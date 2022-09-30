package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.audio.AudioAnalyzer

@TeleOp
class TestTeleOp: LinearOpMode()
{
    override fun runOpMode()
    {
        val analyzer = AudioAnalyzer()
        analyzer.start()

        val motors = arrayOf(hardwareMap.dcMotor["motorFrontLeft"], hardwareMap.dcMotor["motorBackLeft"], hardwareMap.dcMotor["motorFrontRight"], hardwareMap.dcMotor["motorBackRight"])

        motors[2].direction = DcMotorSimple.Direction.REVERSE
        motors[3].direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        if (isStopRequested)
        {
            analyzer.stop()
            return
        }

        while (opModeIsActive())
        {
            for (i in motors.iterator())
                i.power = 1.0 * (analyzer.getAmplitude() > 10).compareTo(false)
        }
    }
}