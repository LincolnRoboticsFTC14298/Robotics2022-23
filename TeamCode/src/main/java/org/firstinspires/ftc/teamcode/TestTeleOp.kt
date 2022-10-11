package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

import org.firstinspires.ftc.teamcode.audio.AudioAnalyzer

@TeleOp
class TestTeleOp: OpMode()
{
    private lateinit var analyzer: AudioAnalyzer
    private lateinit var motors: Array<DcMotorSimple>

    override fun init()
    {
        analyzer = AudioAnalyzer()
        analyzer.start()

        motors = arrayOf(hardwareMap.dcMotor["motorFrontLeft"], hardwareMap.dcMotor["motorBackLeft"], hardwareMap.dcMotor["motorFrontRight"], hardwareMap.dcMotor["motorBackRight"])

        motors[2].direction = DcMotorSimple.Direction.REVERSE
        motors[3].direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop()
    {
        for (i in motors.iterator())
            i.power = 1.0 * (analyzer.getAmplitude() > 10).compareTo(false)
        telemetry.addData("Volume", analyzer.volume)
        if (analyzer.volume > 0)
        {
            telemetry.addData("Success", "This is working")
        }
        telemetry.update()
    }

    override fun stop()
    {
        analyzer.stop()
    }
}