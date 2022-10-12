package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.audio.AudioAnalyzer;

@TeleOp
public class AudioTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        AudioAnalyzer analyzer = new AudioAnalyzer();
        analyzer.start();

        DcMotor[] motors = new DcMotor[4];
        motors[0] = hardwareMap.dcMotor.get("motorFrontLeft");
        motors[1] = hardwareMap.dcMotor.get("motorBackLeft");
        motors[2] = hardwareMap.dcMotor.get("motorFrontRight");
        motors[3] = hardwareMap.dcMotor.get("motorBackRight");

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested())
        {
            analyzer.stop();
            return;
        }

        while (opModeIsActive())
        {
            for (DcMotor i: motors)
            {
                i.setPower(1.0 * ((analyzer.getAmplitude() > 10)? 1 : 0));
                telemetry.addData("Volume", analyzer.getVolume());
                if (analyzer.getVolume() > 0)
                {
                    telemetry.addData("Microphone input", "WORKING");
                }
                telemetry.update();
            }
        }

        analyzer.stop();
    }
}
