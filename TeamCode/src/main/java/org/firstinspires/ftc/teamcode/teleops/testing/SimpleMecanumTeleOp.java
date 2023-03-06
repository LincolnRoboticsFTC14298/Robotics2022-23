package org.firstinspires.ftc.teamcode.teleops.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp
public class SimpleMecanumTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        //motors
        DcMotor motorFrontLeft  = hardwareMap.dcMotor.get(MecanumDrive.leftFrontName);
        DcMotor motorBackLeft = hardwareMap.dcMotor.get(MecanumDrive.leftRearName);
        DcMotor motorFrontRight = hardwareMap.dcMotor.get(MecanumDrive.rightFrontName);
        DcMotor motorBackRight = hardwareMap.dcMotor.get(MecanumDrive.rightRearName);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

        }
    }
}