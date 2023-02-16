package org.firstinspires.ftc.teamcode.teleops.testing;

import static org.firstinspires.ftc.teamcode.RobotConfig.driveLeftFront;
import static org.firstinspires.ftc.teamcode.RobotConfig.driveLeftRear;
import static org.firstinspires.ftc.teamcode.RobotConfig.driveRightFront;
import static org.firstinspires.ftc.teamcode.RobotConfig.driveRightRear;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Passthrough;

@TeleOp
public class SimpleMecanumTeleOp extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        //motors
        DcMotor motorFrontLeft  = hardwareMap.dcMotor.get(driveLeftFront);
        DcMotor motorBackLeft = hardwareMap.dcMotor.get(driveLeftRear);
        DcMotor motorFrontRight = hardwareMap.dcMotor.get(driveRightFront);
        DcMotor motorBackRight = hardwareMap.dcMotor.get(driveRightRear);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw claw = new Claw(hardwareMap);
        Passthrough passthrough = new Passthrough(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.a) {
                claw.close();
            }

            if (gamepad1.b) {
                claw.open();
            }

            if (gamepad1.x) {
                passthrough.pickUp();
            }

            if (gamepad1.y) {
                passthrough.junctionDeposit();
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            claw.periodic();
            passthrough.periodic();
            claw.fetchTelemetry(telemetry);
            passthrough.fetchTelemetry(telemetry);
            telemetry.update();
        }
    }
}