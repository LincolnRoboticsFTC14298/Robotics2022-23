package org.firstinspires.ftc.teamcode.teleops.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.RobotConfig.driveLeftFront
import org.firstinspires.ftc.teamcode.RobotConfig.driveLeftRear
import org.firstinspires.ftc.teamcode.RobotConfig.driveRightFront
import org.firstinspires.ftc.teamcode.RobotConfig.driveRightRear
import kotlin.math.abs

@TeleOp
class CooperativeDrive : LinearOpMode() {

    override fun runOpMode() {

        //motors
        val motorFrontLeft: DcMotor = hardwareMap.dcMotor.get(driveLeftFront)
        val motorBackLeft: DcMotor = hardwareMap.dcMotor.get(driveLeftRear)
        val motorFrontRight: DcMotor = hardwareMap.dcMotor.get(driveRightFront)
        val motorBackRight: DcMotor = hardwareMap.dcMotor.get(driveRightRear)

        motorFrontRight.direction = DcMotorSimple.Direction.REVERSE
        motorBackLeft.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        if (isStopRequested()) return

        while (opModeIsActive()) {

            val y: Double = -gamepad1.left_stick_y.toDouble()
            val x: Double = gamepad2.left_stick_x.toDouble()
            val rx: Double = gamepad2.right_stick_x.toDouble()

            val denominator = (abs(y) + abs(x) + abs(rx)).coerceAtLeast(1.0)

            val frontLeftPower = (y + x + rx) / denominator
            val backLeftPower = (y - x + rx) / denominator
            val frontRightPower = (y - x - rx) / denominator
            val backRightPower = (y + x - rx) / denominator

            motorFrontLeft.power = frontLeftPower
            motorBackLeft.power = backLeftPower
            motorFrontRight.power = frontRightPower
            motorBackRight.power = backRightPower
        }
    }
}