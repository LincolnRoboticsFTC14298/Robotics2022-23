package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.*
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.util.*
import java.util.*
import kotlin.math.ceil

@Config
class MecanumDrive(hardwareMap: HardwareMap, var pose: Pose2d, val localizer: Localizer, val voltageSensor: VoltageSensor) : SubsystemBase() {
    private val kinematics = MecanumKinematics(
        IN_PER_TICK * TRACK_WIDTH_TICKS,
        IN_PER_TICK / LATERAL_IN_PER_TICK
    )
    private val feedforward = MotorFeedforward(kS, kV, kA)
    private val defaultTurnConstraints = TurnConstraints(
        MAX_ANG_VEL, -MAX_ANG_ACCEL, MAX_ANG_ACCEL
    )
    private val defaultVelConstraint: VelConstraint = MinVelConstraint(
        Arrays.asList(
            kinematics.WheelVelConstraint(MAX_WHEEL_VEL),
            AngularVelConstraint(MAX_ANG_VEL)
        )
    )
    private val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL)

    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    val imu: IMU

    var robotVelRobot: Twist2d = Twist2d(Vector2d(0.0, 0.0), 0.0)

    val inPerTick = IN_PER_TICK
    private val poseHistory = LinkedList<Pose2d>()

    inner class DriveLocalizer : Localizer {
        val leftFront: Encoder
        val leftRear: Encoder
        val rightRear: Encoder
        val rightFront: Encoder

        private var lastLeftFrontPos: Int
        private var lastLeftRearPos: Int
        private var lastRightRearPos: Int
        private var lastRightFrontPos: Int

        init {
            leftFront = RawEncoder(this@MecanumDrive.leftFront)
            leftRear = RawEncoder(leftBack)
            rightRear = RawEncoder(rightBack)
            rightFront = RawEncoder(this@MecanumDrive.rightFront)

            lastLeftFrontPos = leftFront.positionAndVelocity.position
            lastLeftRearPos = leftRear.positionAndVelocity.position
            lastRightRearPos = rightRear.positionAndVelocity.position
            lastRightFrontPos = rightFront.positionAndVelocity.position
        }

        override fun updateAndGetIncr(): Twist2dIncrDual<Time> {
            val leftFrontPosVel = leftFront.positionAndVelocity
            val leftRearPosVel = leftRear.positionAndVelocity
            val rightRearPosVel = rightRear.positionAndVelocity
            val rightFrontPosVel = rightFront.positionAndVelocity

            val incrs: MecanumKinematics.WheelIncrements<Time> = MecanumKinematics.WheelIncrements(
                DualNum<Time>(
                    listOf(
                        (leftFrontPosVel.position - lastLeftFrontPos).toDouble(),
                        leftFrontPosVel.velocity.toDouble()
                    )
                ) * inPerTick,
                DualNum<Time>(
                    listOf(
                        (leftRearPosVel.position - lastLeftRearPos).toDouble(),
                        leftRearPosVel.velocity.toDouble()
                    )
                ) * inPerTick,
                DualNum<Time>(
                    listOf(
                        (rightRearPosVel.position - lastRightRearPos).toDouble(),
                        rightRearPosVel.velocity.toDouble()
                    )
                ) * inPerTick,
                DualNum<Time>(
                    listOf(
                        (rightFrontPosVel.position - lastRightFrontPos).toDouble(),
                        rightFrontPosVel.velocity.toDouble()
                    )
                ) * inPerTick
            )
            lastLeftFrontPos = leftFrontPosVel.position
            lastLeftRearPos = leftRearPosVel.position
            lastRightRearPos = rightRearPosVel.position
            lastRightFrontPos = rightFrontPosVel.position
            return kinematics.forward(incrs)
        }
    }

    init {
        LynxFirmwareVersion.throwIfAnyModulesBelowVersion(
            hardwareMap,
            LynxFirmwareVersion(1, 8, 2)
        )

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        leftFront = hardwareMap.get(DcMotorEx::class.java, leftFrontName)
        leftBack = hardwareMap.get(DcMotorEx::class.java, leftRearName)
        rightBack = hardwareMap.get(DcMotorEx::class.java, rightRearName)
        rightFront = hardwareMap.get(DcMotorEx::class.java, rightFrontName)

        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        imu = hardwareMap.get(IMU::class.java, "imu")
        val parameters = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        )

        imu.initialize(parameters)

    }

    override fun periodic() {
        val incr = localizer.updateAndGetIncr()
        pose = pose.plus(incr.value())
        poseHistory.add(pose)
        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }
        LogFiles.recordPose(pose)
        robotVelRobot = incr.velocity().value()
    }

    fun setDriveSignal(vels: Twist2d) {
        val wheelVels: MecanumKinematics.WheelVelocities<Time> =
            kinematics.inverse(Twist2dDual.constant(vels, 1))

        val voltage = voltageSensor.voltage
        leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
        leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
        rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
        rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage
    }

    fun setDrivePowers(powers: Twist2d) {
        val wheelVels: MecanumKinematics.WheelVelocities<Time> =
            kinematics.inverse(Twist2dDual.constant(powers, 1))
        leftFront.power = wheelVels.leftFront[0]
        leftBack.power = wheelVels.leftBack[0]
        rightBack.power = wheelVels.rightBack[0]
        rightFront.power = wheelVels.rightFront[0]
    }

    /**
     * TODO: TEST MULTIPLE ALGORITHMS
     *  Maybe use the velocity vector as well.
     * @return Gets the pole the robot is facing by minimizing the difference of heading.
     */
    fun getFacingPole(): FieldConfig.Pole {
        val headVec = pose.rot.vec()
        val posVec = pose.trans

        /*
        val minX = floor(posVec.x / RobotConfig.tileSize)
        val minY = floor(posVec.y / RobotConfig.tileSize)

        var highestCosT = -1.0
        var optimalPole: RobotConfig.Pole? = null

        for (x in listOf(minX * RobotConfig.tileSize, (minX+1) * RobotConfig.tileSize)) {
            for (y in listOf(minY * RobotConfig.tileSize, (minY+1) * RobotConfig.tileSize)) {
                val corner = Vector2d(x, y)
                val pole = RobotConfig.Pole.getPole(corner)
                if (pole != null) {
                    val diff = corner.minus(posVec)
                    val dist = diff.norm()
                    val distSignedCosT = diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result

                    // Highest dot product means the angle between the corner and heading is lowest
                    if (distSignedCosT > highestCosT) {
                        highestCosT = distSignedCosT
                        optimalPole = pole
                    }
                }
            }
        }
        */

        return enumValues<FieldConfig.Pole>().maxBy {
            val diff = it.vector.minus(posVec)
            val dist = diff.norm()
            diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result
        }
    }

    fun getClosestPoleOfType(poleType: FieldConfig.PoleType): FieldConfig.Pole {
        val poles = FieldConfig.Pole.getPolesOfType(poleType)

        val headVec = pose.rot.vec()
        val posVec = pose.trans

        return poles.maxBy {
            val diff = it.vector.minus(posVec)
            val dist = diff.norm()
            diff.dot(headVec) / (dist * dist) // divide by dist twice so that further values have smaller result
        }
    }


    inner class FollowTrajectoryAction(private val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0
        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0,
                timeTrajectory.path.length(),
                ceil(timeTrajectory.path.length() / 2).toInt()
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val (trans) = timeTrajectory.path[disps[i], 1].value()
                xPoints[i] = trans.x
                yPoints[i] = trans.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0
                return false
            }

            val txWorldTarget = timeTrajectory[t]
            val command = HolonomicController(
                AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                .compute(txWorldTarget, pose, robotVelRobot)

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            LogFiles.recordTargetPose(txWorldTarget.value())

            p.put("x", pose.trans.x)
            p.put("y", pose.trans.y)
            p.put("heading (deg)", Math.toDegrees(pose.rot.log()))

            val (trans, rot) = txWorldTarget.value().minusExp(pose)
            p.put("xError", trans.x)
            p.put("yError", trans.y)
            p.put("headingError (deg)", Math.toDegrees(rot.log()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#4CAF507A")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0
                return false
            }

            val txWorldTarget = turn[t]
            val command = HolonomicController(
                AXIAL_GAIN, LATERAL_GAIN, HEADING_GAIN,
                AXIAL_VEL_GAIN, LATERAL_VEL_GAIN, HEADING_VEL_GAIN
            )
                .compute(txWorldTarget, pose, robotVelRobot)

            val wheelVels: MecanumKinematics.WheelVelocities<Time> = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            leftFront.power = feedforward.compute(wheelVels.leftFront) / voltage
            leftBack.power = feedforward.compute(wheelVels.leftBack) / voltage
            rightBack.power = feedforward.compute(wheelVels.rightBack) / voltage
            rightFront.power = feedforward.compute(wheelVels.rightFront) / voltage

            LogFiles.recordTargetPose(txWorldTarget.value())
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            drawRobot(c, pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.trans.x, turn.beginPose.trans.y, 2.0)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#7C4DFF7A")
            c.fillCircle(turn.beginPose.trans.x, turn.beginPose.trans.y, 2.0)
        }
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            beginPose,
            1e-6,
            defaultTurnConstraints,
            defaultVelConstraint,
            defaultAccelConstraint,
            0.25
        )
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)
        var i = 0
        for ((trans) in poseHistory) {
            xPoints[i] = trans.x
            yPoints[i] = trans.y
            i++
        }
        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    private fun drawRobot(c: Canvas, t: Pose2d) {
        c.setStrokeWidth(1)
        c.strokeRect(t.trans.x, t.trans.y, TRACK_WIDTH, TRACK_WIDTH)
        val halfv = t.rot.vec().times(0.25 * TRACK_WIDTH)
        val p1 = t.trans.plus(halfv)
        val (x, y) = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, x, y)
    }

    companion object {
        const val leftFrontName: String = "leftFront"
        const val leftRearName: String = "leftRear"
        const val rightRearName: String = "rightRear"
        const val rightFrontName: String = "rightFront"

        const val TICKS_PER_REV = 537.6
        const val MAX_RPM = 312.0
        var WHEEL_RADIUS = 1.9685 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (motor) speed
        var TRACK_WIDTH = 12.5 // in

        // drive model parameters
        @JvmField
        var IN_PER_TICK = 0.0
        @JvmField
        var LATERAL_IN_PER_TICK = 1.0
        @JvmField
        var TRACK_WIDTH_TICKS = 0.0

        // feedforward parameters
        @JvmField
        var kS = 0.0
        @JvmField
        var kV = 1.0 / rpmToVelocity(MAX_RPM)
        @JvmField
        var kA = 0.0

        // path profile parameters
        @JvmField
        var MAX_WHEEL_VEL = MAX_RPM / 60.0 * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI * 0.85
        @JvmField
        var MIN_PROFILE_ACCEL = -30.0
        @JvmField
        var MAX_PROFILE_ACCEL = MAX_WHEEL_VEL

        // turn profile parameters
        @JvmField
        var MAX_ANG_VEL = MAX_WHEEL_VEL / TRACK_WIDTH // shared with path
        @JvmField
        var MAX_ANG_ACCEL = MAX_ANG_VEL

        // path controller gains
        @JvmField
        var AXIAL_GAIN = 0.0
        @JvmField
        var LATERAL_GAIN = 0.0
        @JvmField
        var HEADING_GAIN = 0.0 // shared with turn
        @JvmField
        var AXIAL_VEL_GAIN = 0.0
        @JvmField
        var LATERAL_VEL_GAIN = 0.0
        @JvmField
        var HEADING_VEL_GAIN = 0.0 // shared with turn

        private fun rpmToVelocity(rpm: Double) =
            rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0

    }
}