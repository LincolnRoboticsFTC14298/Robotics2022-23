package org.firstinspires.ftc.teamcode.drive.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.filters.particleFilter.ParticleFilter
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.poseToMatrix

// TODO: Multiple instances of localizer will be created please fix oh god.
class MecanumMonteCarloLocalizer(hwMap: HardwareMap, private val vision: Vision, startingPose: Pose2d, startingStandardDeviation: SimpleMatrix) : Localizer {

    private val odometryLocalizer = OdometryLocalizer(hwMap)
    private val visionMeasurementModel = VisionMeasurementModel(RobotConfig.CameraData.LOGITECH_C920)

    private val particleFilter = ParticleFilter(50, odometryLocalizer, SimpleMatrix(arrayOf(doubleArrayOf(0.01, 0.01, 0.01))), visionMeasurementModel)

    private val timer = ElapsedTime()

    override var poseEstimate: Pose2d = Pose2d()

    override var poseVelocity: Pose2d? = Pose2d()
        get() = odometryLocalizer.poseVelocity

    init {
        particleFilter.initialize(poseToMatrix(startingPose), startingStandardDeviation)
    }

    override fun update() {
        val dt = timer.seconds()

        odometryLocalizer.update()

        val wheelDeltas = SimpleMatrix(arrayOf(odometryLocalizer.wheelDeltas.toDoubleArray()))
        particleFilter.predict(wheelDeltas, dt)

        // TODO Move to pole pipeline
        val landmarks = vision.getLandmarkInfo()
        val landmarkArray = Array(landmarks.size) { index -> doubleArrayOf(landmarks[index].point.x, landmarks[index].point.y, landmarks[index].angle, landmarks[index].distance) }

        particleFilter.update(SimpleMatrix(landmarkArray).transpose())

        val state = particleFilter.stateEstimate.ddrm.data
        poseEstimate = Pose2d(state[0], state[1], state[2])
        timer.reset()
    }

}
