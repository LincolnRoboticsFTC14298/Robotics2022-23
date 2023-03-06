package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.*
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.teamcode.FieldConfig
import org.firstinspires.ftc.teamcode.filters.particleFilter.ParticleFilter
import org.firstinspires.ftc.teamcode.subsystems.Vision
import org.firstinspires.ftc.teamcode.util.poseToMatrix

// TODO: Multiple instances of localizer will be created please fix oh god.
class MecanumMonteCarloLocalizer(
    hwMap: HardwareMap,
    private val vision: Vision,
    startingPose: Pose2d,
    startingStandardDeviation: SimpleMatrix
) : GlobalLocalizer {

    override var pose: Pose2d = startingPose
        get() {
            val state = particleFilter.stateEstimate.ddrm.data
            return Pose2d(state[0], state[1], state[2])
        }

    override var robotVelRobot: Twist2d = Twist2d(Vector2d(0.0, 0.0), 0.0)
        get() = odometryLocalizer.getVelocity()



    val odometryLocalizer = OdometryLocalizer(hwMap)
    private val visionMeasurementModel = VisionMeasurementModel(Vision.Companion.CameraData.LOGITECH_C920)

    private val particleFilter = ParticleFilter(50, odometryLocalizer, SimpleMatrix(arrayOf(doubleArrayOf(0.01, 0.01, 0.01))), visionMeasurementModel)

    private val timer = ElapsedTime()

    init {
        particleFilter.initialize(poseToMatrix(startingPose), startingStandardDeviation)
    }

    fun predict() {
        val dt = timer.seconds()

        odometryLocalizer.updateReadings()
        particleFilter.predict(odometryLocalizer.getReadingsMatrix(), dt)

        timer.reset()
    }

    fun update() {
        particleFilter.update(observationsToMatrix(vision.getLandmarkInfo()))
    }

    private fun observationsToMatrix(observations: List<Vision.ObservationResult>) : SimpleMatrix {
        val observationsArray = observations.map { doubleArrayOf(it.angle, it.distance)}.toTypedArray()
        return SimpleMatrix(observationsArray).transpose()
    }

}
