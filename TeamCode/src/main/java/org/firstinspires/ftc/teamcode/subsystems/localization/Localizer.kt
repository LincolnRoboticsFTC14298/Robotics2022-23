package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig.Pole
import org.firstinspires.ftc.teamcode.RobotConfig.tileSize
import org.firstinspires.ftc.teamcode.subsystems.drive.StandardTrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.util.normalize
import kotlin.math.floor

// TODO: Multiple instances of localizer will be created please fix oh god.
class Localizer(hwMap: HardwareMap) : SubsystemBase() {

    private val localizer =
        StandardTrackingWheelLocalizer(
            hwMap
        )

    val poseEstimate: Pose2d
        get() {
            return localizer.poseEstimate
        }

    val poseVelocity: Pose2d?
        get() {
            return localizer.poseVelocity
        }

    /**
     * TODO: TEST MULTIPLE ALGORITHMS
     * @return Gets the pole the robot is facing by minimizing the difference of heading.
     */
    fun getFacingPole(): Pole? {
        val headVec = poseEstimate.headingVec()
        val tileVec = poseEstimate.vec().div(tileSize)

        val minX = floor(tileVec.x / tileSize)
        val minY = floor(tileVec.y / tileSize)

        val highestCosT = -1.0
        var optimalPole: Pole? = null

        for (x in listOf(minX * tileSize, (minX+1) * tileSize)) {
            for (y in listOf(minY * tileSize, (minY+1) * tileSize)) {
                val corner = Vector2d(x, y)
                val pole = Pole.getPole(corner)
                if (pole != null) {
                    val diff = corner.minus(tileVec).normalize()
                    val cosT = diff.dot(headVec)

                    // Highest dot product means the angle between the corner and heading is lowest
                    if (cosT > highestCosT) {
                        optimalPole = pole
                    }
                }
            }
        }

        return optimalPole
    }
}
