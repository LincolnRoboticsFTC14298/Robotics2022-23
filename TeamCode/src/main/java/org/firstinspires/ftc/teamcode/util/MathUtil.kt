package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Vector2d

/**
 * @return Normalized vector.
 */
fun Vector2d.normalize(): Vector2d {
    return this.div(this.norm())
}