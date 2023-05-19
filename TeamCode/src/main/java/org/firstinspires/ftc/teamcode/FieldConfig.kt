package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Vector2d

object FieldConfig {

    const val tileSize = 23.5 // in

    const val coneDiameter = 5.0 // in TODO measure
    const val poleDiameter = 1.0 // in TODO measure
    const val poleBaseHeight = 3.0 //in TODO measure

    /**
     * Different types of poles. Height in inches.
     */
    enum class PoleType(val height: Double) {
        LOW(13.5),
        MEDIUM(23.5),
        HIGH(33.5)
    }

    /**
     * All poles on the field.
     */
    enum class Pole(x: Int, y: Int, val type: PoleType) {
        TWO_ONE(2, 1, PoleType.LOW),
        TWO_NEG_ONE(2, -1, PoleType.LOW),

        ONE_TWO(1, 2, PoleType.LOW),
        ONE_ONE(1, 1, PoleType.MEDIUM),
        ONE_ZERO(1, 0, PoleType.HIGH),
        ONE_NEG_ONE(1, -1, PoleType.MEDIUM),
        ONE_NEG_TWO(1, -2, PoleType.LOW),

        ZERO_ONE(0, 1, PoleType.HIGH),
        ZERO_NEG_ONE(0, -1, PoleType.HIGH),

        NEG_ONE_TWO(-1, 2, PoleType.LOW),
        NEG_ONE_ONE(-1, 1, PoleType.MEDIUM),
        NEG_ONE_ZERO(-1, 0, PoleType.HIGH),
        NEG_ONE_NEG_ONE(-1, -1, PoleType.MEDIUM),
        NEG_ONE_NEG_TWO(-1, -2, PoleType.LOW),

        NEG_TWO_ONE(-2, 1, PoleType.LOW),
        NEG_TWO_NEG_ONE(-2, -1, PoleType.LOW);

        val vector = Vector2d(x * tileSize, y * tileSize)

        companion object {
            fun getPolesOfType(type: PoleType) : List<Pole> {
                return values().filter { it.type == type }
            }
        }
    }

    enum class Junction(x: Int, y: Int) {
        TWO_TWO(2, 2),
        TWO_ZERO(2, 0),
        TWO_NEG_TWO(2, -2),

        ZERO_TWO(0, 2),
        ZERO_ZERO(0, 0),
        ZERO_NEG_TWO(0,-2),

        NEG_TWO_TWO(-2, 2),
        NEG_TWO_ZERO(-2, 0),
        NEG_TWO_NEG_TWO(-2, -2);

        val vector = Vector2d(x * tileSize, y * tileSize)

    }

}