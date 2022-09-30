package org.firstinspires.ftc.teamcode.audio

/**
 * Data class that holds values for the robot FROM AUDIO
 * In order to get this class from audio use the {@link org.firstinspires.ftc.teamcode.audio.ForierTransform}
 */
data class PitchData(var FWD: Float, val BCK: Float, val LFT: Float, val RHT: Float)
{
    override fun toString(): String
    {
        val builder = StringBuilder();
        builder.append("FWD: ")
            .append(FWD)
            .append(" BCK: ")
            .append(BCK)
            .append(" LFT: ")
            .append(LFT)
            .append(" RHT: ")
            .append(RHT);
        return builder.toString();
    }
}