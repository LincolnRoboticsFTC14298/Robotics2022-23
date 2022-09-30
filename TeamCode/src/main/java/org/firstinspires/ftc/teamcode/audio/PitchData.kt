package org.firstinspires.ftc.teamcode.audio

/**
 * Data class that holds values for robot control
 * Values are get from audio
 * @see org.firstinspires.ftc.teamcode.audio.AudioAnalyzer
 * @see org.firstinspires.ftc.teamcode.audio.ForierTransform
 * @author Tony Riggioni
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