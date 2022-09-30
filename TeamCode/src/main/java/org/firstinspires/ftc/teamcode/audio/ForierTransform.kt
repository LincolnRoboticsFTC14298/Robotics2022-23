package org.firstinspires.ftc.teamcode.audio

import org.firstinspires.ftc.teamcode.audio.PitchData

/**
 * Class for calculating forier transform of audio ShortArray buffers
 * @see org.firstinspires.ftc.teamcode.audio.AudioAnalyzer
 * @author Tony Riggioni
 */
class ForierTransform
{
    public fun getForierTransformFromBuffer(buffer: ShortArray): PitchData
    {
        TODO();

        var data = PitchData(0.0f, 0.0f, 0.0f, 0.0f);
        return data;
    }
}