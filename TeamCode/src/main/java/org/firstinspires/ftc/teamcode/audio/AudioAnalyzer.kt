package org.firstinspires.ftc.teamcode.audio

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.audio.ForierTransform

/**
 * Class for analyzing audio and getting recommended action from it
 * @author Tony Riggioni
 */
class AudioAnalyzer
{
    private var ar: AudioRecord? = null
    private var minSize = 0

    @SuppressLint("MissingPermission")
    fun start()
    {
        minSize = AudioRecord.getMinBufferSize(8000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
        ar = AudioRecord(MediaRecorder.AudioSource.MIC, 8000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, minSize)
        ar!!.startRecording()
    }

    fun stop()
    {
        ar!!.stop()
    }

    fun getAmplitude(): Int
    {
        val buffer = ShortArray(minSize)
        ar!!.read(buffer, 0, minSize)

        // Prints Contents of buffer (Don't know what it is)
        val builder = StringBuilder()
        builder.append(buffer)
            .append(" With Size Of: ")
            .append(buffer.size)
        println(builder.toString())

        var max = 0
        for (s in buffer)
        {
            if (abs(s.toInt()) > max)
            {
                max = abs(s.toInt())
            }
        }
        return max
    }

    fun getReccomendedAction(): PitchData
    {
        val buffer = ShortArray(minSize)
        ar!!.read(buffer, 0, minSize)

        var data = ForierTransform().getForierTransformFromBuffer(buffer)
        return data
    }
}