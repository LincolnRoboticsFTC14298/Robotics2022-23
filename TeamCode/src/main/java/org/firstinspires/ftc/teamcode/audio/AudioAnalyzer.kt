package org.firstinspires.ftc.teamcode.audio

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import kotlin.math.abs

/**
 * Class for analyzing audio and getting recommended action from it
 * @author Tony Riggioni
 */
class AudioAnalyzer
{
    private var ar: AudioRecord? = null
    private var minSize = 0
    var volume = 0

    @SuppressLint("MissingPermission") // Checked in activity so it won't fail, probably
    fun start()
    {
        for (rate in arrayOf(8000, 11025, 16000, 22050, 44100))
        {
            val bufferSize = AudioRecord.getMinBufferSize(rate, AudioFormat.CHANNEL_CONFIGURATION_DEFAULT, AudioFormat.ENCODING_PCM_16BIT)
            if (bufferSize > 0)
            {
                minSize = AudioRecord.getMinBufferSize(rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT)
                ar = AudioRecord(MediaRecorder.AudioSource.MIC, rate, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, minSize)
                ar?.startRecording()
            }
        }
    }

    fun stop()
    {
        ar?.stop()
    }

    fun getAmplitude(): Int
    {
        val buffer = ShortArray(minSize)
        ar?.read(buffer, 0, minSize)

        // Prints Contents of buffer (Don't know what it is)

        var max = 0
        for (s in buffer)
        {
            if (abs(s.toInt()) > max)
            {
                max = abs(s.toInt())
            }
        }
        volume = max
        return max
    }

    fun getReccomendedAction(): PitchData
    {
        val buffer = ShortArray(minSize)
        ar?.read(buffer, 0, minSize)

        var data = ForierTransform().getForierTransformFromBuffer(buffer)
        return data
    }
}