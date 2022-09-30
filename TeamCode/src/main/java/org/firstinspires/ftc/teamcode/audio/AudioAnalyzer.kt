package org.firstinspires.ftc.teamcode.audio

import android.annotation.SuppressLint
import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder
import kotlin.math.abs

class AudioAnalyzer
{
    private var ar: AudioRecord? = null;
    private var minSize = 0;

    @SuppressLint("MissingPermission")
    fun start()
    {
        minSize = AudioRecord.getMinBufferSize(8000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);
        ar = AudioRecord(MediaRecorder.AudioSource.MIC, 8000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, minSize);
        ar!!.startRecording();
    }

    fun stop()
    {
        ar!!.stop();
    }

    fun getAmplitude(): Int
    {
        val buffer = ShortArray(minSize);
        ar!!.read(buffer, 0, minSize);

        // Prints Contents of buffer (Don't know what it is)
        val builder = StringBuilder();
        builder.append(buffer)
            .append(" With Size Of: ")
            .append(buffer.size);
        println(builder.toString());

        var max = 0;
        for (s in buffer)
        {
            if (abs(s.toInt()) > max)
            {
                max = abs(s.toInt());
            }
        }
        return max;
    }

    private fun forierTransform(buffer: ShortArray): FloatArray
    {
        var out = FloatArray(4);

        out[0] = 0.0f;
        out[1] = 0.0f;
        out[2] = 0.0f;
        out[3] = 0.0f;

        return out;
    }

    fun getPitches(): FloatArray
    {
        var pitchValues = FloatArray(4);

        // TODO: Implement Forier transform
//        forierTransform();

        return pitchValues;
    }
}