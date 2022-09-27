package org.firstinspires.ftc.teamcode

import android.media.AudioFormat
import android.media.AudioRecord
import android.media.MediaRecorder

class AudioAnalyzer
{
    private var ar: AudioRecord? = null;
    private var minSize = 0;

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
        var max = 0;
        for (s in buffer)
        {
            if (Math.abs(s.toInt()) > max)
            {
                max = Math.abs(s.toInt());
            }
        }
        return max;
    }
}