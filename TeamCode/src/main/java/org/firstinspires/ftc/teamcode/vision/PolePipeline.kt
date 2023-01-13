package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.teamcode.vision.modulelib.InputModule
import org.firstinspires.ftc.teamcode.vision.modulelib.ModularPipeline
import org.firstinspires.ftc.teamcode.vision.modules.*
import org.firstinspires.ftc.teamcode.vision.modules.FilterContours
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

class PolePipeline : ModularPipeline() {

    //val generalConePipeline = GeneralConePipeline()

    // Same thing mask, denoise, contour, filter contour, results

    init {
        addEndModules()
    }

    override fun processFrameForCache(input: Mat): Mat {


        return input
    }

}