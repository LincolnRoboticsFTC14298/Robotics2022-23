package org.firstinspires.ftc.teamcode.vision.testing

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.vision.GeneralPipeline
open class TestGeneralPipeline(telemetry: Telemetry) : GeneralPipeline(displayMode = DisplayMode.ALL_CONTOURS, 60.0, 60.0, 0.0, 0.0, telemetry) {
}