package org.firstinspires.ftc.teamcode.vision.testing

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.vision.GeneralPipeline
open class TestGeneralPipeline(telemetry: Telemetry) : GeneralPipeline(displayMode = DisplayMode.ALL_CONTOURS, RobotConfig.CameraData.LOGITECH_C920, telemetry)