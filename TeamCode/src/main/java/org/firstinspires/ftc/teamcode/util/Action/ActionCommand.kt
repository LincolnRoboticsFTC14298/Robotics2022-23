package org.firstinspires.ftc.teamcode.util.Action

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.command.CommandBase

class ActionCommand (
    private val action: Action
) : CommandBase() {

    private val dash = FtcDashboard.getInstance()

    private var b = true
    override fun execute() {
        val c = Canvas()
        action.preview(c)

        val p = TelemetryPacket()
        p.fieldOverlay().operations.addAll(c.operations)
        b = action.run(p)

        dash.sendTelemetryPacket(p)
    }

    override fun isFinished(): Boolean {
        return b
    }
}