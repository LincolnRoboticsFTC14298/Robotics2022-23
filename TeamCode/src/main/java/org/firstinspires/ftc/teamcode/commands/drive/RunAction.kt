package org.firstinspires.ftc.teamcode.commands.drive

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.arcrobotics.ftclib.command.CommandBase

class RunAction(
    private val action: Action,
    private val packet: () -> TelemetryPacket
) : CommandBase() {

    private var done: Boolean = false
    override fun execute() {
        val packet = packet.invoke()

        done = !action.run(packet)
        action.preview(packet.fieldOverlay())
    }

    override fun isFinished(): Boolean = done

}