package org.firstinspires.ftc.teamcode.util.Action

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class ParallelDeadlineAction(
    val deadline: Action,
    val initialActions: List<Action>
) : Action {
    private var actions = initialActions

    constructor(deadline: Action, vararg actions: Action) : this(deadline, actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        actions = actions.filter { it.run(p) }
        return deadline.run(p)
    }

    override fun preview(fieldOverlay: Canvas) {
        deadline.preview(fieldOverlay)
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}