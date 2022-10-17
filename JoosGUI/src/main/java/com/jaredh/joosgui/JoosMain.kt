package com.jaredh.joosgui

import com.amarcolini.joos.gui.GUI
import com.amarcolini.joos.gui.rendering.Backgrounds
import com.amarcolini.joos.gui.style.Themes
import com.amarcolini.joos.gui.trajectory.WaypointBuilder
import com.amarcolini.joos.trajectory.constraints.GenericConstraints

fun main() {
    GUI()
        .setBackground(Backgrounds.Generic)
        .setTheme(Themes.Dark)
        .followTrajectory(
            WaypointBuilder(constraints = GenericConstraints(40.0, 1000.0))
                .build()
        )
        .start()
}
