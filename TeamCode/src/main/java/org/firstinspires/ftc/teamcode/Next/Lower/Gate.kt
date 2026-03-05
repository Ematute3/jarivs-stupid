package org.firstinspires.ftc.teamcode.Lower.Gate

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.GateConstants

/**
 * Gate Subsystem
 * Simple open/close control for shooting gate
 *
 * Usage:
 *   Bindings:  Gate.open / Gate.close  (InstantCommand vals)
 *   Direct:    Gate.setPosition(GateConstants.GATE_OPEN)
 *
 * NOTE: Previously had both `val open` and `fun open()` — this is a
 * Kotlin naming collision. Functions removed; use setPosition() for
 * direct calls and the vals for command bindings.
 */
object Gate : Subsystem {
    private var gateServo = ServoEx("gate")

    override fun initialize() {}

    /** Direct servo control — use this from Commands.kt and onStop() */
    fun setPosition(pos: Double) {
        gateServo.position = pos
    }

    /** InstantCommand for binding system (e.g. gamepad cross button) */
    val open = InstantCommand { setPosition(GateConstants.GATE_OPEN) }
    val close = InstantCommand { setPosition(GateConstants.GATE_CLOSED) }

    override fun periodic() {}
}