package org.firstinspires.ftc.teamcode.Lower.Gate

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import org.firstinspires.ftc.teamcode.GateConstants

/**
 * Gate Subsystem
 * Simple open/close control for shooting gate
 */
object Gate : Subsystem {
    private var gateServo = ServoEx("gate")

    override fun initialize() {}

    internal fun setPosition(pos: Double) {
        gateServo.position = pos
    }

    val open = InstantCommand { setPosition(GateConstants.GATE_OPEN) }
    val close = InstantCommand { setPosition(GateConstants.GATE_CLOSED) }

    fun open() {
        setPosition(GateConstants.GATE_OPEN)
    }

    fun close() {
        setPosition(GateConstants.GATE_CLOSED)
    }

    override fun periodic() {
        // No periodic updates needed for simple servo
    }
}