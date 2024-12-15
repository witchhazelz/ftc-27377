package org.firstinspires.ftc.teamcode.controls.gainmatrices

data class FullStateGains @JvmOverloads constructor(
    @JvmField var pGain: Double = 0.0,
    @JvmField var vGain: Double = 0.0,
    @JvmField var aGain: Double = 0.0,
)