package org.firstinspires.ftc.teamcode.controls.gainmatrices

data class FeedforwardGains @JvmOverloads constructor(
    @JvmField var kV: Double = 0.0,
    @JvmField var kA: Double = 0.0,
    @JvmField var kStatic: Double = 0.0,
)