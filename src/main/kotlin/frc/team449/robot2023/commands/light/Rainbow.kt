package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.light.Light

class Rainbow(
  private val led: Light
) : CommandBase() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstHue = 0.0

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      val hue = MathUtil.clamp(firstHue + i * 180 / led.buffer.length, 0.0, 180.0)

      led.setHSV(i, hue.toInt(), 255, 255)
    }
    firstHue += 1.5
    firstHue %= 180
  }
}
