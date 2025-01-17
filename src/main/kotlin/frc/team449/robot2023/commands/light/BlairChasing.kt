package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.light.Light

/** Description: Have a linear transition from white to red */
class BlairChasing(
  private val led: Light
) : CommandBase() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstSaturation = 0.0

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      // This number is related to how many lights will show up between the high and low intensity (which technically also affects how fast itll cycle)
      val saturation = MathUtil.inputModulus(firstSaturation + i * 300.0 / led.buffer.length, 0.0, 255.0)
      led.setHSV(i, 0, saturation.toInt(), 255)

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstSaturation += 0.0375
      firstSaturation = MathUtil.inputModulus(firstSaturation, 0.0, 255.0)
    }
  }
}
