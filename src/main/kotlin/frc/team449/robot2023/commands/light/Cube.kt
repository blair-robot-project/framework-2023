package frc.team449.robot2023.commands.light

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team449.robot2023.subsystems.light.Light

/** Description: Have a linear transition from a darker to brighter purple */
class Cube(
  private val led: Light
) : CommandBase() {

  init {
    addRequirements(led)
  }

  override fun runsWhenDisabled(): Boolean {
    return true
  }

  private var firstIntensity = 175.0

  override fun execute() {
    for (i in 0 until led.buffer.length) {
      // This number is related to how many lights will show up between the high and low intensity
      val intensity = MathUtil.inputModulus(firstIntensity + i * 37.5 / led.buffer.length, 150.0, 255.0)
      led.setHSV(i, 150, 255, intensity.toInt())

      // The i * 255.0 relates to how fast it will cycle in between the high and low intensity
      firstIntensity += 0.075
      firstIntensity = MathUtil.inputModulus(firstIntensity, 150.0, 255.0)
    }
  }
}
