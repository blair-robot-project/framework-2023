package frc.team449.robot2023.subsystems.testShooter

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.MotorFeedbackSensor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.motor.EncoderType
import frc.team449.system.motor.SparkUtil

class Shooter(
  shooterId: Int
): SubsystemBase() {

  private val shooterMotor = CANSparkMax(shooterId, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val shooterEncoder = shooterMotor.encoder

  init {
    SparkUtil.applySparkSettings(
      shooterMotor,
      encoder = shooterEncoder,
      gearing = 1.0,
      unitPerRotation = 1.0,
    )

    SparkUtil.setPID(shooterMotor, Triple(1.0, 0.0, 0.0))
  }

  private var runShoot = false

  fun runShoot(): Command {
    return this.runOnce { shooterMotor.pidController.setReference(5.0, CANSparkMax.ControlType.kVelocity) }
  }

  fun stopShoot(): Command {
    return this.runOnce { shooterMotor.stopMotor() }
  }


  companion object {
    fun createShooter(): Shooter {
      return Shooter(ShooterConstants.SHOOTER_ID)
    }
  }

}