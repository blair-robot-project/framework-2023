package frc.team449.robot2023.subsystems.testShooter

<<<<<<< HEAD
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelSelf
=======
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.MotorFeedbackSensor
<<<<<<< HEAD
>>>>>>> parent of 8cf148c (cleaned up new spark max and code and shooter code to teach the rookies)
=======
>>>>>>> parent of 8cf148c (cleaned up new spark max and code and shooter code to teach the rookies)
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.system.encoder.NEOEncoder
import frc.team449.system.motor.createSparkMax
import io.github.oblarg.oblog.Loggable

class Shooter(
  shooterId: Int,
  pidConstants: Triple<Double, Double, Double>,
  ffConstants: Triple<Double, Double, Double>,
  private val goal: Double
): SubsystemBase(), Loggable {

  private val motor = createSparkMax(
    "shooter",
    shooterId,
    NEOEncoder.creator(1.0, 1.0)
  )

  private val controller = PIDController(
    pidConstants.first,
    pidConstants.second,
    pidConstants.third
    )

  private val ff = SimpleMotorFeedforward(
    ffConstants.first,
    ffConstants.second,
    ffConstants.third
  )

<<<<<<< HEAD
<<<<<<< HEAD
  fun runShoot(): Command {
    return this.runOnce {
      motor.setVoltage(
        controller.calculate(motor.velocity, goal) +
          ff.calculate(goal)
      )
    }.repeatedly()
      .withInterruptBehavior(kCancelSelf)
  }

  fun stopShoot(): Command {
    return this.runOnce {
      motor.setVoltage(0.0)
    }
  }

  companion object {
    fun createShooter(): Shooter {
      return Shooter(
        ShooterConstants.SHOOTER_ID,
        ShooterConstants.kPID,
        ShooterConstants.kFF,
        ShooterConstants.GOAL
      )
=======
  fun runShoot() {
    runShoot = true;
  }

  fun stopShoot() {
    runShoot = false;
  }

  override fun periodic() {
    if (runShoot) {
      shooterMotor.pidController.setReference(5.0, CANSparkMax.ControlType.kVelocity)
    }
    else {
      shooterMotor.stopMotor()
    }
  }

  companion object {
    fun createShooter(): Shooter {
      return Shooter(ShooterConstants.SHOOTERID)
>>>>>>> parent of 8cf148c (cleaned up new spark max and code and shooter code to teach the rookies)
=======
  fun runShoot() {
    runShoot = true;
  }

  fun stopShoot() {
    runShoot = false;
  }

  override fun periodic() {
    if (runShoot) {
      shooterMotor.pidController.setReference(5.0, CANSparkMax.ControlType.kVelocity)
    }
    else {
      shooterMotor.stopMotor()
    }
  }

  companion object {
    fun createShooter(): Shooter {
      return Shooter(ShooterConstants.SHOOTERID)
>>>>>>> parent of 8cf148c (cleaned up new spark max and code and shooter code to teach the rookies)
    }
  }

}