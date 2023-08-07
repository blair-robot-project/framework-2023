package frc.team449.system.motor

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame
import com.revrobotics.MotorFeedbackSensor
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxAlternateEncoder
import com.revrobotics.SparkMaxRelativeEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.RobotController
import javax.naming.ldap.Control

class WrappedSparkMax(
  private val name: String,
  private val sparkMax: CANSparkMax,
  val encType: EncoderType,
  unitPerRotation: Double,
  gearing: Double,
  offset: Double,
  val countsPerRev: Int
) {

  private val sparkMaxController = sparkMax.pidController

  val encoder: MotorFeedbackSensor = when (encType) {
    EncoderType.DutyCycle -> sparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    EncoderType.HallSensor -> sparkMax.encoder
    EncoderType.Quadrature -> sparkMax.getAlternateEncoder(countsPerRev)
  }

  init {
    when (encoder) {
      is SparkMaxAbsoluteEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
        encoder.zeroOffset = offset
      }
      is SparkMaxRelativeEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
      }
      is SparkMaxAlternateEncoder -> {
        encoder.positionConversionFactor = unitPerRotation * gearing
        encoder.velocityConversionFactor = unitPerRotation * gearing / 60
      }
      else -> throw IllegalStateException("UNSUPPORTED ENCODER PLUGGED INTO SPARK MAX.")
    }

    sparkMaxController.setFeedbackDevice(encoder)
  }

  fun set(output: Double) {
    sparkMax.set(output)
  }

  fun setVoltage(voltage: Double) {
    sparkMax.setVoltage(voltage)
  }

  fun stop() {
    sparkMax.stopMotor()
  }

  /**
   * Set the PID and FF gains of the controller.
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative gain.
   */
  fun setPID(p: Double, i: Double, d: Double) {
    sparkMaxController.p = p
    sparkMaxController.i = i
    sparkMaxController.d = d
  }

  // Put PID values onto SmartDashboard and have a periodic() that checks if they are the same as the ones on the controller.
  // If !=, change the constants on the controller to match.

  fun setConstraints(minVel: Double = 0.0, maxVel: Double, maxAccel: Double, tolerance: Double) {
    sparkMaxController.setSmartMotionMinOutputVelocity(minVel, 0)
    sparkMaxController.setSmartMotionMaxVelocity(maxVel, 0)
    sparkMaxController.setSmartMotionMaxAccel(maxAccel, 0)
    sparkMaxController.setSmartMotionAllowedClosedLoopError(tolerance, 0)
  }

  /**
   * Enables continuous input on the PID controller.
   * Should not be used unless the desired mechanism is capable of FULLY continuous rotational motion.
   * @param min The minimum value expected from the input.
   * @param max The maximum value expected from the input.
   */
  fun enableContinuousInput(min: Double, max: Double) {
    sparkMaxController.setPositionPIDWrappingEnabled(true)
    sparkMaxController.setPositionPIDWrappingMaxInput(max)
    sparkMaxController.setPositionPIDWrappingMinInput(min)
  }

  /**
   * Set the controller reference based on the specified [controlType].
   * @param reference The setpoint of the PID controller.
   * @param controlType The control type that the controller should match.
   * kSmartMotion for position control (rotations) or kVelocity for velocity control (RPM).
   * @param feedForwardVoltage The arbitrary feedforward voltage to add after the controller has calculated its next setpoint.
   * This value should be calculated using WPILib.
   */
  fun setGoal(reference: Double, controlType: CANSparkMax.ControlType, feedForwardVoltage: Double = 0.0) {
    // controlType should be kSmartMotion for position, and kVelocity for velocity.
    sparkMaxController.setReference(reference, controlType, 0, feedForwardVoltage)
  }



  companion object {
    fun createWrappedSpark(
      // spark max information
      name: String,
      id: Int,
      enableBrakeMode: Boolean = true,
      inverted: Boolean = false,
      currentLimit: Int = 0,
      enableVoltageComp: Boolean = false,
      slaveSparks: Map<Int, Boolean> = mapOf(),
      controlFrameRateMillis: Int = -1,
      statusFrameRatesMillis: Map<PeriodicFrame, Int> = mapOf(),
      // encoder information
      encType: EncoderType,
      unitPerRotation: Double,
      gearing: Double,
      offset: Double = Double.NaN,
      countsPerRev: Int
    ): WrappedSparkMax {

      val sparkMax = CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless)

      sparkMax.restoreFactoryDefaults()
      sparkMax.idleMode = if (enableBrakeMode) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
      sparkMax.inverted = inverted
      if (currentLimit > 0) sparkMax.setSmartCurrentLimit(currentLimit)
      if (enableVoltageComp) sparkMax.enableVoltageCompensation(RobotController.getBatteryVoltage()) else sparkMax.disableVoltageCompensation()
      if (controlFrameRateMillis >= 1) sparkMax.setControlFramePeriodMs(controlFrameRateMillis) // Must be between 1 and 100 ms.
      for ((statusFrame, period) in statusFrameRatesMillis) {
        sparkMax.setPeriodicFramePeriod(statusFrame, period)
      }

      for ((slavePort, slaveInverted) in slaveSparks) {
        val slave = CANSparkMax(slavePort, CANSparkMaxLowLevel.MotorType.kBrushless)
        slave.restoreFactoryDefaults()
        slave.follow(sparkMax, slaveInverted)
        slave.idleMode = sparkMax.idleMode
        if (currentLimit > 0) slave.setSmartCurrentLimit(currentLimit)
        slave.burnFlash()
      }

      sparkMax.burnFlash()

      return WrappedSparkMax(name, sparkMax, encType, unitPerRotation, gearing, offset, countsPerRev)
    }
  }
}

enum class EncoderType {
  DutyCycle,
  HallSensor,
  Quadrature
}