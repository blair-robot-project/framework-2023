package frc.team449.control.holonomic

import com.revrobotics.AbsoluteEncoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.RelativeEncoder
import com.revrobotics.SparkMaxAbsoluteEncoder
import com.revrobotics.SparkMaxRelativeEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import frc.team449.robot2023.constants.drives.SwerveConstants
import frc.team449.system.motor.SparkUtil
import io.github.oblarg.oblog.Loggable
import kotlin.math.PI
import kotlin.math.abs


/**
 * Controls a Swerve Module.
 * @param name The name of the module (used for logging).
 * @param drivingID The id of the motor that controls the speed of the module.
 * @param turningID The id of the motor that controls the angle of the module.
 * @param inverted The inverted statuses of the (driving motor, turning motor, turning encoder] in a [Triple].
 * @param drivePidGains The pid gains for the driving motor.
 * @param turnPidGains The pid gains for the turning motor.
 * @param driveFeedforward The voltage predicting equation for a given speed of the module.
 * @param location The location of the module in reference to the center of the robot.
 * NOTE: In relation to the robot [+X is forward, +Y is left, and +THETA is Counter Clock-Wise].
 */
open class SwerveModule(
  private val name: String,
  drivingID: Int,
  turningID: Int,
  inverted: Triple<Boolean, Boolean, Boolean>,
  drivePidGains: Triple<Double, Double, Double>,
  turnPidGains: Triple<Double, Double, Double>,
  private val driveFeedforward: SimpleMotorFeedforward,
  turnEncoderOffset: Double,
  val location: Translation2d
) : Loggable {

  private val drivingMotor: CANSparkMax = CANSparkMax(drivingID, CANSparkMaxLowLevel.MotorType.kBrushless)
  private val turningMotor: CANSparkMax = CANSparkMax(turningID, CANSparkMaxLowLevel.MotorType.kBrushless)

  val drivingEnc: RelativeEncoder = drivingMotor.encoder
  val turningEnc: AbsoluteEncoder = turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)

  init {
    SparkUtil.applySparkSettings(
      drivingMotor,
      enableBrakeMode = true,
      inverted = inverted.first,
      currentLimit = SwerveConstants.DRIVE_CURRENT_LIM,
      encoder = drivingEnc,
      gearing = SwerveConstants.DRIVE_GEARING,
      unitPerRotation = SwerveConstants.DRIVE_UPR,
    )

    SparkUtil.applySparkSettings(
      turningMotor,
      enableBrakeMode = false,
      inverted = inverted.second,
      currentLimit = SwerveConstants.STEERING_CURRENT_LIM,
      encoder = turningEnc,
      gearing = 1.0,
      unitPerRotation = SwerveConstants.TURN_UPR,
      offset = turnEncoderOffset,
      encInverted = inverted.third
    )

    SparkUtil.setPID(drivingMotor, drivePidGains)
    SparkUtil.setPID(turningMotor, turnPidGains)

    SparkUtil.enableContinuousInput(turningMotor, 0.0, 2 * PI)
  }

  var desiredAngle = turningEnc.position
  var desiredSpeed = 0.0

  /** The module's [SwerveModuleState], containing speed and angle. */
  open var state: SwerveModuleState
    get() {
      return SwerveModuleState(
        drivingEnc.velocity,
        Rotation2d(turningEnc.position)
      )
    }
    set(desiredState) {
      if (abs(desiredState.speedMetersPerSecond) < .001) {
        stop()
        return
      }
      /** Ensure the module doesn't turn more than 90 degrees. */
      val state = SwerveModuleState.optimize(
        desiredState,
        Rotation2d(turningEnc.position)
      )

      desiredAngle = state.angle.radians
      desiredSpeed = state.speedMetersPerSecond
    }

  /** The module's [SwerveModulePosition], containing distance and angle. */
  open val position: SwerveModulePosition
    get() {
      return SwerveModulePosition(
        drivingEnc.position,
        Rotation2d(turningEnc.position)
      )
    }

  /** Set module speed to zero but keep module angle the same. */
  fun stop() {
    turningMotor.stopMotor()
    drivingMotor.stopMotor()
    desiredSpeed = 0.0
  }

  override fun configureLogName() = this.name

  open fun update() {
    /** CONTROL speed of module */
    val driveFF = driveFeedforward.calculate(
      desiredSpeed
    )

    drivingMotor.pidController.setReference(desiredSpeed, CANSparkMax.ControlType.kVelocity, 0, driveFF)

    /** CONTROL direction of module */
    turningMotor.pidController.setReference(desiredAngle, CANSparkMax.ControlType.kPosition)
  }
  
  companion object {
    /** Create a real or simulated [SwerveModule] based on the simulation status of the robot. */
    fun create(
      name: String,
      drivingID: Int,
      turningID: Int,
      inverted: Triple<Boolean, Boolean, Boolean>,
      turnEncoderOffset: Double,
      location: Translation2d
    ): SwerveModule {
      if (RobotBase.isReal()) {
        return SwerveModule(
          name,
          drivingID,
          turningID,
          inverted,
          Triple(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD),
          Triple(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD),
          SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA),
          turnEncoderOffset,
          location
        )
      } else {
        return SwerveModuleSim(
          name,
          drivingID,
          turningID,
          inverted,
          Triple(SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD),
          Triple(SwerveConstants.TURN_KP, SwerveConstants.TURN_KI, SwerveConstants.TURN_KD),
          SimpleMotorFeedforward(SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA),
          turnEncoderOffset,
          location
        )
      }
    }
  }
}

/** A "simulated" swerve module. Immediately reaches to its desired state. */
class SwerveModuleSim(
  name: String,
  drivingID: Int,
  turningID: Int,
  inverted: Triple<Boolean, Boolean, Boolean>,
  drivingPidGains: Triple<Double, Double, Double>,
  turningPidGains: Triple<Double, Double, Double>,
  driveFeedforward: SimpleMotorFeedforward,
  turnEncoderOffset: Double,
  location: Translation2d
) : SwerveModule(
  name,
  drivingID,
  turningID,
  inverted,
  drivingPidGains,
  turningPidGains,
  driveFeedforward,
  turnEncoderOffset,
  location
) {
  private var prevTime = Timer.getFPGATimestamp()
  private var moduleAngle = 0.0
  private var drivingVelocity = 0.0

  override var state: SwerveModuleState
    get() = SwerveModuleState(
      drivingEnc.velocity,
      Rotation2d(turningEnc.position)
    )
    set(desiredState) {
      super.state = desiredState
      drivingVelocity = desiredState.speedMetersPerSecond
      moduleAngle = desiredState.angle.radians
    }
  override val position: SwerveModulePosition
    get() = SwerveModulePosition(
      drivingEnc.position,
      Rotation2d(moduleAngle)
    )

  override fun update() {
    val currTime = Timer.getFPGATimestamp()
    drivingEnc.position += drivingVelocity * (currTime - prevTime)
    prevTime = currTime
  }
}
