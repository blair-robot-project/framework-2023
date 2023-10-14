package frc.team449.robot2023.subsystems.testShooter

object ShooterConstants {
  const val SHOOTER_ID = 23

  // Order: kP, kD, kI
  val kPID = Triple(0.1, 0.0, 0.0)

  // Order: kS, kV, kA
  val kFF = Triple(0.15, 0.15, 0.0075)

  const val GOAL = 25.0
}