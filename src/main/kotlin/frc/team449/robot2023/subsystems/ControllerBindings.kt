package frc.team449.robot2023.subsystems

import edu.wpi.first.wpilibj.XboxController
import frc.team449.robot2023.Robot

class ControllerBindings(
  private val driveController: XboxController,
  private val mechanismController: XboxController,
  private val robot: Robot
) {

  /** Binds the controls for the robot. */
  fun bindButtons() {
    /** Example:
     * JoystickButton(driveController, XboxController.Button.kLeftBumper.value).onTrue(
     *        PrintCommand("hello!")
     *     ).onFalse(
     *       PrintCommand("goodbye.").andThen(
     *          PrintCommand("goodbye again.")
     *       )
     *     }
     */
  }
}
