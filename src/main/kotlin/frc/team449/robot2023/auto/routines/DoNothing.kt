package frc.team449.robot2023.auto.routines

import com.pathplanner.lib.PathPlannerTrajectory
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.control.auto.HolonomicRoutine
import frc.team449.control.auto.RoutineStructure
import frc.team449.robot2023.Robot

class DoNothing(
  robot: Robot
) : RoutineStructure {

  override val routine =
    HolonomicRoutine(
      drive = robot.drive,
      eventMap = hashMapOf()
    )

  override val trajectory: MutableList<PathPlannerTrajectory> = mutableListOf(PathPlannerTrajectory())

  override fun createCommand(): Command {
    return InstantCommand()
  }
}
