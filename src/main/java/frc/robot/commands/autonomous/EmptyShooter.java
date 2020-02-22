/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class EmptyShooter extends CommandBase {
  /**
   * Creates a new EmptyShooter.
   */

  public EmptyShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(new Shooter(), new Conveyor());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Robot.shooter.setBottomSpeed(Constants.AUTON_SHOOTER_SPEED);
    Robot.conveyor.start(Constants.AUTON_CONVEYOR_SPEED);
    //sensor check here if(!topSensor){ isFinished = true;}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
