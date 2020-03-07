/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SnapTo0 extends CommandBase {
  /**
   * Creates a new SnapTo0.
   */
  boolean finished;
  public SnapTo0() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    if(Robot.navX.getYaw() < 0){
      Robot.driveTrain.setDrive(0.5, -0.5);
    }
    else {
      Robot.driveTrain.setDrive(-0.5, 0.5);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Robot.navX.getYaw() >= (-5) && Robot.navX.getYaw() <= 5)) {
      finished = true;
    } 
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
