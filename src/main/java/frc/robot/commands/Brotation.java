/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class Brotation extends CommandBase {
  /**
   * Creates a new Brotation.
   */
  char direction;
  double angle;
  boolean finished;
  public Brotation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(this.angle < 0){
      Robot.driveTrain.setDrive(0.5, -0.5);
    }
    else {
      Robot.driveTrain.setDrive(-0.5, 0.5);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Robot.navX.getYaw()<= (angle + 3) && Robot.navX.getYaw() >= (angle - 3))) {
      finished = true;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
    Robot.driveTrain.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}