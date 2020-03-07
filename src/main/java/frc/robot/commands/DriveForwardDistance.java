/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveForwardDistance extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  double distance, encoderStart;
  boolean finished;
  public DriveForwardDistance(double dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    distance = dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    encoderStart = Robot.driveTrain.getRightEncoderValue();
    Robot.driveTrain.setDrive(.5, .5);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance <= (Robot.driveTrain.getRightEncoderValue() - encoderStart) * Constants.DRIVE_ENCODER_MULTIPLIER) {
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
    return finished;
  }
}
