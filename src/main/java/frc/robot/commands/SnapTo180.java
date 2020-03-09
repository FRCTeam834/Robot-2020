/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
<<<<<<< HEAD
import frc.robot.subsystems.DriveTrain;
=======
>>>>>>> master
import java.lang.Math;

public class SnapTo180 extends CommandBase {
  /**
   * Creates a new Brotation.
   */
  boolean finished;
<<<<<<< HEAD
  public SnapTo180() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);    
=======
  double lMotor, rMotor;
  public SnapTo180() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain, Robot.navX);    
>>>>>>> master
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    if(Math.abs(Robot.navX.getYaw()-180) < Math.abs(Robot.navX.getYaw())) {

    }
    if(Robot.navX.getYaw() < 0){
      Robot.driveTrain.setDrive(0.5, -0.5);
    }
    else {
      Robot.driveTrain.setDrive(-0.5, 0.5);
    }
=======
    finished = false;
    lMotor = 1; 
    rMotor =1;

    lMotor = 1;
    rMotor = -1;
    Robot.driveTrain.setDrive(.25, -.25);
   /* //start turning robot in correct direction
    if (Robot.navX.getYaw() < 0) { //if robot to right of 180, turn left
      Robot.driveTrain.setDrive(-0.5, 0.5);
      lMotor = -1;
      rMotor = 1;
    } else if (Robot.navX.getYaw() >= 0) { //if robot to left of 180, turn right
      Robot.driveTrain.setDrive(0.5, -0.5);
      lMotor = 1;
      rMotor = -1;
    } 
    */
>>>>>>> master
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    if((Robot.navX.getYaw()<= (-175) && Robot.navX.getYaw() >= (175))) {
      finished = true;
    } 
=======
    if(Math.abs(Robot.navX.getYaw()) >= 165) { // spin slower once close for percision 
      Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
      if(Math.abs(Robot.navX.getYaw()) >= 175) {
        //now the that we are facing 180, we can gg ez stop spinning
        finished = true;
      }
     } 

>>>>>>> master
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
<<<<<<< HEAD
    return false;
=======
    return finished;
>>>>>>> master
  }
}