/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class RunConveyorSensor extends CommandBase {
  /**
   * Creates a new RunConveyorSensor.
   */
  boolean isBall;
  int trueCounter, falseCounter;

  public RunConveyorSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBall = false;
    falseCounter = 0;
    trueCounter = 0;

    Robot.conveyor.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isBall = Robot.conveyor.getSensor();

    //check if ball blocking sensor, if it's been long enough, start the motor
    // if not, add to counter
    if (isBall == false && falseCounter == 5) {
      Robot.conveyor.start(.75);
      falseCounter = 0;
      trueCounter = 0;
    } else if (isBall == false) {
      falseCounter++;
    }
    //check if sensor is clear, if it's been long enough, stop the motor
    // if not, add to counter
    else if (isBall == true && trueCounter == 5) {
      Robot.conveyor.stop();
      trueCounter = 0;
      falseCounter = 0;
    } else if (isBall == true) {
      trueCounter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trueCounter = 0;
    falseCounter = 0;
    isBall = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
