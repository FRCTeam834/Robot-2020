/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.createdclasses.Goal;
import frc.robot.subsystems.GimbalLock;

public class LookAtGoalY extends CommandBase {
  /**
   * Creates a new LookAtGoalY.
   */

  private GimbalLock g = Robot.gimbalLock;
  private double angleRequirement = 0;
  private boolean isFinished = false;
  private Goal goal;

  public LookAtGoalY() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.gimbalLock);
    addRequirements(Robot.EVSNetworkTables);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.stop();

    isFinished = false;

    try {

      if (Robot.EVSNetworkTables.getGoalArray().get(0).size() != 0) {

        goal = new Goal(Robot.EVSNetworkTables.getGoalArray().get(0));

      } else {

        System.out.println("No Goal Found");
        isFinished = true;

      }

    } catch (Exception e) {

    }

    double distance = goal.getDistance() * Math.sin(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION)/12; //The 5 is a placeholder for shooter angle

    //Apply Angle formula
    //98.25/12 - Constants.ROBOT_HEIGHT - Constants.SHOOTER_MOUTH_WIDTH * Math.cos(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION) = Math.tan(theta) (distance - Constants.SHOOTER_MOUTH_WIDTH * Math.sin(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION)) - 16 * Math.pow(distance - Constants.SHOOTER_MOUTH_WIDTH * Math.sin(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION), 2)/(Math.pow(Constants.S_WHEEL_SPEED * Constants.WHEEL_CIRCUMFERENCE * Math.cos(theta), 2))
    98.25/12 - Constants.ROBOT_HEIGHT - Constants.SHOOTER_MOUTH_WIDTH * Math.cos(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION) = Math.tan(theta) (distance - Constants.SHOOTER_MOUTH_WIDTH * Math.sin(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION)) - 16 * Math.pow(distance - Constants.SHOOTER_MOUTH_WIDTH * Math.sin(g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION), 2)/(Math.pow(Constants.S_WHEEL_SPEED * Constants.WHEEL_CIRCUMFERENCE * Math.cos(theta), 2))
    //angleRequirement = solution to ^^.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION < angleRequirement) {
      g.tiltUp(Constants.GIMBAL_MULTIPLIER
          * (angleRequirement - g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION));
    } else if (g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION > angleRequirement)
      ;
    g.tiltDown(Constants.GIMBAL_MULTIPLIER
        * (angleRequirement - g.getEncoder().getDistance() * Constants.VENEZUELA_INFLATION));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

}
