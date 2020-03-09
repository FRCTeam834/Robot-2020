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

public class EmptyShooterNoVision extends CommandBase {
  /**
   * Creates a new EmptyShooterNoVision.
   */
<<<<<<< HEAD
  double time, speed, timeStart;
=======
  double time, timeStart;
>>>>>>> master
  boolean finished;
  public EmptyShooterNoVision() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooter, Robot.conveyor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    time = 3;
    speed = .75;
=======
    time = 2;
>>>>>>> master
    timeStart = System.currentTimeMillis();
    finished = false;
    Robot.shooter.getMotor().setVoltage(Constants.S_WHEEL_VOLTAGE);
    Robot.conveyor.start(Constants.AUTON_CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
=======
    if(((System.currentTimeMillis()-timeStart)/1000) > time) {
      finished = true;
    }
>>>>>>> master
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< HEAD
    finished = false;
=======
>>>>>>> master
    Robot.shooter.stop();
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
