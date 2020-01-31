/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {
  /**
   * Creates a new MoveTurret.
   */
  private boolean done = false;
  private Turret t = Robot.turret;
  private int encoderValue = 0;
  private Joystick joy = new Joystick(1);
  private double target = 0;

  public MoveTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    t.stop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // encoderValue = t.getEncoderValue();
    target = joy.getRawAxis(3);
    if (joy.getRawButton(5)) {

      t.runForward();

    } else if (joy.getRawButton(4)) {

      t.runBackward();

    } else {

      t.stop();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    t.stop();
    done = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
