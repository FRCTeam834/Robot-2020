/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterPID extends PIDCommand {

  public ShooterPID(double targetBottomRPM) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        (Robot.shooter.getBottomEncoder()::getVelocity),
        // This should return the setpoint (can also be a constant)
        (targetBottomRPM),
        // This uses the output
        output -> {
          // Use the output here
          Robot.shooter.setBottomSpeed(Robot.shooter.normalize(output));
          Robot.shooter.setTopSpeed(Robot.shooter.normalize(output));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooter);
    // Configure additional PID options by calling `getController` here.

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
