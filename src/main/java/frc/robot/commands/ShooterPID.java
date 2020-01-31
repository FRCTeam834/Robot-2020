/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterPID extends PIDCommand {
  /**
   * Creates a new ShooterPID.
   */
  public ShooterPID(Shooter shooter) {
    super(
        // The controller that the command will use
        new PIDController(Constants.S_PROPORTIONAL_CONSTANT, Constants.S_INTEGRAL_CONSTANT,
            Constants.S_DERIVATIVE_CONSTANT),
        // This should return the measurement
        shooter.getBottomEncoder()::getVelocity,
        // This should return the setpoint (can also be a constant)
        Constants.S_BOTTOM_WHEEL_SPEED,
        // This uses the output
        output -> {
          // Use the output here
          double currentSetting = shooter.getBottomMotor().get();
          shooter.setBottomSpeed(currentSetting + shooter.normalize(output));
          shooter.setTopSpeed(Constants.S_BACKSPIN_RATIO * (currentSetting + shooter.normalize(output)));
        });

    getController().enableContinuousInput(0, 1);
    getController().setTolerance(.005);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return getController().atSetpoint();

  }
}
