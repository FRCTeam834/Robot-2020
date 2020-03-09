/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

<<<<<<< HEAD:src/main/java/frc/robot/commands/autonomous/autons/CenterAuto.java
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.EmptyShooterNoVision;
import frc.robot.commands.autonomous.ShooterToSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterAuto extends SequentialCommandGroup {
=======
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveBackwardsDistance extends CommandBase {
>>>>>>> master:src/main/java/frc/robot/commands/DriveBackwardsDistance.java
  /**
   * Creates a new DriveBackwardsDistance.
   */
<<<<<<< HEAD:src/main/java/frc/robot/commands/autonomous/autons/CenterAuto.java
  public CenterAuto() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ShooterToSpeed(), new EmptyShooterNoVision());
=======
  double distance, encoderStart;
  boolean finished;
  public DriveBackwardsDistance(double dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    distance = dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    Robot.driveTrain.resetEncoderPosition();
    Robot.driveTrain.setDrive(.2, .2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance <= (Robot.driveTrain.getRightEncoderValue() *3.015)) {
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
>>>>>>> master:src/main/java/frc/robot/commands/DriveBackwardsDistance.java
  }
}
