/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveMaxSpeed;
import frc.robot.commands.DriveNormal;
import frc.robot.commands.DriveSlowSpeed;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunShooter;
import frc.robot.commands.SpinCP;
import frc.robot.commands.SetCPColor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  private final DriveNormal driveNormal = new DriveNormal();
  private final DriveSlowSpeed driveSlowSpeed = new DriveSlowSpeed();
  private final DriveMaxSpeed driveMaxSpeed = new DriveMaxSpeed();
  private final RunIntake runIntake = new RunIntake();
  private final RunShooter runShooter = new RunShooter();
  private final SpinCP spinCP = new SpinCP();
  private final SetCPColor setCPColor = new SetCPColor();
  private final RunConveyor runConveyor = new RunConveyor();

  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  private final JoystickButton leftButton0 = new JoystickButton(leftJoystick, 0);
  private final JoystickButton leftButton1 = new JoystickButton(leftJoystick, 1);
  private final JoystickButton leftButton2 = new JoystickButton(leftJoystick, 2);
  private final JoystickButton leftButton3 = new JoystickButton(leftJoystick, 3);
  private final JoystickButton leftButton4 = new JoystickButton(leftJoystick, 4);
  private final JoystickButton leftButton6 = new JoystickButton(leftJoystick, 6);

  private final JoystickButton rightButton0 = new JoystickButton(rightJoystick, 0);
  private final JoystickButton rightButton1 = new JoystickButton(rightJoystick, 1);
  private final JoystickButton rightButton2 = new JoystickButton(rightJoystick, 2);
  private final JoystickButton rightButton3 = new JoystickButton(rightJoystick, 3);
  private final JoystickButton rightButton4 = new JoystickButton(rightJoystick, 4);
  private final JoystickButton rightButton11 = new JoystickButton(rightJoystick, 11);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //rightButton3.whenPressed(driveMaxSpeed);
    leftButton3.whenPressed(runIntake);
    rightButton2.whenPressed(driveMaxSpeed);
    leftButton1.whenPressed(driveSlowSpeed);
    rightButton1.whenPressed(driveNormal);
    //rightButton4.whenPressed(spinCP);
    //leftButton4.whenPressed(setCPColor);
    rightButton11.toggleWhenPressed(runShooter);
    leftButton6.whenPressed(runIntake);
    rightButton3.whenHeld(runConveyor);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // An ExampleCommand will run in autonomous
  // return m_autoCommand;
  // }
}
