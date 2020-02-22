/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
*/
/*
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
/*
CANSparkMax leftDrive1 = new CANSparkMax(Constants.L_DRIVE1_PORT, CANSparkMax.MotorType.kBrushless);
CANSparkMax leftDrive2 = new CANSparkMax(Constants.L_DRIVE2_PORT, CANSparkMax.MotorType.kBrushless);
//CANSparkMax leftDrive3 = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
CANSparkMax rightDrive1 = new CANSparkMax(Constants.R_DRIVE1_PORT, CANSparkMax.MotorType.kBrushless);
CANSparkMax rightDrive2 = new CANSparkMax(Constants.R_DRIVE2_PORT, CANSparkMax.MotorType.kBrushless);
//CANSparkMax rightDrive3 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(leftDrive1, leftDrive2);
SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(rightDrive1, rightDrive2);

Joystick l = new Joystick(0);
Joystick r = new Joystick(1);

public DriveTrain() {

leftDriveGroup.setInverted(true);
}

@Override
public void periodic() {
// This method will be called once per scheduler run
setDefaultCommand(Robot.driveNormal);
}

public void leftDrive(double speed) {

leftDriveGroup.set(speed);

}

public void rightDrive(double speed) {

rightDriveGroup.set(speed);

}

public void setDrive(double lSpeed, double rSpeed) {

leftDriveGroup.set(lSpeed);
rightDriveGroup.set(rSpeed);

}

public void stop() {

leftDriveGroup.set(0);
rightDriveGroup.set(0);

}

public void setDriveWithMultiplier(double multiplier) {

setDrive(l.getY() * multiplier, r.getY() * multiplier);

}

}*/
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  CANSparkMax leftDrive1 = new CANSparkMax(Constants.L_DRIVE1_PORT, CANSparkMax.MotorType.kBrushless);
  CANSparkMax leftDrive2 = new CANSparkMax(Constants.L_DRIVE2_PORT, CANSparkMax.MotorType.kBrushless);
  //CANSparkMax leftDrive3 = new CANSparkMax(3, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightDrive1 = new CANSparkMax(Constants.R_DRIVE1_PORT, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightDrive2 = new CANSparkMax(Constants.R_DRIVE2_PORT, CANSparkMax.MotorType.kBrushless);
  //CANSparkMax rightDrive3 = new CANSparkMax(6, CANSparkMax.MotorType.kBrushless);

  SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(leftDrive1, leftDrive2);
  SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(rightDrive1, rightDrive2);

  DifferentialDrive dDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

  Joystick l = new Joystick(0);
  Joystick r = new Joystick(1);
  float compassHeading = Robot.navX.getCurrentDegrees();
  DifferentialDriveOdometry dDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(compassHeading));

  public DriveTrain() {

    leftDriveGroup.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Turnt: " + Rotation2d.fromDegrees(Robot.navX.getYaw()));

    dDriveOdometry.update(Rotation2d.fromDegrees(Robot.navX.getCurrentDegrees()), leftDrive1.getEncoder().getPosition(),
        rightDrive1.getEncoder().getPosition());

    //setDefaultCommand(Robot.driveNormal);

  }

  public void leftDrive(double speed) {

    leftDriveGroup.set(speed);

  }

  public void rightDrive(double speed) {

    rightDriveGroup.set(speed);

  }

  public void setDrive(double lSpeed, double rSpeed) {

    leftDriveGroup.set(lSpeed);
    rightDriveGroup.set(rSpeed);

  }

  public void stop() {

    leftDriveGroup.set(0);
    rightDriveGroup.set(0);

  }

  public void setDriveWithMultiplier(double multiplier) {

    setDrive(l.getY() * multiplier, r.getY() * multiplier);

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(leftDrive1.getEncoder().getVelocity(),
        rightDrive1.getEncoder().getVelocity());

  }

  public Pose2d getPose() {

    return dDriveOdometry.getPoseMeters();

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    leftDrive1.setVoltage(leftVolts);
    rightDrive1.setVoltage(-rightVolts);
    dDrive.feed();

  }

}
