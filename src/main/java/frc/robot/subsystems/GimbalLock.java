/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GimbalLock extends SubsystemBase {
  /**
   * Creates a new ShooterPivot.
   */

  WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.SHOOTER_PIVOT_MOTOR);
  Encoder e = new Encoder(Constants.GIMBAL_LOCK_PORT1, Constants.GIMBAL_LOCK_PORT2);

  public GimbalLock() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void tiltUp(double n) {

    pivot.set(n);

  }

  public void tiltDown(double n) {

    pivot.set(-n);

  }

  public void stop() {

    pivot.set(0);

  }

  public Encoder getEncoder() {

    return e;

  }

}
