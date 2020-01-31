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
import frc.robot.commands.MoveTurret;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */

  // WPI_TalonSRX turret = new WPI_TalonSRX(9);
  // Encoder twist = new Encoder(1, 2);

  public Turret() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new MoveTurret());
  }

  public void runForward() {

    // turret.set(.25);

  }

  public void runBackward() {

    // turret.set(-.25);

  }

  public void stop() {

    // turret.set(0);

  }

}
