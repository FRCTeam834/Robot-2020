/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StopConveyor;

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new Conveyor.
   */
  WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(Constants.CONVEYOR_MOTOR);

  public Conveyor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per schedule run
    setDefaultCommand(new StopConveyor());
  }

  public void start() {
    conveyorMotor.set(.75);
  }

  public void stop() {
    conveyorMotor.set(0);
  }
}
