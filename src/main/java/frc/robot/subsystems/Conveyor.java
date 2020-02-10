/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot; 

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new Conveyor.
   */
  WPI_TalonSRX conveyorMotor = new WPI_TalonSRX(Constants.CONVEYOR_MOTOR);
  DigitalInput ballSensor = new DigitalInput(Constants.BALL_SENSOR_PORT);

  public Conveyor() {
    //setDefaultCommand(new StopConveyor());
  }

  @Override
  public void periodic() {
    // This method will be called once per schedule run
    //setDefaultCommand(new StopConveyor());
  }

  public boolean getSensor() {
    return ballSensor.get();
  }

  public void start(double speed) {
    conveyorMotor.set(speed);
  }

  public void stop() {
    conveyorMotor.set(0);
  }
}
