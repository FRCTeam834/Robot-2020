/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

<<<<<<< HEAD
package frc.robot.commands.autonomous.parallelgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.RunIntake;
=======
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.DriveForwardDistance;
>>>>>>> master
import frc.robot.commands.RunConveyorSensor;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
<<<<<<< HEAD
public class DriveAndIntake extends ParallelCommandGroup {
=======
public class DriveAndIntake extends ParallelRaceGroup {
>>>>>>> master
  /**
   * Creates a new DriveAndIntake.
   */
  public DriveAndIntake(double distance) {
    // Add your commands in the super() call, e.g.
<<<<<<< HEAD
    // super(new FooCommand(), new BarCommand());super();
=======
    // super(new FooCommand(), new BarCommand());
>>>>>>> master
    super(new DriveForwardDistance(distance), new RunConveyorSensor());
  }
}
