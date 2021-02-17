/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NetworkTableQuerier;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Processor2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoTestGroup1 extends SequentialCommandGroup {
  /**
   * Creates a new AutoTestGroup1.
   */
  public AutoTestGroup1(Drivetrain drive, Pneumatics shift, Processor2 process, NetworkTableQuerier ntables) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AutoDriveToBall(drive, shift, ntables, 5), new AutoRunProcessor(process, false, 5));
  }
}
