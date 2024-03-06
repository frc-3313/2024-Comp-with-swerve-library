// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CompoundCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.HandOffNoteCMD;
import frc.robot.commands.BasicCommands.IntakeNoteCMD;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartIntakeCMD extends SequentialCommandGroup {
  public static final Command cancel = null;

/** Creates a new QuickIntake. */
  public SmartIntakeCMD(Intake intake, Tilter tilter, Shooter shooter)
  {
  addCommands(
    new IntakeNoteCMD(intake, shooter),
    new HandOffNoteCMD(intake, tilter, shooter));
  }
}
