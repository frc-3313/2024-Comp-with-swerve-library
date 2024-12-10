// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class ReturnToNormal extends InstantCommand {
  public Intake intake;
  public Elevator elevator;
  public Tilter tilter;
  public Shooter shooter;
  public Timer timer;
  /** Creates a new AmpScoreCMD. */
  public ReturnToNormal(Intake m_Intake, Elevator m_Elevator, Tilter m_Tilter, Shooter m_Shooter){
    intake = m_Intake;
    elevator = m_Elevator;
    tilter = m_Tilter;
    shooter = m_Shooter;
    addRequirements(intake, elevator, tilter, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    tilter.GoToPosition(Constants.Tilter.stowPosition);
    elevator.GoToHeight(Constants.Elevator.elvBottomPosition);
    shooter.StopAllMotors();
    intake.StopIntake();
  }
}
