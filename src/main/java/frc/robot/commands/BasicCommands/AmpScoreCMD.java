// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class AmpScoreCMD extends Command {
  public Intake intake;
  public Elevator elevator;
  public Tilter tilter;
  public Shooter shooter;
  /** Creates a new AmpScoreCMD. */
  public AmpScoreCMD(Intake m_Intake, Elevator m_Elevator, Tilter m_Tilter, Shooter m_Shooter)
  {
    addRequirements(intake, elevator, tilter, shooter);
    intake = m_Intake;
    elevator = m_Elevator;
    tilter = m_Tilter;
    shooter = m_Shooter;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
  // tilter.GoToPosition(Constants.Tilter.ampPosition); 
  // shooter.StartShooter();
    if (shooter.hasNote()) 
    {
      elevator.GoToHeight(Constants.Elevator.elvAmpPosition);
      tilter.GoToPosition(Constants.Tilter.ampPosition);
      shooter.StartShooter(.3);
    } 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if(shooter.hasNote())
    {
      shooter.StartFeeder(.5);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
