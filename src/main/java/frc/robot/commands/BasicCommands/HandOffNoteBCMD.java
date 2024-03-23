// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class HandOffNoteBCMD extends Command 
{

  public Intake intake;
  public Tilter tilter;
  public Shooter shooter;
  public Timer timer;

  public HandOffNoteBCMD(Intake m_Intake, Tilter m_Tilter, Shooter m_Shooter) {
    
    intake = m_Intake;
    tilter = m_Tilter;
    shooter = m_Shooter;
    addRequirements(tilter, shooter, intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    timer = new Timer();
    timer.reset();
    
    tilter.GoToPosition(Constants.Tilter.handOffPosition);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(tilter.atSetpoint())
    {
      intake.RunIntake(.6);
      shooter.StartFeeder(.2);
      timer.start();
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    shooter.MoveFeederDistance(500);
    tilter.GoToPosition(Constants.Tilter.stowPosition);
    shooter.StopFeeder();
    intake.StopIntake();
    shooter.MoveFeederDistance(-3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(timer.hasElapsed(0.2))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
