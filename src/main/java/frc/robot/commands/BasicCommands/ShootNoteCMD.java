// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import edu.wpi.first.wpilibj.Timer;

public class ShootNoteCMD extends Command {
  
  public Tilter tilter;
  public Shooter shooter;
  public Double shootAngle;
  public Timer timer;

  public ShootNoteCMD(Tilter m_Tilter, Shooter m_Shooter, Double m_shootAngle) {
    // Use addRequirements() here to declare subsystem dependencies.

    tilter = m_Tilter;
    shooter = m_Shooter;
    shootAngle = m_shootAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

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
    shooter.StopAllMotors();
    tilter.GoToPosition(Constants.Tilter.stowPosition);
    shooter.StopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(shooter.IsShooterAboveRPM(2500) && tilter.atSetpoint())
    {
      shooter.MoveFeederDistance(300);
      shooter.FeederDone();
      return true;
    }
    else
    {
      return false;
    }
  }
}
