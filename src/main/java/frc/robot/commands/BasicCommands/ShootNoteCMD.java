// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;

public class ShootNoteCMD extends Command {
  

  public Intake intake;
  public Elevator elevator;
  public Tilter tilter;
  public Shooter shooter;
  public Double shootAngle;
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
    if (shooter.hasNote()) 
    {
      tilter.GoToPosition(shootAngle);
      shooter.StartShooter(.6); 
    }
    else
    {
      return;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(shooter.IsShooterAboveRPM(2500) && tilter.atSetpoint())
    {
      shooter.StartFeeder(.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    shooter.StopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
