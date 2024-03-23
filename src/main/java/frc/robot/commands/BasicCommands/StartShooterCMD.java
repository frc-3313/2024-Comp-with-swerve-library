package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;


public class StartShooterCMD extends InstantCommand {

  public Tilter tilter;
  public Shooter shooter;
  public Double shootAngle;
  /** Creates a new StartShooterCMD. */
  public StartShooterCMD(Shooter m_shooter, Tilter m_tilter, Double m_shootAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
    tilter = m_tilter;
    addRequirements(tilter, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    tilter.GoToPosition(shootAngle);
    shooter.StartShooter(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
