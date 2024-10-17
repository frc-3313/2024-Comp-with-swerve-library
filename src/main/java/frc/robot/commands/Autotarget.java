// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class Autotarget extends Command {

  private final Limelight limelight;
  private final Tilter tilter;
  private final SwerveSubsystem driveSubsystem;
  private final CommandXboxController driveController;
  private final Shooter shooter;
  private final PIDController steeringPID;
  private double targetDistance;
  private double minimumShootDis = 120; //12 feet
  private double kP = 0.05; // Proportional gain
  private double kI = 0.1; // Integral gain
  private double kD = 0.00; // Derivative gain
  boolean firsttime = true;
  private double angle = 30; // angle of the goal from the shooter
  private int speakerID;

  /** Creates a new Autotarget. */
  public Autotarget(Limelight limelight, SwerveSubsystem drive,Tilter tilter, CommandXboxController controller, Shooter shooter) {
    this.limelight = limelight;
    this.driveSubsystem = drive;
    this.tilter = tilter;
    this.driveController = controller;
    this.shooter = shooter;
    this.steeringPID = new PIDController(kP, kI, kD);
    addRequirements(limelight, drive, tilter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() 
  {
    if(DriverStation.getAlliance().get() == Alliance.Red) {speakerID = 4;}
    else {speakerID = 7;}

    steeringPID.setP(kP);
    steeringPID.setD(kD);
    steeringPID.setI(kI);

    SmartDashboard.putNumber("kp", kP);
    SmartDashboard.putNumber("kd", kD);
    SmartDashboard.putNumber("kI", kI);

    shooter.SetShooterSpeed(0.65);
  }

  @Override
  public void execute() 
  {
    double translationX = driveController.getRawAxis (1);
    double translationY = driveController.getRawAxis (0);
    double rotation = driveController.getRightX();

    SmartDashboard.putBoolean("target Valid", limelight.isTargetValid());
    SmartDashboard.putNumber("lined up", limelight.getTX());
    SmartDashboard.putNumber("Distance to April Tag", targetDistance);

    
    if(firsttime)
    {
      SmartDashboard.putNumber("kP", kP);
      SmartDashboard.putNumber("kD", kD);  
      SmartDashboard.putNumber("kI", kI);
      firsttime = false; 
    }
    kP = SmartDashboard.getNumber("kP", kP);
    kD = SmartDashboard.getNumber("kD", kD);
    kI = SmartDashboard.getNumber("kI", kI);

    steeringPID.setP(kP);
    steeringPID.setD(kD);
    steeringPID.setI(kI);  

    if (limelight.isTargetValid())
    { 
      int detectedTagID = limelight.getAprilTagID();
      if (detectedTagID == speakerID) 
      {
        double tx = limelight.getTX();
        targetDistance = limelight.GetDistanceInches();
        // Use PID to calculate steering adjustment
        double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset
            
        // Convert steering adjustment to swerve drive inputs
        double autoRotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
            
        // Drive the swerve robot
        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX)* driveSubsystem.getMaximumVelocity(),
                            driveSubsystem.powerof2(-translationY)* driveSubsystem.getMaximumVelocity()),
                            driveSubsystem.powerof2(autoRotation)* driveSubsystem.getMaximumAngularVelocity(),
                        true);

        // Tells the tilter to go to angle to get to the goal
        angle = limelight.CalculateShootAngle(targetDistance);
        SmartDashboard.putNumber("AngleToGoal", angle);
        tilter.GoToPosition(angle +140);

        //NEW UNTESTED CODE ----------- SHOOT AFTER TARGET LOCATED
        //once at full speed run feeter
        //after the note is no longer in the shooter 
        //set isfinished to true
        if(targetDistance <= minimumShootDis && limelight.getTX() < 0.3 && limelight.getTX() > -03  && tilter.atSetpoint())
        {
          shooter.SetShooterSpeed(.80);
          if(shooter.IsShooterAboveRPM())
          {
            shooter.StartFeeder();
          }
        }
      } 
      else 
      {
        // Stop if target is not correct
        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX) * driveSubsystem.getMaximumVelocity(),
                            driveSubsystem.powerof2(-translationY)* driveSubsystem.getMaximumVelocity()),
                            driveSubsystem.powerof2(-rotation)* driveSubsystem.getMaximumAngularVelocity(),
                        true);
      }
    } 
    else 
    {
      // Stop if no target is visible
      driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX)* driveSubsystem.getMaximumVelocity(),
                        driveSubsystem.powerof2(-translationY)* driveSubsystem.getMaximumVelocity()),
                        driveSubsystem.powerof2(-rotation)* driveSubsystem.getMaximumAngularVelocity(),
                    true);
    }
  }

  // @Override
  // public boolean isFinished() 
  // {
  //   return !shooter.hasNote();
  // }

  @Override
  public void end(boolean interrupted) 
  {
      shooter.StopAllMotors();
      tilter.GoToPosition(Constants.Tilter.stowPosition);
  }
}
