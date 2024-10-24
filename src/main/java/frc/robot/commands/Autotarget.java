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
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
public class Autotarget extends Command {

  private final Tilter tilter;
  private final SwerveSubsystem driveSubsystem;
  private final CommandXboxController driveController;
  private final Shooter shooter;
  private final PIDController steeringPID;
  private double targetDistance;
  private double minimumShootDis = 120; //12 feet
  private double kP = 0.04; // Proportional gain
  private double kI = 0.05; // Integral gain, pervios i was .1
  private double kD = 0.0; // Derivative gain
  boolean firsttime = true;
  private double angle = 30; // angle of the goal from the shooter
  private Double speakerID;
  private double cameraAngleOffset = 41; //Should be angle that the Limelight is facing in degrees, but in actuality, it is not. 41 just seems to work.
  private double goalHeight = 78; //Height of the speaker in inches
  private double limelightLensHeight = 8.25; //Limelight height in inches
  //private double shootHeightOffset = 10; //Height from Limelight to Shooter pivot in inches
  //private double shootDistanceOffset = 9; //Distance from Limelight to Shooter pivot in inches
  //private double fineDistanceAdjustment = 12; // Distance away from the wall in inches, to adjust for note drop

  /** Creates a new Autotarget. */
  public Autotarget(SwerveSubsystem drive,Tilter tilter, CommandXboxController controller, Shooter shooter) {
    this.driveSubsystem = drive;
    this.tilter = tilter;
    this.driveController = controller;
    this.shooter = shooter;
    this.steeringPID = new PIDController(kP, kI, kD);
    addRequirements(drive, tilter);
  }

  @Override
  public void initialize() 
  {
    if(DriverStation.getAlliance().get() == Alliance.Red) {speakerID = 4.0;}
    else {speakerID = 7.0;}

    steeringPID.setP(kP);
    steeringPID.setD(kD);
    steeringPID.setI(kI);

    shooter.SetShooterSpeed(0.8);  
  }

  @Override
  public void execute() 
  {
    double translationX = driveController.getRawAxis (1);
    double translationY = driveController.getRawAxis (0);
    double rotation = driveController.getRightX();

    SmartDashboard.putBoolean("target Valid", limelight.isTargetValid());
    SmartDashboard.putNumber("getTX", limelight.getTX());

    if (LimelightHelpers.getTV(Constants.Limelight.FRONT))
    { 
      double detectedTagID = LimelightHelpers.getFiducialID(Constants.Limelight.FRONT);
      if (detectedTagID == speakerID) 
      {
        double tx = LimelightHelpers.getTX(Constants.Limelight.FRONT);
        targetDistance = GetDistanceInches();
        // Use PID to calculate steering adjustment
        double steeringAdjust = steeringPID.calculate(tx, 0.0); // Aim for zero offset
            
        // Convert steering adjustment to swerve drive inputs
        double autoRotation = MathUtil.clamp(steeringAdjust, -1.0, 1.0); // Ensure rotation speed is within limits
            
        // Drive the swerve robot

        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX)* driveSubsystem.swerveDrive.getMaximumVelocity(),
                            driveSubsystem.powerof2(-translationY)* driveSubsystem.swerveDrive.getMaximumVelocity()),
                            driveSubsystem.powerof2(autoRotation)* driveSubsystem.swerveDrive.getMaximumAngularVelocity(),
                        true);

        // Tells the tilter to go to angle to get to the goal
        angle = CalculateShootAngle(targetDistance);
        SmartDashboard.putNumber("AngleToGoal", angle);
        tilter.GoToPosition(angle +140);

        //NEW UNTESTED CODE ----------- SHOOT AFTER TARGET LOCATED
        //once at full speed run feeter
        //after the note is no longer in the shooter 
        //set isfinished to true

        if(targetDistance <= minimumShootDis && LimelightHelpers.getTX(Constants.Limelight.FRONT) < 3 && LimelightHelpers.getTX(Constants.Limelight.FRONT) > -3  && tilter.atSetpoint())
        {
          if(shooter.IsShooterAboveRPM())
          {
            shooter.StartFeeder();
          }
        }
      } 
      else 
      {
        // Stop if target is not correct

        driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX) * driveSubsystem.swerveDrive.getMaximumVelocity(),
                            driveSubsystem.powerof2(-translationY)* driveSubsystem.swerveDrive.getMaximumVelocity()),
                            driveSubsystem.powerof2(-rotation)* driveSubsystem.swerveDrive.getMaximumAngularVelocity(),
                        true);
      }
    } 
    else 
    {
      // Stop if no target is visible
      driveSubsystem.drive(new Translation2d(driveSubsystem.powerof2(-translationX)* driveSubsystem.swerveDrive.getMaximumVelocity(),
                        driveSubsystem.powerof2(-translationY)* driveSubsystem.swerveDrive.getMaximumVelocity()),
                        driveSubsystem.powerof2(-rotation)* driveSubsystem.swerveDrive.getMaximumAngularVelocity(),
                    true);
    }
  }

  @Override
  public boolean isFinished() 
  {
    return !shooter.hasNote();
  }

  @Override
  public void end(boolean interrupted) 
  {
      shooter.StopAllMotors();
      tilter.GoToPosition(Constants.Tilter.stowPosition);
  }
  public double CalculateShootAngle(double distance)
  {
    return ((-0.278)*distance)+(66.62);
  }
  public double GetDistanceInches()
  {
    double angleToGoalDegrees = GetYAngle();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }
  public double GetYAngle()
  {
    return (LimelightHelpers.getTY(Constants.Limelight.FRONT) + cameraAngleOffset);
  }
}

