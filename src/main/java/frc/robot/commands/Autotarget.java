// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.Limelight;
import frc.robot.subsystem.SwerveSubsystem;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Tilter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autotarget extends CommandBase {

  private final Limelight limelight;
  private final SwerveSubsystem swerveDrive;
  private final Shooter shooter;
  private final Tilter tilter;
  private final Joystick driveContoller;
  private final int targetTagID = 8;
  private final double targetDistance;

  private static final double kP = 0.02;

  /** Creates a new Autotarget. */
  public Autotarget(Limelight limelight, SwerveSubsystem swerveDrive, Shooter shooter, Tilter tilter, Joystick controller, int tagID) {
    this.limelight = limelight;
    this.swerveDrive = SwerveSubsystem;
    this.shooter = Shooter;
    this.tilter = Tilter;
    this.controller = controller;
    this.targetTagID = tagID;
    addRequirements(Limelight, SwerveSubsystem, Shooter)

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationX = controller.getRawAxis (1);
    double translationY = controller.getRawAxis (0);

    double rotationalSpeed = 0.0;

    if (limelight.hasValidTarget()){
      int detectedTagID = limelight.getAprilTagID();
      if (detectedTagID == targetTagID) {
        double horizontalOffest = limelight.getTx();
        rotationalSpeed = kP *horizontalOffest;
        SmartDashboard.putNumber("Distance to April Tag", distanceToTag);
      }
    }
    
    swerveDrive.drive(translationX, translationY, rotationalSpeed);

    double[] pose = limelight.getAprilTagPose();
    double distanceToTag = pose[2];
    
    // if (distanceToTag > targetDistance){
    //   double shootAngle = CalculateShootAngle(distanceToTag);
    //   shooter.setShooterAngle(shootAngle);
    // }
    else {
      //FIX add normal drive code here
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  private void rotateSwerveToTarget(double horizontalOffest) {
    if (Math.abs(horizontalOffest) > 1.0){
      
      swerveDrive.rotate(rotationalSpeed);
    }
  }

  private double CalculateShootAngle(double distance){
    return 45 - (distance * 0.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
