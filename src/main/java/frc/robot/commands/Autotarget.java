// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autotarget extends Command {

  private final Limelight limelight;
  private final SwerveSubsystem swerveDrive;
  private final Shooter shooter;
  private final Tilter tilter;
  private final CommandXboxController driveController;
  private final int targetTagID;
  private double targetDistance;

  private static final double kP = 0.02;

  /** Creates a new Autotarget. */
  public Autotarget(Limelight limelight, SwerveSubsystem swerveDrive, Shooter shooter, Tilter tilter, CommandXboxController controller, int tagID) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    this.shooter = shooter;
    this.tilter = tilter;
    this.driveController = controller;
    this.targetTagID = tagID;
    addRequirements(limelight, swerveDrive, shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationX = driveController.getRawAxis (1);
    double translationY = driveController.getRawAxis (0);

    double rotationalSpeed = 0.1;
    SmartDashboard.putBoolean("target Valid Auto", limelight.isTargetValid());
    SmartDashboard.putNumber("Distance to April Tag", targetDistance);
    SmartDashboard.putNumber("targetTagID", targetTagID);

    if (limelight.isTargetValid()){
      
      int detectedTagID = limelight.getAprilTagID();
      if (detectedTagID == targetTagID) {
        double horizontalOffest = limelight.getTX();
        rotationalSpeed = kP *horizontalOffest;
        targetDistance = limelight.GetDistanceInches();
      }
    }
    

     swerveDrive.drive(new Translation2d(swerveDrive.powerof2(translationX) * .95,
                            swerveDrive.powerof2(translationY) * .95),
                            swerveDrive.powerof2(rotationalSpeed),
                        true);
    
    // if (distanceToTag < targetDistance){
    //   double shootAngle = CalculateShootAngle(distanceToTag);
    //   shooter.setShooterAngle(shootAngle);
    // }
    // else {
    //   //FIX add normal drive code here
    // }

  }

  // Called once the command ends or is interrupted.
  private void rotateSwerveToTarget(double horizontalOffest) {
    if (Math.abs(horizontalOffest) > 1.0){
      
      //swerveDrive.rotate(rotationalSpeed);
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
