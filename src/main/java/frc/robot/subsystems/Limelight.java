// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
  
  private NetworkTable limelightTable;
  private double tx, ty, ta;
  private double tv;
  //camera offsets
  private double cameraAngleOffset = 41;
  private double fineAngleAdjustment = 0;
  private double goalHeight = 78;
  private double limelightLensHeight = 8.25; //Limelight Hight inches 19.5
  private double shootHeightOffset = 10; //Shooter to limelight
  private double shootDistanceOffset = 9; //lime light to shooter idealy <12
  private double fineDistanceAdjustment = 0; // distance way from the wall, to adjust for note drop

  public Limelight() 
  {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-mech");
  }

  @Override
  public void periodic() 
  {
    //
    tx = limelightTable.getEntry("tx").getDouble(0);
    ty = limelightTable.getEntry("ty").getDouble(0);
    ta = limelightTable.getEntry("ta").getDouble(0);
    tv = limelightTable.getEntry("tv").getDouble(0);

    SmartDashboard.putBoolean("TargetLocket", isTargetValid());
    SmartDashboard.putNumber("DistanceToTarget", GetDistanceInches());
    SmartDashboard.putNumber("AngleToTarget", GetYAngle());
    SmartDashboard.putNumber("Tv", tv);

  }

  public double GetYAngle()
  {
    return (getTY() + cameraAngleOffset);
  }

  public double GetDistanceInches()
  {
    double angleToGoalDegrees = GetYAngle();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180);
    double distanceFromLimelightToGoalInches = (goalHeight - limelightLensHeight) / Math.tan(angleToGoalRadians);
    return distanceFromLimelightToGoalInches;
  }

  public double CalculateShootAngle()
  {
    //arctan, tan^-1, or atan will find the angle of the height/distance in radians, and dividing by PI/180 will convert to degrees
    double shootAngle = (Math.atan((goalHeight - shootHeightOffset) / (GetDistanceInches() + shootDistanceOffset - fineDistanceAdjustment))) / (Math.PI /180);
    return shootAngle;
  }

  public double getTX() {
    return tx;
  }
  public double getTY() {
    return ty;
  }
  public double getTA() {
    return ta;
  }

  public boolean isTargetValid() {
    return .75 <= tv; 
  }
  public void setLEDMode(int mode)  { 
      limelightTable.getEntry("ledMode").setNumber(mode);
  }

  public int getAprilTagID() {
    return (int) limelightTable.getEntry("tid").getDouble(-1);
  }
}