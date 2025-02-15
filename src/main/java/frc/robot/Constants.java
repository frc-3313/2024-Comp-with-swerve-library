// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  //// SUBSYSTEM CONSTANTS ////
  public static final class Intake
  {
      //INTAKE
      
      
      public static final int IntakeMotor_ID = 23;
      
      DigitalInput IntakeBeam = new DigitalInput(0);
  }   
  public static final class Elevator
  {
      //ELEVATOR
      public static final int ElevatorMotor1_ID = 22;
      public static final int Elevator_ENCODER_ID = 2;
      public static final int ElevatorMotor2_ID = 28;
      public static final double elvBottomPosition = 0; 
      public static final double climbLowPosition = 0;
      public static final double elvAmpPosition = 65;
      public static final double SorceIntakePosition = 10; 
      public static final double elvTrapPosition = 74; 
      public static final double elvHighest = 74;
  }
  public static final class Shooter
  {
      //SHOOTER
      public static final int ShooterMotor1_ID = 24;
      public static final int FeederMotor_ID = 26;
      public static final double FeederBackDistance = -3;
      public static final double fastShotSpeed = .7;
      public static final double midShotSpeed = .6;
      public static final double ampShotSpeed = .45;
      
      public static final double sourceIntakeSpeed = -.5;
  }
  public static final class Tilter
  {   
      //TILTER
      public static final int TilterMotor_ID = 27;
      public static final double zeroDegrees = 150;
      public static final double stowPosition = 140;  //255; 
      public static final double handOffPosition = 195;//310; this one
      public static final double shootFromStage = 190; //305;
      public static final double shootFromSpeaker = 199; //322;
      public static final double ampPosition = 120; //240;
      public static final double passLowPosition = 140; //280;
      public static final double MaxLow = 115; 
      public static final double MaxHigh = 205;
  }
  public static final class Limelight
  {
    public static final String FRONT = "limelight-mech";
  }
}