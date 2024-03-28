// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasicCommands.ClimbCMD;
import frc.robot.commands.BasicCommands.ElevatorGoPosition;
import frc.robot.commands.BasicCommands.IntakeNoteCMD;
import frc.robot.commands.BasicCommands.JogIntake;
import frc.robot.commands.BasicCommands.JogShooter;
import frc.robot.commands.BasicCommands.PrimeShootCMD;
import frc.robot.commands.BasicCommands.ReturnToNormal;
import frc.robot.commands.BasicCommands.ShootNoteCMD;
import frc.robot.commands.BasicCommands.SmartShootNoteCMD;
import frc.robot.commands.BasicCommands.SorceIntakeCMD;
import frc.robot.commands.BasicCommands.ZeroGyro;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
import frc.robot.subsystems.Limelight;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final Tilter tilter = new Tilter();
  private final Shooter shooter = new Shooter();
  private final Limelight limelight = new Limelight();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private CommandXboxController manipulatorXbox = new CommandXboxController(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private CommandXboxController driverXbox = new CommandXboxController(0);
  private SendableChooser<Command> auto_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Register Named Commands
    NamedCommands.registerCommand("ShootFromStage", new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.fastShotSpeed, Constants.Tilter.shootFromStage, Constants.Elevator.elvBottomPosition));
    NamedCommands.registerCommand("ShootFromSpeaker", new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.fastShotSpeed, Constants.Tilter.shootFromSpeaker, Constants.Elevator.elvBottomPosition));
    NamedCommands.registerCommand("ShootAmp", new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.ampShotSpeed, Constants.Tilter.ampPosition, Constants.Elevator.elvAmpPosition));
    NamedCommands.registerCommand("ReturnToNormal", new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));
    
    // Configure the trigger bindings
    configureBindings();


    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    //Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
    //   () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //    () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //    () -> driverXbox.getRightX(),
    //    () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translati on
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocityNew = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(), 
        () -> driverXbox.x().getAsBoolean(),
        () -> driverXbox.y().getAsBoolean(),
        () -> driverXbox.b().getAsBoolean(),
        () -> driverXbox.a().getAsBoolean()).withName("DefaultDriveCommand");         
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX()).withName("DefaultDriveCommand2");
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityNew);
    
    auto_chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(auto_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //*** Driver ***//
    //hold to raise elevator and on release it will climb = right bumper
    driverXbox.rightBumper().whileTrue(new ClimbCMD(elevator, tilter));

    //zero gyro = start button
    driverXbox.start().onTrue(new ZeroGyro(drivebase));

    //TODO
    //Add Button to rotate towards speaker 
    //add button to rotate towards amp
    //add button to face source

    //driverXbox.a().onTrue((new InstantCommand(drivebase::zeroGyro)));
    //driverXbox.x().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    //driverXbox.b().whileTrue(
    //    Commands.deferredProxy(() -> drivebase.driveToPose(
    //                               new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                          ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    //------------------------------------- Manipulator -------------------------------------//

    //intake from sorce
    // manipulatorXbox.povDown().whileTrue(new SorceIntakeCMD(intake, elevator, tilter, shooter));
    // manipulatorXbox.povDown().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter));
    manipulatorXbox.axisGreaterThan(4, 0.5).onTrue(new SorceIntakeCMD(intake, elevator, tilter, shooter));
    
    // //low pass = right stick up
    // manipulatorXbox.axisGreaterThan(5, 0.5).onTrue(new PrimeShootCMD(tilter, shooter, elevator, 0.3, Constants.Tilter.stowPosition, Constants.Elevator.elvBottomPosition));
    // manipulatorXbox.axisLessThan(5, 0.5).onFalse(new SequentialCommandGroup(
    //       new ShootNoteCMD(tilter, shooter, elevator),
    //       new ReturnToNormal(intake, elevator, tilter, shooter)));
  
    // //high pass = right stick left
    // manipulatorXbox.axisGreaterThan(4, 0.5).onTrue(new PrimeShootCMD(tilter, shooter, elevator, 0.3, Constants.Tilter.shootFromSpeaker, Constants.Elevator.elvBottomPosition));
    // manipulatorXbox.axisLessThan(4, 0.5).onFalse(new SequentialCommandGroup(
    //       new ShootNoteCMD(tilter, shooter, elevator),
    //       new ReturnToNormal(intake, elevator, tilter, shooter)));

    //intake=A 
    manipulatorXbox.a().onTrue(new SequentialCommandGroup(new IntakeNoteCMD(intake, shooter, tilter),new ReturnToNormal(intake, elevator, tilter, shooter)));
  
    //scoer amp = B
    manipulatorXbox.b().onTrue(new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.ampShotSpeed, Constants.Tilter.ampPosition, Constants.Elevator.elvAmpPosition));
    manipulatorXbox.b().onFalse(new SequentialCommandGroup(
      new ShootNoteCMD(tilter, shooter, elevator),
      new ReturnToNormal(intake, elevator, tilter, shooter)));

    //shoot from speaker = Y
    manipulatorXbox.y().onTrue(new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.fastShotSpeed, Constants.Tilter.shootFromSpeaker, Constants.Elevator.elvBottomPosition));
    manipulatorXbox.y().onFalse(new SequentialCommandGroup(
      new ShootNoteCMD(tilter, shooter, elevator),
      new ReturnToNormal(intake, elevator, tilter, shooter)));

    // manipulatorXbox.rightTrigger(0.5).whileTrue(new SequentialCommandGroup(
    //   new SmartShootNoteCMD(tilter, shooter, elevator, limelight),
    //   new ReturnToNormal(intake, elevator, tilter, shooter)));
        
    //shoot from the stage = D pad up
    manipulatorXbox.povUp().onTrue(new PrimeShootCMD(tilter, shooter, elevator, Constants.Shooter.fastShotSpeed, Constants.Tilter.shootFromStage, Constants.Elevator.elvBottomPosition));
    manipulatorXbox.povUp().onFalse(new SequentialCommandGroup(
      new ShootNoteCMD(tilter, shooter, elevator),
      new ReturnToNormal(intake, elevator, tilter, shooter)));
          
    //return to normal = x
    manipulatorXbox.x().onTrue(new ReturnToNormal(intake, elevator, tilter, shooter));

    //Jog commands
     //manipulatorXbox.rightBumper().onTrue(new JogIntake(intake, false));
     //manipulatorXbox.rightTrigger(.5).onTrue(new JogIntake(intake, true));
     //manipulatorXbox.leftBumper().onTrue(new JogShooter(shooter, false));
     //manipulatorXbox.leftTrigger(.5).onTrue(new JogShooter(shooter, true)); 
   
    // manipulatorXbox.start().onTrue(new ElevatorGoPosition(elevator, Constants.Elevator.elvAmpPosition, tilter));
    // manipulatorXbox.back().onFalse(new ElevatorGoPosition(elevator, Constants.Elevator.elvBottomPosition, tilter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return auto_chooser.getSelected();

  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
