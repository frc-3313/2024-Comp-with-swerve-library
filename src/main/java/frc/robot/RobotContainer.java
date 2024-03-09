// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasicCommands.AmpScoreCMD;
import frc.robot.commands.BasicCommands.DeployIntakeCMD;
import frc.robot.commands.BasicCommands.ElevatorGoPosition;
import frc.robot.commands.BasicCommands.HandOffNoteBCMD;
import frc.robot.commands.BasicCommands.JogIntake;
import frc.robot.commands.BasicCommands.JogShooter;
import frc.robot.commands.BasicCommands.ReturnToNormal;
import frc.robot.commands.BasicCommands.ShootNoteCMD;
import frc.robot.commands.BasicCommands.SorceIntakeCMD;
import frc.robot.commands.BasicCommands.ZeroGyro;
import frc.robot.commands.CompoundCommands.SmartIntakeCMD;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tilter;
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
    NamedCommands.registerCommand("SmartIntakeCMD", new SmartIntakeCMD(intake, tilter, shooter));
    NamedCommands.registerCommand("HandOffNoteBCMD", new HandOffNoteBCMD(intake, tilter, shooter));
    NamedCommands.registerCommand("ShootFromStage", new ShootNoteCMD(tilter, shooter, Constants.Tilter.shootFromStage));
    NamedCommands.registerCommand("ShootFromSpeaker", new ShootNoteCMD(tilter, shooter, Constants.Tilter.shootFromSpeaker));
    NamedCommands.registerCommand("ShootAmp", new AmpScoreCMD(intake, elevator, tilter, shooter));
    NamedCommands.registerCommand("ReturnToNormal", new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));
    NamedCommands.registerCommand("DeployIntakeCMD", new DeployIntakeCMD(intake, shooter));
    
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   () -> driverXbox.y().getAsBoolean(),
                                                                   () -> driverXbox.a().getAsBoolean(),
                                                                   () -> driverXbox.x().getAsBoolean(),
                                                                   () -> driverXbox.b().getAsBoolean());

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
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    //Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
    //    () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    //    () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    //    () -> driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : closedAbsoluteDriveAdv);
    
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.a().onTrue((new InstantCommand(drivebase::zeroGyro)));
    driverXbox.x().onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
//    new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    //oporator buttons 
    driverXbox.b().onTrue(new ZeroGyro(drivebase));
    //intake from sorce=d pad down
    manipulatorXbox.povDown().whileTrue(new SorceIntakeCMD(intake, elevator, tilter, shooter));
    //manipulatorXbox.povDown().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter));;
    //intake=A 
    manipulatorXbox.a().toggleOnTrue(new SmartIntakeCMD(intake, tilter, shooter));
  
    //handoff = x
    manipulatorXbox.x().onTrue(new HandOffNoteBCMD(intake, tilter, shooter));
    
    //shoot high =y
    manipulatorXbox.povUp().whileTrue(new ShootNoteCMD(tilter, shooter, Constants.Tilter.shootFromStage));
    manipulatorXbox.povUp().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));;

    manipulatorXbox.y().whileTrue(new ShootNoteCMD(tilter, shooter, Constants.Tilter.shootFromSpeaker));
    manipulatorXbox.y().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));;
    
    //amp=b
    manipulatorXbox.b().whileTrue(new AmpScoreCMD(intake, elevator, tilter, shooter));
    manipulatorXbox.b().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));;

    //Jog commands
    manipulatorXbox.rightBumper().onTrue(new JogIntake(intake, false));
    manipulatorXbox.rightTrigger(.5).onTrue(new JogIntake(intake, true));
    manipulatorXbox.leftBumper().onTrue(new JogShooter(shooter, false));
    manipulatorXbox.leftTrigger(.5).onTrue(new JogShooter(shooter, true)); 
   
    manipulatorXbox.start().onTrue(new ElevatorGoPosition(elevator, Constants.Elevator.elvAmpPosition, tilter));
    manipulatorXbox.back().onTrue(new ElevatorGoPosition(elevator, Constants.Elevator.elvBottomPosition, tilter));

    //pass=povLeft
    manipulatorXbox.povLeft().whileTrue(new ShootNoteCMD(tilter, shooter, Constants.Tilter.passPosition));
    manipulatorXbox.povLeft().onFalse(new ReturnToNormal(intake, elevator, tilter, shooter).withTimeout(1));;

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
