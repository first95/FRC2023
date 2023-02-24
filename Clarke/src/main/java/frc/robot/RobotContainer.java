// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ControlArm;
import frc.robot.commands.AutoHandoffCone;
import frc.robot.commands.AutoHandoffCube;
import frc.robot.commands.Autos;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.ExampleCommand;
import frc.robot.drivebase.AbsoluteDrive;
import frc.robot.drivebase.TeleopDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveBase drivebase = new SwerveBase();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  private final AbsoluteDrive absoluteDrive, closedAbsoluteDrive;
  private final TeleopDrive openFieldRel, openRobotRel, closedFieldRel, closedRobotRel;
  private final ControlArm controlArm;
  private final ControlIntake controlIntake;

  private AutoParser autoParser = new AutoParser(drivebase, arm, intake);
  private SendableChooser<CommandBase> driveModeSelector;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(OperatorConstants.DRIVE_CONTROLLER_PORT);
  CommandJoystick rotationController = new CommandJoystick(OperatorConstants.ANGLE_CONTROLLER_PORT);
  CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    absoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), true);

    closedAbsoluteDrive = new AbsoluteDrive(
      drivebase,
      // Applies deadbands and inverts controls because joysticks are back-right positive while robot
      // controls are front-left positive
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -rotationController.getX(),
      () -> -rotationController.getY(), false);

    openRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, true);
    
    closedRobotRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> false, false);
    
    openFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, true);

    closedFieldRel = new TeleopDrive(
      drivebase,
      () -> (Math.abs(driverController.getY()) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getY() : 0,
      () -> (Math.abs(driverController.getX()) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getX() : 0,
      () -> -driverController.getTwist(), () -> true, false);

    driveModeSelector = new SendableChooser<>();
    driveModeSelector.setDefaultOption("AbsoluteDrive", absoluteDrive);
    driveModeSelector.addOption("Field Relative", openFieldRel);
    driveModeSelector.addOption("Robot Relative", openRobotRel);
    driveModeSelector.addOption("Absolute (Closed)", closedAbsoluteDrive);
    driveModeSelector.addOption("Field Relative (Closed)", closedFieldRel);
    driveModeSelector.addOption("Robot Relative (Closed)", closedRobotRel);
    
    SmartDashboard.putData(driveModeSelector);
    drivebase.setDefaultCommand(absoluteDrive);

    SmartDashboard.putNumber("ANGLE", 0);
    SmartDashboard.putData("setAngle", new InstantCommand(() -> drivebase.setGyro(new Rotation2d(SmartDashboard.getNumber("ANGLE", 0)))).ignoringDisable(true));
    SmartDashboard.putData("sendAlliance", new InstantCommand(() -> drivebase.setAlliance(DriverStation.getAlliance())).ignoringDisable(true));

    //intake.setDefaultCommand(new ControlIntake(() -> operatorController.getLeftX(), () -> operatorController.getLeftY(), () -> (operatorController.getLeftTriggerAxis() - operatorController.getRightTriggerAxis()), intake));
    controlIntake = new ControlIntake(
      () -> operatorController.getLeftX(), 
      () -> operatorController.getLeftY(),
      () -> operatorController.getLeftTriggerAxis(),
      operatorController.y(),
      operatorController.b(),
      operatorController.a(),
      operatorController.x(),
      intake);

    controlArm = new ControlArm(
      () -> (Math.abs(Math.pow(operatorController.getRightY(), 3)) > OperatorConstants.RIGHT_Y_DEADBAND) 
              ? ((Math.pow(operatorController.getRightY(), 3)) / 3) 
              : 0
      , 
      operatorController.povDown(),   // STOW
      operatorController.povUp(),     // middle
      operatorController.povLeft(),   // low
      operatorController.povRight(),  // high
      new BooleanSupplier() { public boolean getAsBoolean() {return false;};}, // HANDOFF
      arm);   
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Hacky solutuon to stow arm on cube intake //
    // Could be causing a CommandScheduler loop overrun //
    operatorController.x().onTrue(new InstantCommand(() -> {arm.setPreset(ArmConstants.PRESETS.STOWED);}));

    operatorController.back().onTrue(new AutoHandoffCube(arm, intake));
    // operatorController.start().onTrue(new PrecisionAlign("Node1High", Alliance.Red, drivebase));
    operatorController.leftBumper().onTrue(new AutoHandoffCone(arm, intake));
    operatorController.rightBumper().onTrue(new InstantCommand(() -> {arm.toggleGrip();}));
    
    driverController.button(1).onTrue((new InstantCommand(drivebase::zeroGyro)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoParser.getAutoCommand();
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }
  public void prepareDriveForTeleop() {
    drivebase.setDefaultCommand(absoluteDrive);
    arm.setDefaultCommand(controlArm);
    intake.setDefaultCommand(controlIntake);
    absoluteDrive.setHeading(drivebase.getYaw());
    closedAbsoluteDrive.setHeading(drivebase.getYaw());
  }
  public void prepareDriveForAuto() {
    arm.setDefaultCommand(new RepeatCommand(new InstantCommand(() -> {}, arm))); // these feel so wrong
    intake.setDefaultCommand(new RepeatCommand(new InstantCommand(() -> {}, intake)));
    drivebase.setDefaultCommand(new RepeatCommand(new InstantCommand(() -> {}, drivebase)));
  }
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void setArmBrakes(boolean brake) {
    arm.setBreaks(brake);
  }

  public void parseAuto() {
    String autoText = SmartDashboard.getString("AutoCode", "");
    String parserOutput = "";
    try {
      parserOutput = autoParser.parse(autoText, DriverStation.getAlliance());
    } catch (Exception e) {
      parserOutput = e.getMessage();
      e.printStackTrace();
    } finally {
      SmartDashboard.putString("Compiler Message", parserOutput);
    }
  }
}
