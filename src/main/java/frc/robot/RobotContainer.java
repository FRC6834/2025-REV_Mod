// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.AutoAlign;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    
    //Setting up named commands for PathPlanner
    NamedCommands.registerCommand("RunAlgaeIntake", m_algaeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("ReverseAlgaeIntake", m_algaeSubsystem.reverseIntakeCommand());
    NamedCommands.registerCommand("StowAlgae", m_algaeSubsystem.stowCommand());
    NamedCommands.registerCommand("ElevatorToFeederStation", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    NamedCommands.registerCommand("ElevatorToLevel1", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    NamedCommands.registerCommand("ElevatorToLevel2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    NamedCommands.registerCommand("ElevatorToLevel3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    NamedCommands.registerCommand("ElevatorToLevel4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    NamedCommands.registerCommand("RunCoralIntake", m_coralSubSystem.runIntakeCommand());
    NamedCommands.registerCommand("ReverseCoralIntake", m_coralSubSystem.reverseIntakeCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    // Default position can be changed in Algae Subsystem code
    // Default position is currently the "stowed" state
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    /******************************************************************************************
     * SPECIAL CONTROLS
     *******************************************************************************************/

    // Left Stick Button -> Set swerve to X position (prevent being pushed)
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());

    // Back Button -> Zero swerve heading
    m_driverController.back().onTrue(m_robotDrive.zeroHeadingCommand());

    // Left Bumper -> Run command to auto-align robot chassis according to AprilTag 
    m_driverController.leftBumper().onTrue(new AutoAlign(true, m_robotDrive).withTimeout(3));

    /******************************************************************************************
     * CORAL CONTROLS
     *******************************************************************************************/

    // Right stick -> Run coral intake (bring in coral)
    m_driverController.rightStick().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Right Bumper -> Run coral intake in reverse (spit out coral)
    m_driverController.rightBumper().onTrue(m_coralSubSystem.reverseIntakeCommand());

    /******************************************************************************************
     * ELEVATOR CONTROLS
     *******************************************************************************************/

    //Start Button -> Elevator does not move, flips coral intake to algae side of bot to collect coral
    m_driverController.start().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation).alongWith(m_algaeSubsystem.stowCommand()));
    
    // A Button -> Elevator and coral arm to L1 position
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));

    // B Button -> Elevator and coral arm to L2 position
    m_driverController.b().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator and coral arm to L3 position
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator and coral arm to L4 position
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    /******************************************************************************************
     * ALGAE CONTROLS
     *******************************************************************************************/

    // Right Trigger -> Run algae intake, set to leave out when idle
    // Should make the algae arm drop and rotate the wheels to intake algae when Right Trigger is held
    // When the right trigger is released the arm should move to its hold position and apply power to wheels to keep the algae contained
    m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    // Should make the wheels spin and score the algae from the arms hold position
    // When the left trigger is released the arm should return to its stow position
    m_driverController.leftTrigger(OIConstants.kTriggerButtonThreshold).whileTrue(m_algaeSubsystem.reverseIntakeCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
