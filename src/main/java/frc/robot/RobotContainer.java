
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CageDoorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import com.pathplanner.lib.commands.PathPlannerAuto;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final Joystick driverLeftStick = new Joystick(0);
    private static final Joystick driverRightStick = new Joystick(1);
    private static final XboxController copilotXbox = new XboxController(2);

    private final JoystickButton left1Button = new JoystickButton(driverLeftStick, 1);
    private final JoystickButton left2Button = new JoystickButton(driverLeftStick, 2);
    private final JoystickButton left3Button = new JoystickButton(driverLeftStick, 3);
    private final JoystickButton left4Button = new JoystickButton(driverLeftStick, 4);

    private final JoystickButton left7Button = new JoystickButton(driverLeftStick, 7);
    private final JoystickButton left8Button = new JoystickButton(driverLeftStick, 8);
    private final JoystickButton left9Button = new JoystickButton(driverLeftStick, 9);
    private final JoystickButton left10Button = new JoystickButton(driverLeftStick, 10);

    private final JoystickButton right1Button = new JoystickButton(driverRightStick, 1);
    private final JoystickButton right2Button = new JoystickButton(driverRightStick, 2);
    private final JoystickButton right3Button = new JoystickButton(driverRightStick, 3);
    private final JoystickButton right4Button = new JoystickButton(driverRightStick, 4);
    private final JoystickButton right6Button = new JoystickButton(driverRightStick, 6);

    private final JoystickButton right9Button = new JoystickButton(driverRightStick, 9);
    private final JoystickButton right10Button = new JoystickButton(driverRightStick, 10);
    
    private final JoystickButton xButton = new JoystickButton(copilotXbox, Button.kX.value);
    private final JoystickButton bButton = new JoystickButton(copilotXbox, Button.kB.value);
    private final JoystickButton aButton = new JoystickButton(copilotXbox, Button.kA.value);
    private final JoystickButton yButton = new JoystickButton(copilotXbox, Button.kY.value);

    private SendableChooser<Command> auto = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The shooter subsystem
  private final CoralShooter m_coralShooter = new CoralShooter();

  private final ClimberSubsystem m_Climber = new ClimberSubsystem();

  private final CageDoorSubsystem m_CageDoorSubsystem = new CageDoorSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverLeftStick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverLeftStick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverRightStick.getX(), OIConstants.kDriveDeadband),
                true),
                m_robotDrive));
            //     new RunCommand(
            // () -> m_robotDrive.drive(
            //     -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
            //     -MathUtil.applyDeadband(0, OIConstants.kDriveDeadband),
            //     -MathUtil.applyDeadband(0.2, OIConstants.kDriveDeadband),
            //     true),
            //     m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
   
    aButton.whileTrue(m_coralShooter.shootCoralCommand(1));
    xButton.whileTrue(m_coralShooter.shootCoralCommand(-1));
    bButton.whileTrue(m_coralShooter.shootCoralCommand(0.25));
    yButton.whileTrue(m_Climber.climberDownCommand());
    right4Button.whileTrue(m_Climber.climberUpCommand());
    right2Button.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    left3Button.whileTrue(m_CageDoorSubsystem.cagedooropenCommand());
    right3Button.whileTrue(m_CageDoorSubsystem.cagedoorcloseCommand());

    createAuto();
  }

    public void createAuto() {
        // auto = new SendableChooser<>();

        // auto.setDefaultOption("F-E Auto", new PathPlannerAuto("F-E Auto"));
        // auto.addOption("J-I Auto", new PathPlannerAuto("J-I Auto"));
        // auto.addOption("H Auto", new PathPlannerAuto("H Auto"));
        // auto.addOption("G auto", new PathPlannerAuto("G auto"));

        // SmartDashboard.putData("Autonomous Command", auto);
    }
    
    public Command getAutonomousCommand() {
                // Optional<Alliance> alliance = DriverStation.getAlliance();
                // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                // m_robotDrive.setFlipped(true);
                // } else {
                // m_robotDrive.setFlipped(false);
                // }
                Command command = auto.getSelected();
                createAuto();
                return command;
 }
}
