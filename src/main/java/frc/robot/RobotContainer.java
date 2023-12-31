package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.arm.ArmJoystickCmd;
import frc.robot.commands.swerve.SwerveJoystickCmd;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

  public SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public Arm armSubsystem = new Arm();

  public XboxController driveController = new XboxController(0);
  //public CommandXboxController cmdDriveController = new CommandXboxController(0);

  private final JoystickButton robotCentric = new JoystickButton(driveController, XboxController.Button.kA.value);
  private final JoystickButton zeroGyro = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);

  public RobotContainer() {

    // Xbox Controller Driving
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
    () -> -driveController.getRawAxis(0), // Axis 0 = Left X Stick
    () -> driveController.getRawAxis(1), // Axis 1 = Left Y Stick
    () -> driveController.getRawAxis(4), // Axis 2 = Right X Stick
    () -> robotCentric.getAsBoolean()));

    armSubsystem.setDefaultCommand(new ArmJoystickCmd(armSubsystem, 
    () -> driveController.getRawAxis(5),
    () -> driveController.getRawAxis(3)));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.resetHeading()));
  }
  
}
