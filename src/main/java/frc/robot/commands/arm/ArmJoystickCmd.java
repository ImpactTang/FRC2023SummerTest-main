package frc.robot.commands.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmJoystickCmd extends CommandBase {

  private final Arm armSubsystem;
  Supplier<Double> rotateSetpoint, extensionSetpoint;

  public ArmJoystickCmd(Arm armSubsystem, Supplier<Double> rotateSetpoint, Supplier<Double> extensionSetpoint) {
    this.armSubsystem = armSubsystem;
    this.rotateSetpoint = rotateSetpoint;
    this.extensionSetpoint = extensionSetpoint;
    
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    armSubsystem.updateArmExtensionSetpoint(rotateSetpoint.get());
    armSubsystem.updateArmRotationSetpoint(extensionSetpoint.get());
  }

  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
