package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  private CANSparkMax intakeMotor;
  private PIDController intakePidController;

  public Intake() {

    intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setInverted(IntakeConstants.intakeMotorInverted);
    intakeMotor.setSmartCurrentLimit(IntakeConstants.intakeMotorStallCurrentLimit, IntakeConstants.intakeMotorFreeSpinCurrentLimit);
    intakeMotor.setOpenLoopRampRate(IntakeConstants.openLoopRampRate);
    intakeMotor.burnFlash();
    
    intakePidController = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);

  }

  @Override
  public void periodic() {

  }

  
}
