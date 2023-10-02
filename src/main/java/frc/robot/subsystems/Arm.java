package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase {

    private CANSparkMax rotateMotor;
    private CANSparkMax extensionMotor;

    private RelativeEncoder extensionMotorEncoder;
    private CANCoder rotateCanCoder;

    private PIDController rotatePidController;
    private PIDController extensionPidController;

    private double rotationSetpointRadians;
    private double extensionSetpointMeters;

    private SlewRateLimiter rotateSlewRateLimiter;
    private SlewRateLimiter extensionSlewRateLimiter;

    private ArmFeedforward rotateFeedforward;
    private ElevatorFeedforward extensionFeedforward;

    public Arm() {

        rotateMotor = new CANSparkMax(ArmConstants.rotateMotorId, MotorType.kBrushless);
        extensionMotor = new CANSparkMax(ArmConstants.extensionMotorId, MotorType.kBrushless);

        rotateMotor.setIdleMode(IdleMode.kBrake);
        extensionMotor.setIdleMode(IdleMode.kBrake);

        extensionMotorEncoder = extensionMotor.getEncoder();
        extensionMotorEncoder.setPositionConversionFactor(0.02367145);

        rotateCanCoder = new CANCoder(ArmConstants.rotateCanCoderId);
        rotateCanCoder.configFactoryDefault();
        rotateCanCoder.configMagnetOffset(0); // TODO: CONFIG OFFSET
        rotateCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotateCanCoder.configSensorDirection(false);
        rotateCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rotateCanCoder.configGetFeedbackTimeBase();

        rotateFeedforward = new ArmFeedforward(0.5, 0.5, 0.5, 0.5);
        extensionFeedforward = new ElevatorFeedforward(0.5, 0.5, 0.5, 0.05);

        rotatePidController = new PIDController(ArmConstants.rotatekP, ArmConstants.rotatekI, ArmConstants.rotatekD);
        extensionPidController = new PIDController(ArmConstants.extensionkP, ArmConstants.extensionkI, ArmConstants.extensionkD);

        rotateSlewRateLimiter = new SlewRateLimiter(5, 0, 0);
        extensionSlewRateLimiter = new SlewRateLimiter(5, 0, 0);

        rotationSetpointRadians = 0;
        extensionSetpointMeters = 0;
    }

    @Override
    public void periodic() {
        // TODO: Put SmartDashboard Data Here
        updateArmExtensionMeters();
        SmartDashboard.putNumber("Arm Extension Setpoint", extensionSetpointMeters);
    }

    public void updateArmExtensionMeters(){

        double calculated = extensionPidController.calculate(extensionMotorEncoder.getPosition(), extensionSetpointMeters);

        extensionMotor.set(calculated);

        calculated = extensionSlewRateLimiter.calculate(calculated);

        extensionMotor.set(calculated);
    }

    public void updateArmRotationRadians(){

        if (getArmRotateCanCoderRad() < -Math.PI / 2 || getArmRotateCanCoderRad() > Math.PI / 2){
            rotateMotor.set(0);
        }

        double canCoderValueRad = rotateCanCoder.getAbsolutePosition() * (2 * Math.PI / 4096.0);

        double calculated = rotatePidController.calculate(canCoderValueRad, rotationSetpointRadians);

        rotateMotor.set(calculated);

        calculated = rotateSlewRateLimiter.calculate(calculated);

        rotateMotor.set(calculated);
    }

    public void updateArmExtensionSetpoint(double input){
        extensionSetpointMeters = input * 1.4;
    }

    public void updateArmRotationSetpoint(double input){
        rotationSetpointRadians = input * 1.4;
    }

    public void setArmRotationRadians(double radians){
        rotationSetpointRadians = radians;
    }

    public void addArmRotationSetpoint(double input){
        rotationSetpointRadians += input;
    }

    public void subtractArmRotationSetpoint(double input){
        rotationSetpointRadians -= input;
    }

    public void setArmExtensionMeters(double meters){
        extensionSetpointMeters = meters;
    }

    public void addArmExtensionSetpoint(double input){
        extensionSetpointMeters += input;
    }

    public void subrtactArmExtensionSetpoint(double input){
        extensionSetpointMeters -= input;
    }

    public void setArmExtensionMotors(double input){
        extensionMotor.set(input);
    }

    public void setArmRotationMotors(double input){
        rotateMotor.set(input);
    }

    private double getArmRotateCanCoderRad(){
        return rotateCanCoder.getPosition() * (2 * Math.PI / 4096.0);
    }

    // Sets the arm rotation to stow 0 radians
    public boolean zeroArmRotation(){
        if (getArmRotateCanCoderRad() < -0.39){
            setArmRotationRadians(0.25);
        } else if (getArmRotateCanCoderRad() > 0.39){
            setArmRotationRadians(-0.25);
        } else {
            if (getArmRotateCanCoderRad() < -0.196){
                setArmRotationRadians(0.1);
            } else if (getArmRotateCanCoderRad() > 0.196){
                setArmRotationRadians(-0.1);
            } else {
                setArmRotationRadians(0);
                return true;
            }
        }
        return true;
    }

    public void stopArmExtension(){
        setArmExtensionMotors(0);
    }

    public void stopArmRotation(){
        setArmRotationMotors(0);
    }

    public void stopArm(){
        stopArmExtension();
        stopArmRotation();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Arm Extension Encoder", extensionMotorEncoder.getPosition());
        SmartDashboard.putNumber("Arm Rotation Encoder", getArmRotateCanCoderRad());
    }
}
