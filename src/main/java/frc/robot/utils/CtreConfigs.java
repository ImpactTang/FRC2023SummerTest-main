package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class CtreConfigs {
    public static TalonFXConfiguration swerveDriveFXConfig;
    public static TalonFXConfiguration swerveTurnFXConfig;
    public static CANCoderConfiguration swerveCanCoderConfig;

    public CtreConfigs(){
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveTurnFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Drive Motor Configs 
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 35, 60, 0.1);
        
        swerveDriveFXConfig.slot0.kP = 0.5;
        swerveDriveFXConfig.slot0.kI = 0.0;
        swerveDriveFXConfig.slot0.kD = 0.0;
        swerveDriveFXConfig.slot0.kF = 0.0;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        //swerveDriveFXConfig.closedloopRamp = 0.0;

        /* Swerve Turn Motor Configs 
        SupplyCurrentLimitConfiguration turnSupplyLimit = new SupplyCurrentLimitConfiguration(
            true, 25, 40, 0.1);

        swerveTurnFXConfig.slot0.kP = 0.5;
        swerveTurnFXConfig.slot0.kI = 0.0;
        swerveTurnFXConfig.slot0.kD = 0.0;
        swerveTurnFXConfig.slot0.kF = 0.0;
        swerveTurnFXConfig.supplyCurrLimit = turnSupplyLimit;
        //swerveTurnFXConfig.closedloopRamp = 0.0;

        /* Swerve CANCoder Configs 
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = false;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        */
    }
}
