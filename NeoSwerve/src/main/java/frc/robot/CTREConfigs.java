package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    private static final class Container {
        public static final CTREConfigs INSTANCE = new CTREConfigs();
    }

    public static CTREConfigs Get() {
        return Container.INSTANCE;
    }

    public final TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public final CANCoderConfiguration swerveCanCoderConfig = new CANCoderConfiguration();
    public final TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    public final TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    private CTREConfigs() {
        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.kAngleEnableCurrentLimit,
                Constants.SwerveConstants.kAngleContinuousCurrentLimit,
                Constants.SwerveConstants.kAnglePeakCurrentLimit,
                Constants.SwerveConstants.kAnglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConstants.kAngleKp;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConstants.kAngleKi;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConstants.kAngleKd;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConstants.kAngleKf;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
                Constants.SwerveConstants.kDriveEnableCurrentLimit,
                Constants.SwerveConstants.kDriveContinuousCurrentLimit,
                Constants.SwerveConstants.kDrivePeakCurrentLimit,
                Constants.SwerveConstants.kDrivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConstants.kDriveKp;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConstants.kDriveKi;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConstants.kDriveKd;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConstants.kDriveKf;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConstants.kOpenLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstants.kClosedLoopRamp;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.kInvertCancoder;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}