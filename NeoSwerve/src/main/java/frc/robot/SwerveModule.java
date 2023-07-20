package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private WPI_TalonFX m_steerMotor;
    private WPI_TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;

        m_angleEncoder = new CANCoder(moduleConstants.cancoderId, "Canivore");
        configAngleEncoder();

        m_steerMotor = new WPI_TalonFX(moduleConstants.angleMotorId, "Canivore");
        configAngleMotor();

        m_driveMotor = new WPI_TalonFX(moduleConstants.driveMotorId, "Canivore");
        configDriveMotor();

        m_lastAngle = getState().angle;
    }

    public void periodic() {
    }

    public double makePositiveDegrees(double angle) {
        double degrees = angle;
        degrees = degrees % 360;
        if (degrees < 0.0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double makePositiveDegrees(Rotation2d angle) {
        return makePositiveDegrees(angle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle) {
        double steerAngle = makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360)
        // instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;
        } else if (difference < -360) {
            steerAngle += 360;
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is > 90 deg < -90 deg the drive can be inverted so the
        // total movement of the module is < 90 deg
        if (difference > 90 || difference < -90) {
            // Only need to add 180 deg here because the target angle will be put back
            // into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(makePositiveDegrees(steerAngle));
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * 
     * @param steerInPlace If modules should steer to target angle when target
     *                     velocity is 0.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState, steerInPlace);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxVelocity;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.mpsToFalcon(desiredState.speedMetersPerSecond, kWheelCircumference,
                    kDriveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    kDriveFf.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocity * 0.01)) ? m_lastAngle
                : desiredState.angle;

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), kAngleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(m_steerMotor.getSelectedSensorPosition(), kAngleGearRatio));
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    public Rotation2d getCancoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double cancoderDeg = getCancoder().getDegrees();
        double absPosDeg = cancoderDeg < 0 ? makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees() - 360
                : makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees();
        double absolutePosition = Conversions.degreesToFalcon(
                absPosDeg, kAngleGearRatio);
        m_steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    public void resetDriveToZero() {
        m_driveMotor.setSelectedSensorPosition(0);
    }

    private void configAngleEncoder() {
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.Get().swerveCanCoderConfig);
    }

    public void setOdoTestMode(boolean test) {
        m_steerMotor.setNeutralMode(test ? NeutralMode.Brake : NeutralMode.Coast);
        m_driveMotor.setNeutralMode(test ? NeutralMode.Coast : NeutralMode.Brake);

    }

    private void configAngleMotor() {
        m_steerMotor.configFactoryDefault();
        m_steerMotor.configAllSettings(CTREConfigs.Get().swerveAngleFXConfig);
        m_steerMotor.setInverted(kInvertAngleMotor);
        m_steerMotor.setNeutralMode(kAngleNeutralMode);
        Timer.delay(0.1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(CTREConfigs.Get().swerveDriveFXConfig);
        m_driveMotor.setInverted(kInvertDriveMotor);
        m_driveMotor.setNeutralMode(kDriveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.falconToMps(m_driveMotor.getSelectedSensorVelocity(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }
}