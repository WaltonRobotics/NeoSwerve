package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModule {
    public final String moduleName;
    public final int moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private CANCoder m_angleEncoder;

    public SwerveModule(String name, int moduleNumber, SwerveModuleConstants moduleConstants) {
        moduleName = name;
        this.moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.angleOffset;

        m_angleEncoder = new CANCoder(moduleConstants.cancoderId, "Canivore");
        configAngleEncoder();

        m_angleMotor = new CANSparkMax(moduleConstants.angleMotorId, MotorType.kBrushless);
        configAngleMotor();

        m_driveMotor = new CANSparkMax(moduleConstants.driveMotorId, MotorType.kBrushless);
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
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.mpsToNeo(desiredState.speedMetersPerSecond, kMaxVelocity);
            m_driveMotor.set(velocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxVelocity * 0.01)) ? m_lastAngle
                : desiredState.angle;

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_angleEncoder.setPosition(Conversions.degreesToCancoder(angle.getDegrees(), kAngleGearRatio));
        m_lastAngle = angle;
        // System.out.println(m_angleMotor.getAppliedOutput());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.neoToDegrees(m_angleEncoder.getAbsolutePosition(), kAngleGearRatio));
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public Rotation2d getCancoder() {
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double cancoderDeg = getCancoder().getDegrees();
        double absPosDeg = cancoderDeg < 0 ? makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees() - 360
                : makePositiveDegrees(cancoderDeg) - m_angleOffset.getDegrees();
        double absolutePosition = Conversions.degreesToNeo(
                absPosDeg, kAngleGearRatio);
        m_angleEncoder.setPosition(absolutePosition);
    }

    public void resetDriveToZero() {
        m_driveMotor.getEncoder().setPosition(0);
    }

    private void configAngleEncoder() {
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(CTREConfigs.Get().swerveCanCoderConfig);
    }

    public void setOdoTestMode(boolean test) {
        m_angleMotor.setIdleMode(test ? IdleMode.kBrake : IdleMode.kCoast);
        m_driveMotor.setIdleMode(test ? IdleMode.kCoast : IdleMode.kBrake);
    }

    private void configAngleMotor() {
        m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.burnFlash();
        m_angleMotor.setInverted(kInvertAngleMotor);
        m_angleMotor.setIdleMode(kAngleIdleMode);
        Timer.delay(0.1);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.burnFlash();
        m_driveMotor.setInverted(kInvertDriveMotor);
        m_driveMotor.setIdleMode(kDriveIdleMode);
        m_driveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.neoToMps(m_driveMotor.get(), kMaxVelocity),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.neoToMeters(m_driveMotor.getEncoder().getPosition(), kWheelCircumference,
                        kDriveGearRatio),
                getAngle());
    }
}