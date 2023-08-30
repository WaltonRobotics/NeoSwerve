package frc.robot;

import static frc.robot.Constants.SwerveK.ANGLE_IDLE_MODE;
import static frc.robot.Constants.SwerveK.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.SwerveK.DRIVE_IDLE_MODE;
import static frc.robot.Constants.SwerveK.MAX_VELOCITY;
import static frc.robot.Constants.SwerveK.WHEEL_CIRCUMFERENCE;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveK;

public class SwerveModule {
    public final String MODULE_NAME;
    public final int MODULE_NUMBER;

    private final SwerveModuleConstants m_constants;
    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private SparkMaxAbsoluteEncoder m_angleEncoder;
    private SparkMaxPIDController m_angleController;

    public SwerveModule(String name, int number, SwerveModuleConstants constants) {
        MODULE_NAME = name;
        MODULE_NUMBER = number;
        m_constants = constants;

        m_angleMotor = new CANSparkMax(constants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.disableVoltageCompensation();
        m_angleMotor.setClosedLoopRampRate(0.05);
        m_angleMotor.setInverted(false);
        m_angleMotor.setIdleMode(ANGLE_IDLE_MODE);
        m_angleMotor.setSmartCurrentLimit(35);

        m_angleEncoder = m_angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

        m_angleEncoder.setPositionConversionFactor(SwerveK.ANGLE_ENC_POS_FACTOR);
        m_angleEncoder.setVelocityConversionFactor(SwerveK.ANGLE_ENC_VELO_FACTOR);

        m_angleController = m_angleMotor.getPIDController();
        m_angleController.setFeedbackDevice(m_angleEncoder);
        m_angleController.setPositionPIDWrappingEnabled(true);
        m_angleController.setPositionPIDWrappingMinInput(SwerveK.ANGLE_ENC_POS_PID_MIN_INPUT);
        m_angleController.setPositionPIDWrappingMaxInput(SwerveK.ANGLE_ENC_POS_PID_MAX_INPUT);
        m_angleController.setOutputRange(-1.0, 1.0);

        m_angleEncoder.setZeroOffset(0.0);

        m_angleController.setP(1);
        m_angleController.setI(0);
        m_angleController.setD(0);
        m_angleController.setFF(0);

        m_angleController.setReference(m_angleEncoder.getPosition(), ControlType.kPosition);

        m_driveMotor = new CANSparkMax(constants.DRIVE_MOTOR_ID, MotorType.kBrushless);
        configDriveMotor();
    }

    public void periodic() {
        SmartDashboard.putNumber(MODULE_NAME + " output voltage", m_angleMotor.getAppliedOutput());
        SmartDashboard.putNumber(MODULE_NAME + " angle position", m_angleEncoder.getPosition());
        SmartDashboard.putNumber(MODULE_NAME + " drive position", getDriveMotorPosition());
    }

    /**
     * Command this swerve module to the desired angle and velocity.
     * Falcon onboard control is used instead of on-RIO control to avoid latency.
     * 
     * @param steerInPlace If modules should steer to target angle when target
     *                     velocity is 0.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_angleEncoder.getPosition()));
        setAngle(optimizedDesiredState, steerInPlace);
        setSpeed(optimizedDesiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / MAX_VELOCITY;
            m_driveMotor.set(percentOutput);
        } else {
            double velocity = Conversions.mpsToNeo(desiredState.speedMetersPerSecond, MAX_VELOCITY);
            m_driveMotor.set(velocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState, boolean steerInPlace) {
        // // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <=
        // (MAX_VELOCITY * 0.01)) ? m_lastAngle
        // : desiredState.angle;

        // if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
        // angle = m_lastAngle;
        // }

        // m_lastAngle = angle;
        SmartDashboard.putNumber(MODULE_NAME + " setpoint radians", desiredState.angle.getRadians());
        m_angleController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    }

    private Rotation2d getAbsAngle() {
        return Rotation2d.fromRadians(m_angleEncoder.getPosition());
    }

    public void setAngleCoast() {
        m_angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public void setAngle(Rotation2d angle) {
        m_angleController.setReference(angle.getRotations(), ControlType.kPosition);
    }

    public void resetDriveToZero() {
        m_driveMotor.getEncoder().setPosition(0);
    }

    public void setOdoTestMode(boolean test) {
        m_angleMotor.setIdleMode(test ? IdleMode.kBrake : IdleMode.kCoast);
        m_driveMotor.setIdleMode(test ? IdleMode.kCoast : IdleMode.kBrake);
    }

    private void configDriveMotor() {
        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setInverted(m_constants.driveInverted);
        m_driveMotor.setIdleMode(DRIVE_IDLE_MODE);
        m_driveMotor.getEncoder().setPosition(0);
        m_driveMotor.setSmartCurrentLimit(65);
        Timer.delay(0.1);
        m_driveMotor.burnFlash();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.neoToMps(
                m_driveMotor.getEncoder().getVelocity(),
                MAX_VELOCITY),
            getAbsAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.neoToMeters(
                m_driveMotor.getEncoder().getPosition(),
                WHEEL_CIRCUMFERENCE,
                DRIVE_GEAR_RATIO),
            getAbsAngle());
    }
}