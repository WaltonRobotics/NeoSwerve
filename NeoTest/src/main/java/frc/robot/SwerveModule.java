package frc.robot;

import static frc.robot.Constants.SwerveK.ANGLE_GEAR_RATIO;
import static frc.robot.Constants.SwerveK.ANGLE_IDLE_MODE;
import static frc.robot.Constants.SwerveK.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.SwerveK.DRIVE_IDLE_MODE;
import static frc.robot.Constants.SwerveK.ANGLE_MOTOR_INVERTED;
import static frc.robot.Constants.SwerveK.DRIVE_MOTOR_INVERTED;
import static frc.robot.Constants.SwerveK.MAX_VELOCITY;
import static frc.robot.Constants.SwerveK.WHEEL_CIRCUMFERENCE;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
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

public class SwerveModule {
    public final String MODULE_NAME;
    public final int MODULE_NUMBER;
    private Rotation2d m_lastAngle;

    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private AbsoluteEncoder m_angleEncoder;
    private SparkMaxPIDController m_angleController;

    public SwerveModule(String name, int number, SwerveModuleConstants constants) {
        MODULE_NAME = name;
        MODULE_NUMBER = number;

        m_angleMotor = new CANSparkMax(constants.ANGLE_MOTOR_ID, MotorType.kBrushless);
        var resetErr = m_angleMotor.restoreFactoryDefaults();
        m_angleMotor.disableVoltageCompensation();
        m_angleMotor.setClosedLoopRampRate(0.125);
        m_angleMotor.setInverted(ANGLE_MOTOR_INVERTED);
        m_angleMotor.setIdleMode(ANGLE_IDLE_MODE);

        m_angleEncoder = m_angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
        
        var encPosConvErr = m_angleEncoder.setPositionConversionFactor(360.0);

        m_angleController = m_angleMotor.getPIDController();
        var ctrlFeedbackErr = m_angleController.setFeedbackDevice(m_angleEncoder);
        var pidWrapOnErr = m_angleController.setPositionPIDWrappingEnabled(true);
        var pidWrapMinErr = m_angleController.setPositionPIDWrappingMinInput(0);
        var pidWrapMaxErr = m_angleController.setPositionPIDWrappingMaxInput(360);
        m_angleController.setOutputRange(-12.0, 12.0);
        // System.out.printf("[SwerveModule %d] REV CANSparkMax.restoreFactoryDefaults %s\n", number, "error: " + resetErr);
        // System.out.printf("[SwerveModule %d] REV AbsoluteEncoder.setEncoderPositionConversion %s\n", number, "error: " + encPosConvErr);
        // System.out.printf("[SwerveModule %d] REV SparkMaxPIDController.setFeedbackDevice %s\n", number, "error: " + ctrlFeedbackErr);
        // System.out.printf("[SwerveModule %d] REV SparkMaxPIDController.setPositionPIDWrappingEnabled %s\n", number, "error: " + pidWrapOnErr);
        // System.out.printf("[SwerveModule %d] REV SparkMaxPIDController.setPositionPIDWrappingMinInput %s\n", number, "error: " + pidWrapMinErr);
        // System.out.printf("[SwerveModule %d] REV SparkMaxPIDController.setPositionPIDWrappingMaxInput %s\n", number, "error: " + pidWrapMaxErr);
      
        m_angleEncoder.setZeroOffset(0.0);

        
        m_angleController.setP(0.00058);
        m_angleController.setI(0);
        m_angleController.setD(0);
        m_angleController.setFF(0);
     
        m_angleController.setReference(m_angleEncoder.getPosition(), ControlType.kPosition);

        m_driveMotor = new CANSparkMax(constants.DRIVE_MOTOR_ID, MotorType.kBrushless);
        configDriveMotor();

        m_lastAngle = getState().angle;
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
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        // desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState, steerInPlace);
        setSpeed(desiredState, isOpenLoop);
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
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_VELOCITY * 0.01)) ? m_lastAngle
                : desiredState.angle;

        if (!steerInPlace && Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
            angle = m_lastAngle;
        }

        m_lastAngle = angle;
        SmartDashboard.putNumber(MODULE_NAME + " setpoint", angle.getRotations());
        m_angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                Conversions.neoToDegrees(
                        m_angleMotor.getEncoder().getPosition(),
                        ANGLE_GEAR_RATIO));
    }

    public void setAngleCoast() {
        m_angleMotor.setIdleMode(IdleMode.kCoast);
    }

    public double getDriveMotorPosition() {
        return m_driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }

    public Rotation2d getCanandcoder() {
        return Rotation2d.fromRotations(m_angleEncoder.getPosition());
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
        m_driveMotor.setInverted(DRIVE_MOTOR_INVERTED);
        m_driveMotor.setIdleMode(DRIVE_IDLE_MODE);
        m_driveMotor.getEncoder().setPosition(0);
        Timer.delay(0.1);
        m_driveMotor.burnFlash();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.neoToMps(m_driveMotor.getEncoder().getVelocity(), MAX_VELOCITY),
                getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.neoToMeters(m_driveMotor.getEncoder().getPosition(), WHEEL_CIRCUMFERENCE,
                        DRIVE_GEAR_RATIO),
                getAngle());
    }
}