package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.SwerveModule;
import frc.robot.Constants;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveConstants.*;

public class Swerve extends SubsystemBase {
	private final SwerveModule flModule = new SwerveModule("FrontLeft", 0, Mod0.kConstants);
	private final SwerveModule frModule = new SwerveModule("FrontRight", 1, Mod1.kConstants);
	private final SwerveModule rlModule = new SwerveModule("RearLeft", 2, Mod2.kConstants);
	private final SwerveModule rrModule = new SwerveModule("RearRight", 3, Mod3.kConstants);

	private final SwerveModule[] m_modules = new SwerveModule[] {
			flModule, frModule, rlModule, rrModule
	};

	private final Pigeon2 m_pigeon = new Pigeon2(Constants.SwerveConstants.kPigeonCanId);

	protected final PIDController autoThetaController = new PIDController(kPthetaController, 0, kDthetaController);

	private int m_periodicCallCount = 0;
	private int m_lastPigeonGyroReq = 0;

	private double[] m_pigeonGyroRateDPS = new double[3];

	public Swerve() {
	}

	private void updatePigeonGyroRate() {
		if (m_periodicCallCount > m_lastPigeonGyroReq) {
			m_lastPigeonGyroReq++;
			m_pigeon.getRawGyro(m_pigeonGyroRateDPS);
		}
	}

	public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace) {
		setModuleStates(kKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
	}

	public void setCoastModules() {
		for (SwerveModule mod : m_modules) {
			mod.setAngleCoast();
		}
	}

	public void setModuleAngle(double degrees) {
		var rot2d = Rotation2d.fromDegrees(degrees);
		for (SwerveModule mod : m_modules) {
			mod.setAngle(rot2d);
		}
	}

	/**
	 * Basic teleop drive control. ChassisSpeeds values representing vx, vy, and
	 * omega are converted to individual module states for the robot to follow.
	 * 
	 * @param x        X velocity (forward) in m/s.
	 * @param y        Y velocity (strafe) in m/s.
	 * @param omega    Angular velocity (rotation CCW+) in radians.
	 * @param openLoop If swerve modules should not use velocity PID.
	 */
	public void drive(double x, double y, double omega, boolean fieldRelative, boolean openLoop) {
		ChassisSpeeds targetChassisSpeeds = fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, getHeading())
				: new ChassisSpeeds(x, y, omega);

		setChassisSpeeds(targetChassisSpeeds, openLoop, false);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxVelocity);

		for (SwerveModule mod : m_modules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop, steerInPlace);
		}
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : m_modules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public void zeroGyro() {
		m_pigeon.setYaw(180);
		m_pigeon.addYaw(0);
		for (var mod : m_modules) {
			mod.setAngle(Rotation2d.fromDegrees(0));
		}
	}

	protected double getGyroYaw() {
		return m_pigeon.getYaw();
	}

	protected double getGyroRoll() {
		return m_pigeon.getPitch(); // CTRE is dumb
	}

	protected double getGyroPitch() {
		return m_pigeon.getRoll(); // CTRE is dumb
	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(getGyroYaw());
	}

	/**
	 * Set relative drive encoders to 0.
	 */
	public void resetDriveEncoders() {
		for (var module : m_modules) {
			module.resetDriveToZero();
		}
	}

	public CommandBase driveTeleop(
			DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
			BooleanSupplier robotCentric) {
		return run(() -> {
			double xVal = MathUtil.applyDeadband(x.getAsDouble(), Constants.kStickDeadband);
			double yVal = MathUtil.applyDeadband(y.getAsDouble(), Constants.kStickDeadband);
			double omegaVal = MathUtil.applyDeadband(omega.getAsDouble(), Constants.kStickDeadband) * .80;

			xVal *= kMaxVelocity;
			yVal *= kMaxVelocity;
			omegaVal *= kMaxAngularVelocity;

			drive(xVal, yVal, omegaVal, !robotCentric.getAsBoolean(), true);
		}).withName("TeleopDrive");
	}

	public CommandBase teleopReset() {
		return runOnce(this::zeroGyro);
	}

	@Override
	public void periodic() {
		m_periodicCallCount++;
		updatePigeonGyroRate();

		for (var module : m_modules) {
			module.periodic();
		}
	}
}