package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
	public static boolean kDebugLoggingEnabled = true;
	public static final double kStickDeadband = 0.1;
	public static final String kCanbus = "Canivore";

	public static final class SwerveConstants {
		// public static final String kDbTabName = "SwerveSubsys";

		/**
		 * Set to true to use external CANcoder for inital zero and switch to internal
		 * falcon encoder for angle control.
		 * Set to false to always use external CANcoder for angle control.
		 * Recommended to set to false and always use CANCoder.
		 */
		public static final boolean kUseInternalEncoder = false;

		public static final int kPigeonCanId = 1;

		public static final COTSFalconSwerveConstants kSwerveModule = COTSFalconSwerveConstants
				.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

		// TODO: check these values
		public static final double kTrackWidth = Units.inchesToMeters(33);
		public static final double kWheelBase = Units.inchesToMeters(33);
		public static final double kWheelCircumference = kSwerveModule.wheelCircumference;

		public static final Translation2d[] kModuleTranslations = {
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
		};

		public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

		public static final double kDriveGearRatio = kSwerveModule.driveGearRatio;
		public static final double kAngleGearRatio = kSwerveModule.angleGearRatio;

		public static final boolean kInvertAngleMotor = kSwerveModule.angleMotorInvert;
		public static final boolean kInvertDriveMotor = kSwerveModule.driveMotorInvert;

		public static final boolean kInvertCancoder = kSwerveModule.cancoderInvert;

		public static final int kAngleContinuousCurrentLimit = 25;
		public static final int kAnglePeakCurrentLimit = 40;
		public static final double kAnglePeakCurrentDuration = 0.1;
		public static final boolean kAngleEnableCurrentLimit = true;

		public static final int kDriveContinuousCurrentLimit = 40;
		public static final int kDrivePeakCurrentLimit = 70;
		public static final double kDrivePeakCurrentDuration = 0.1;
		public static final boolean kDriveEnableCurrentLimit = true;

		/*
		 * These values are used by the drive Falcon to ramp in open loop and closed
		 * loop driving.
		 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc.
		 */
		public static final double kOpenLoopRamp = 0.25;
		public static final double kClosedLoopRamp = 0.0;

		public static final double kAngleKp = kSwerveModule.angleKP;
		public static final double kAngleKi = kSwerveModule.angleKI;
		public static final double kAngleKd = kSwerveModule.angleKD;
		public static final double kAngleKf = kSwerveModule.angleKF;

		public static final double kDriveKp = 0.05;
		public static final double kDriveKi = 0.0;
		public static final double kDriveKd = 0.0;
		public static final double kDriveKf = 0.0;

		public static final double kDriveKs = 0.32 / 12;
		public static final double kDriveKv = 1.51 / 12;
		public static final double kDriveKa = 0.27 / 12;

		public static final SimpleMotorFeedforward kDriveFf = new SimpleMotorFeedforward( // real
				kDriveKs, // Voltage to break static friction
				kDriveKv, // Volts per meter per second
				kDriveKa // Volts per meter per second squared
		);
		public static final SimpleMotorFeedforward kSteerFf = new SimpleMotorFeedforward( // real
				0.5, // Voltage to break static friction
				0.23, // Volts per radian per second
				0.0056 // Volts per radian per second squared
		);

		public static final double kMaxVelocity = 5.0; // m/s
		public static final double kMaxAngularVelocity = 11.5; // rad/s

		public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;
		public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

		/* Front left module */
		public static final class Mod0 {
			public static final int kDriveMotorId = 1;
			public static final int kAngleMotorId = 2;
			public static final int kCancoderId = 1;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(3.95);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorId,
					kAngleMotorId,
					kCancoderId, kAngleOffset);
		}

		/* Front right module */
		public static final class Mod1 {
			public static final int kDriveMotorId = 3;
			public static final int kAngleMotorId = 4;
			public static final int kCancoderId = 3;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(147.8);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorId,
					kAngleMotorId,
					kCancoderId, kAngleOffset);
		}

		/* Back left module */
		public static final class Mod2 {
			public static final int kDriveMotorId = 5;
			public static final int kAngleMotorId = 6;
			public static final int kCancoderId = 7;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(73.4);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorId,
					kAngleMotorId,
					kCancoderId, kAngleOffset);
		}

		/* Back right module */
		public static final class Mod3 {
			public static final int kDriveMotorId = 7;
			public static final int kAngleMotorId = 8;
			public static final int kCancoderId = 5;
			public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(42.36);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(kDriveMotorId,
					kAngleMotorId,
					kCancoderId, kAngleOffset);
		}
	}

	public static final class AutoConstants {
		public static double kPxController = 3.25;
		public static double kPyController = 3.25;
		public static double kPthetaController = 3.15;

		public static final double kDthetaController = 0.5;
	}
}
