// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.lib.util.SwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final boolean DEBUG_LOGGING_ENABLED = true;
	public static final double STICK_DEADBAND = 0.1;
	public static final String CANBUS = "Canivore";

	public static final class SwerveK {
		public static final String DB_TAB_NAME = "Swerve";

		/**
		 * Set to true to use external CANcoder for inital zero and switch to internal
		 * falcon encoder for angle control.
		 * Set to false to always use external CANcoder for angle control.
		 * Recommended to set to false and always use CANCoder.
		 */
		public static final boolean USE_INTERNAL_ENCODER = false;

		public static final int PIGEON_CAN_ID = 1;

		public static final SwerveConstants SWERVE_MODULE = SwerveConstants
				.SDSMK4i(SwerveConstants.driveGearRatios.SDSMK4i_L2);

		// TODO: check these values
		public static final double TRACK_WIDTH = Units.inchesToMeters(33);
		public static final double WHEEL_BASE = Units.inchesToMeters(33);
		public static final double WHEEL_CIRCUMFERENCE = SWERVE_MODULE.WHEEL_CIRCUMFERENCE;

		public static final Translation2d[] MODULE_TRANSLATIONS = {
				new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
				new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
		};

		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

		public static final double DRIVE_GEAR_RATIO = SWERVE_MODULE.DRIVE_GEAR_RATIO;
		public static final double ANGLE_GEAR_RATIO = SWERVE_MODULE.ANGLE_GEAR_RATIO;

		public static final boolean ANGLE_MOTOR_INVERTED = SWERVE_MODULE.ANGLE_MOTOR_INVERTED;
		public static final boolean DRIVE_MOTOR_INVERTED = SWERVE_MODULE.DRIVE_MOTOR_INVERTED;

		public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
		public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
		public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
		public static final boolean ANGLE_CURRENT_LIMIT_ENABLED = true;

		public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
		public static final int DRIVE_PEAK_CURRENT_LIMIT = 70;
		public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
		public static final boolean DRIVE_CURRENT_LIMIT_ENABLED = true;

		/*
		 * These values are used by the drive Falcon to ramp in open loop and closed
		 * loop driving.
		 * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc.
		 */
		public static final double OPEN_LOOP_RAMP = 0.25;
		public static final double CLOSED_LOOP_RAMP = 0.0;

		public static final double ANGLE_KP = SWERVE_MODULE.ANGLE_KP;
		public static final double ANGLE_KI = SWERVE_MODULE.ANGLE_KI;
		public static final double ANGLE_KD = SWERVE_MODULE.ANGLE_KD;
		public static final double ANGLE_KF = SWERVE_MODULE.ANGLE_KF;

		public static final double DRIVE_KP = 0.05;
		public static final double DRIVE_KI = 0.0;
		public static final double DRIVE_KD = 0.0;
		public static final double DRIVE_KF = 0.0;

		public static final double DRIVE_KS = 0.32 / 12;
		public static final double DRIVE_KV = 1.51 / 12;
		public static final double DRIVE_KA = 0.27 / 12;

		public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward( // real
				DRIVE_KS, // Voltage to break static friction
				DRIVE_KV, // Volts per meter per second
				DRIVE_KA // Volts per meter per second squared
		);
		public static final SimpleMotorFeedforward ANGLE_FF = new SimpleMotorFeedforward( // real
				0.5, // Voltage to break static friction
				0.23, // Volts per radian per second
				0.0056 // Volts per radian per second squared
		);

		public static final double MAX_VELOCITY = 5.0; // m/s
		public static final double MAX_ANGULAR_VELOCITY = 11.5; // rad/s

		public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;
		public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;

		/* Front left module */
		public static final class Mod0 {
			public static final int DRIVE_MOTOR_ID = 1;
			public static final int ANGLE_MOTOR_ID = 2;
			public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
			public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
					ANGLE_MOTOR_ID, ANGLE_OFFSET);
		}

		/* Front right module */
		public static final class Mod1 {
			public static final int DRIVE_MOTOR_ID = 3;
			public static final int ANGLE_MOTOR_ID = 4;
			public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
			public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(DRIVE_MOTOR_ID,
					ANGLE_MOTOR_ID, ANGLE_OFFSET);
		}

		/* Back left module */
		public static final class Mod2 {
			public static final int DRIVE_MOTOR_ID = 5;
			public static final int ANGLE_MOTOR_ID = 6;
			public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
					ANGLE_MOTOR_ID, ANGLE_OFFSET);
		}

		/* Back right module */
		public static final class Mod3 {
			public static final int DRIVE_MOTOR_ID = 7;
			public static final int ANGLE_MOTOR_ID = 8;
			public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(0);
			public static final SwerveModuleConstants kConstants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
					ANGLE_MOTOR_ID, ANGLE_OFFSET);
		}
	}

	public static final class AutoConstants {
		public static final double PX_CONTROLLER = 3.25;
		public static final double PY_CONTROLLER = 3.25;
		public static final double PTHETA_CONTROLLER = 3.15;

		public static final double DTHETA_CONTROLLER = 0.5;
	}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
