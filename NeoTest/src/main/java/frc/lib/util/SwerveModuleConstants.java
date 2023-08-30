package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final boolean driveInverted;
    public final int ANGLE_MOTOR_ID;
    public final String CANBUS;

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId,
        boolean invertDrive) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        driveInverted = invertDrive;
        CANBUS = "";
    }

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     * @param angleOffset
     * @param canbus
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId,
            boolean invertDrive, String canbus) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        driveInverted = invertDrive;
        CANBUS = canbus;
    }

}