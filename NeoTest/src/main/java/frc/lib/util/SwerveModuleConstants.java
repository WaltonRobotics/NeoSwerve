package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int DRIVE_MOTOR_ID;
    public final boolean driveInverted;
    public final int ANGLE_MOTOR_ID;
    public final Rotation2d chassisAngularOffset;
    public final String CANBUS;

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, Rotation2d angleOffset,
        boolean invertDrive) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        chassisAngularOffset = angleOffset;
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
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, Rotation2d angleOffset,
            boolean invertDrive, String canbus) {
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        chassisAngularOffset = angleOffset;
        driveInverted = invertDrive;
        CANBUS = canbus;
    }

}