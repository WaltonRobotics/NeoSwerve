package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final Rotation2d angleOffset;
    public final String canbus;

    /**
     * Swerve module constants to be used when creating swerve modules.
     * 
     * @param driveMotorId
     * @param angleMotorId
     * @param cancoderId
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, int cancoderId, Rotation2d angleOffset) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderId = cancoderId;
        this.angleOffset = angleOffset;
        this.canbus = "";
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
    public SwerveModuleConstants(int driveMotorId, int angleMotorId, int cancoderId, Rotation2d angleOffset,
            String canbus) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderId = cancoderId;
        this.angleOffset = angleOffset;
        this.canbus = canbus;
    }

}