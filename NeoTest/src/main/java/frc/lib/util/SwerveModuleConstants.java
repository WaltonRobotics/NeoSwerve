package frc.lib.util;

public class SwerveModuleConstants {
    public final int MODULE_ID;
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
    public SwerveModuleConstants(
        int moduleId,
        int driveMotorId,
        int angleMotorId,
        boolean invertDrive
    ) {
        this(moduleId, driveMotorId, angleMotorId, invertDrive, "");
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
    public SwerveModuleConstants(
        int moduleId,
        int driveMotorId,
        int angleMotorId,
        boolean invertDrive,
        String canbus
    ) {
        MODULE_ID = moduleId;
        DRIVE_MOTOR_ID = driveMotorId;
        ANGLE_MOTOR_ID = angleMotorId;
        driveInverted = invertDrive;
        CANBUS = canbus;
    }

}