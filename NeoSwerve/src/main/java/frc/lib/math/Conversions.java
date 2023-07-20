package frc.lib.math;

public class Conversions {

    /**
     * @param counts    CANCoder position counts.
     * @param gearRatio Gear ratio between CANCoder and mechanism.
     * @return Mechanism degrees of rotation.
     */
    public static double cancoderToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees   Mechanism degrees of rotation.
     * @param gearRatio Gear ratio between CANCoder and mechanism.
     * @return CANCoder position counts.
     */
    public static double degreesToCancoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts    Falcon position counts.
     * @param gearRatio Gear ratio between Falcon and mechanism.
     * @return Mechanism degrees of rotation.
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees   Mechanism degrees of rotation.
     * @param gearRatio Gear ratio between Falcon and mechanism.
     * @return Falcon position counts.
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param counts    Falcon velocity counts.
     * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for
     *                  Falcon rpm).
     * @return Mechanism rpm.
     */
    public static double falconToRpm(double counts, double gearRatio) {
        double motorRpm = counts * (600.0 / 2048.0);
        double mechRpm = motorRpm / gearRatio;
        return mechRpm;
    }

    /**
     * @param rpm       Mechanism rpm.
     * @param gearRatio Gear ratio between Falcon and mechanism (set to 1 for
     *                  Falcon rpm).
     * @return Falcon velocity counts.
     */
    public static double rpmToFalcon(double rpm, double gearRatio) {
        double motorRPM = rpm * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param counts        Falcon velocity counts.
     * @param circumference Wheel circumference.
     * @param gearRatio     Gear ratio between Falcon and mechanism (set to 1 for
     *                      Falcon m/s).
     * @return Falcon velocity counts.
     */
    public static double falconToMps(double counts, double circumference, double gearRatio) {
        double wheelRpm = falconToRpm(counts, gearRatio);
        double wheelMps = (wheelRpm * circumference) / 60;
        return wheelMps;
    }

    /**
     * @param velocity      Velocity m/s.
     * @param circumference Wheel circumference.
     * @param gearRatio     Gear ratio between falcon and mechanism (set to 1 for
     *                      Falcon m/s).
     * @return Falcon velocity counts.
     */
    public static double mpsToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRpm = ((velocity * 60) / circumference);
        double wheelVelocity = rpmToFalcon(wheelRpm, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Falcon position counts.
     * @param circumference  Wheel circumference.
     * @param gearRatio      Gear ratio between Falcon and wheel.
     * @return Meters traveled.
     */
    public static double falconToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 2048.0));
    }

    /**
     * @param meters        Meters traveled.
     * @param circumference Wheel circumference.
     * @param gearRatio     Gear ratio between Falcon and wheel.
     * @return Falcon position counts.
     */
    public static double metersToFalcon(double meters, double circumference, double gearRatio) {
        return meters / (circumference / (gearRatio * 2048.0));
    }
}