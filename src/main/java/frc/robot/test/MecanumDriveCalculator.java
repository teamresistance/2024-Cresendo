package frc.robot.test;

public class MecanumDriveCalculator {
    public static double[] calculateMecanumDrive(double xSpeed, double ySpeed, double zRotation, double gyroAngle) {
        double wheelFL = ySpeed - xSpeed + zRotation;
        double wheelFR = ySpeed + xSpeed - zRotation;
        double wheelBL = ySpeed + xSpeed + zRotation;
        double wheelBR = ySpeed - xSpeed - zRotation;

        // Rotate the mecanum drive vector based on the gyro angle
        double cosA = Math.cos(Math.toRadians(gyroAngle));
        double sinA = Math.sin(Math.toRadians(gyroAngle));

        double tempFL = cosA * wheelFL - sinA * wheelFR;
        double tempFR = sinA * wheelFL + cosA * wheelFR;
        double tempBL = cosA * wheelBL - sinA * wheelBR;
        double tempBR = sinA * wheelBL + cosA * wheelBR;

        wheelFL = tempFL;
        wheelFR = tempFR;
        wheelBL = tempBL;
        wheelBR = tempBR;

        // Normalize the values to be in the range [-1, 1]
        double maxMagnitude = Math.max(Math.max(Math.abs(wheelFL), Math.abs(wheelFR)),
                                       Math.max(Math.abs(wheelBL), Math.abs(wheelBR)));

        if (maxMagnitude > 1.0) {
            wheelFL /= maxMagnitude;
            wheelFR /= maxMagnitude;
            wheelBL /= maxMagnitude;
            wheelBR /= maxMagnitude;
        }

        return new double[]{wheelFL, wheelFR, wheelBL, wheelBR};
    }
}
