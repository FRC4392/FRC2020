package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {

    PigeonIMU mGyro;
    double mTrackWidth;
    double mWheelBase;
    SwervePod mPods[];
    boolean mIsFieldOriented;
    double kLengthComponent;
    double kWidthComponent;
    double ws[] = new double[4];
    double wa[] = new double[4];

    public SwerveDrive(PigeonIMU gyro, double trackWidth, double wheelBase, SwervePod pods[], boolean isFeildOriented) {
        mGyro = gyro;
        mTrackWidth = trackWidth;
        mWheelBase = wheelBase;
        mPods = pods;
        mIsFieldOriented = isFeildOriented && mGyro.getState() != PigeonState.NoComm;

        double radius = Math.hypot(wheelBase, trackWidth);
        kLengthComponent = wheelBase / radius;
        kWidthComponent = trackWidth / radius;
    }

    public void setAllOpenLoop(double azimuth, double drive) {
        for (SwervePod pod : mPods) {
            pod.setOpenLoop(azimuth, drive);
        }
    }

    public void stop() {
        for (SwervePod pod : mPods) {
            pod.stop();
        }
    }

    public void checkAllZeros(){
        for (SwervePod pod : mPods){
            pod.checkAzimuthZero();
        }
    }

    public void drive(double forward, double strafe, double azimuth) {

        double tempforward = Math.pow(forward, 2);
        if (forward < 0){
            tempforward *= -1;
        }
        forward = tempforward;

        double tempstrafe = Math.pow(strafe, 2);
        if (strafe < 0){
            tempstrafe *= -1;
        }
        strafe = tempstrafe;

        double tempAzimuth = Math.pow(azimuth, 2);
        if (azimuth < 0){
            tempAzimuth *= -1;
        }
        azimuth = tempAzimuth;

        if (Math.abs(forward) < 0.05) {
            forward = 0;
        }

        if (Math.abs(strafe) < 0.05) {
            strafe = 0;
        }
        if (Math.abs(azimuth) < 0.05) {
            azimuth = 0;
        }

        // Use gyro for field-oriented drive. We use getAngle instead of getYaw to enable arbitrary
        // autonomous starting positions.
        if (mIsFieldOriented) {
            double angle = mGyro.getFusedHeading();
            //angle = Math.IEEEremainder(angle, 360.0); //Can we uncomment this and have it still work?

            angle = Math.toRadians(angle);
            final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
            strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
            forward = temp;
        }

        final double a = strafe - azimuth * kLengthComponent;
        final double b = strafe + azimuth * kLengthComponent;
        final double c = forward - azimuth * kWidthComponent;
        final double d = forward + azimuth * kWidthComponent;

        // wheel speed
        ws[0] = Math.hypot(b, d);
        ws[1] = Math.hypot(b, c);
        ws[2] = Math.hypot(a, d);
        ws[3] = Math.hypot(a, c);

        // wheel azimuth
        wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
        wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
        wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
        wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

        // normalize wheel speed
        final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
        if (maxWheelSpeed > 1.0) {
            for (int i = 0; i < 4; i++) {
                ws[i] /= maxWheelSpeed;
            }
        }

        // set wheels
        for (int i = 0; i < 4; i++) {
            mPods[i].set(wa[i], ws[i]);
        }
    }

    public void setGyroAngle(double angle){
        mGyro.setFusedHeading(angle);
    }

    public double getGyroAngle(){
        return Math.IEEEremainder(mGyro.getFusedHeading(), 360.0);
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(mGyro.getFusedHeading());
    }

    public void setHeading(Rotation2d heading){
        mGyro.setFusedHeading(heading.getDegrees());
    }

    public boolean getIsFeildOriented() {
        return mIsFieldOriented;
    }


}