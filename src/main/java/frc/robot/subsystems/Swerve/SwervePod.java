package frc.robot.swerve;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwervePod {
    private static final int TICKS_PER_ROTATION = 4096;
    private final Spark mDriveMotor;
    private final TalonSRX mAzimuthMotor;
    private boolean isInverted = false;

    public SwervePod(Spark drive, TalonSRX azimuth) {
        this.mDriveMotor = drive;
        this.mAzimuthMotor = azimuth;

        azimuth.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        azimuth.configNominalOutputForward(0);
        azimuth.configNominalOutputReverse(0);
        azimuth.configPeakOutputForward(1);
        azimuth.configPeakOutputReverse(-1);

        azimuth.selectProfileSlot(0, 0);
        azimuth.configMotionCruiseVelocity(2000);
        azimuth.configMotionAcceleration(20000);
        azimuth.config_kF(0, 0.51);
        azimuth.config_kP(0, 7);
        azimuth.setSensorPhase(true);
    }
    
    public void set(double angle, double driveSpeed){

        angle *= TICKS_PER_ROTATION;
        
        double azimuthPosition = mAzimuthMotor.getSelectedSensorPosition(0);
        double azimuthError = Math.IEEEremainder(angle - azimuthPosition, TICKS_PER_ROTATION);

        isInverted = Math.abs(azimuthError) > 0.25 * TICKS_PER_ROTATION;
        if (isInverted) {
            azimuthError -= Math.copySign(0.5 * TICKS_PER_ROTATION, azimuthError);
            driveSpeed = -driveSpeed;
        }
        mAzimuthMotor.set(ControlMode.MotionMagic, azimuthError + azimuthPosition);
        //mAzimuthMotor.set(ControlMode.MotionMagic, angle);
        mDriveMotor.set(driveSpeed);
  }
    
    public void setOpenLoop(double azimuthSpeed, double driveSpeed){
        mDriveMotor.set(driveSpeed);
        mAzimuthMotor.set(ControlMode.PercentOutput,  azimuthSpeed);

    }

    public void setAzimuthPosition(int position) {
        mAzimuthMotor.set(ControlMode.MotionMagic, position);

    }

    public void setAzimuthOpenLoop(double speed){
        mAzimuthMotor.set(ControlMode.PercentOutput, speed);
    }

    public void disableAzimuth(){
        mAzimuthMotor.neutralOutput();
    }

    public void setWheelOpenLoop(double speed){
        mDriveMotor.set(speed);
    }

    //public void setWheelVelocity(double speed){
    //    mDriveMotor.set(ControlMode.Velocity, speed);
    //}

    public void stop(){
        mAzimuthMotor.set(ControlMode.MotionMagic, mAzimuthMotor.getSelectedSensorPosition(0));
        mDriveMotor.set(0);

    }

    public void setAzimuthZero(int zero) {
        int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
        mAzimuthMotor.setSelectedSensorPosition(-azimuthSetpoint, 0, 10);
        mAzimuthMotor.set(ControlMode.MotionMagic, azimuthSetpoint);
    }

    public int getAzimuthAbsolutePosition(){
        return mAzimuthMotor.getSensorCollection().getPulseWidthPosition() & 0xFFF;
    }

    //public void setWheelPosition(int position){
    //    mDriveMotor.setSelectedSensorPosition(position);
    //}

    //public int getWheelPosition(){
    //    return mDriveMotor.getSelectedSensorPosition();
    //}

    public TalonSRX getAzimuth() {
        return mAzimuthMotor;
    }
    public Spark getDrive(){
       return mDriveMotor; 
    }

    public boolean isInverted() {
        return isInverted;
    }
}