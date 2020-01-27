package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;


public class SwervePod {
    private static final int TICKS_PER_ROTATION = 4096;
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mAzimuthMotor;
    CANEncoder mEncoder;
    CANPIDController mPIDController;
    CANCoder mCanCoder;
    private boolean isInverted = false;

    public SwervePod(CANSparkMax drive, CANSparkMax azimuth, CANCoder canCoder) {
        this.mDriveMotor = drive;
        this.mAzimuthMotor = azimuth;
        this.mCanCoder = canCoder;

        mEncoder = mAzimuthMotor.getEncoder();
        mPIDController = mAzimuthMotor.getPIDController();

        /**zimuth.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        azimuth.configNominalOutputForward(0);
        azimuth.configNominalOutputReverse(0);
        azimuth.configPeakOutputForward(1);
        azimuth.configPeakOutputReverse(-1);

        azimuth.selectProfileSlot(0, 0);
        azimuth.configMotionCruiseVelocity(2000);
        azimuth.configMotionAcceleration(20000);
        azimuth.config_kF(0, 0.51);
        azimuth.config_kP(0, 7);
        azimuth.setSensorPhase(true);**/
    }
    
    public void set(double angle, double driveSpeed){

        angle *= TICKS_PER_ROTATION;
        
        double azimuthPosition = mEncoder.getPosition();
        double azimuthError = Math.IEEEremainder(angle - azimuthPosition, TICKS_PER_ROTATION);

        isInverted = Math.abs(azimuthError) > 0.25 * TICKS_PER_ROTATION;
        if (isInverted) {
            azimuthError -= Math.copySign(0.5 * TICKS_PER_ROTATION, azimuthError);
            driveSpeed = -driveSpeed;
        }
        mPIDController.setReference(azimuthError + azimuthPosition, ControlType.kPosition);
        mDriveMotor.set(driveSpeed);
  }
    
    public void setOpenLoop(double azimuthSpeed, double driveSpeed){
        mDriveMotor.set(driveSpeed);
        mAzimuthMotor.set(azimuthSpeed);

    }

    public void setAzimuthPosition(int position) {
        mPIDController.setReference(position , ControlType.kPosition);
    }

    public void setAzimuthOpenLoop(double speed){
        mAzimuthMotor.set(speed);
    }

    public void setWheelOpenLoop(double speed){
        mDriveMotor.set(speed);
    }

    //public void setWheelVelocity(double speed){
    //    mDriveMotor.set(ControlMode.Velocity, speed);
    //}

    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);

    }

    public void setAzimuthZero(int zero) {
        double azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
        //calculate position to increments
        //672PPR
        double cpr = 672.0/360.0;
        double position = azimuthSetpoint * cpr;

        mEncoder.setPosition(position);
    }

    public double getAzimuthAbsolutePosition(){
        return mCanCoder.getAbsolutePosition();
    }

    //public void setWheelPosition(int position){
    //    mDriveMotor.setSelectedSensorPosition(position);
    //}

    //public int getWheelPosition(){
    //    return mDriveMotor.getSelectedSensorPosition();
    //}

    public CANSparkMax getAzimuth() {
        return mAzimuthMotor;
    }
    public CANSparkMax getDrive(){
       return mDriveMotor; 
    }

    public boolean isInverted() {
        return isInverted;
    }
}