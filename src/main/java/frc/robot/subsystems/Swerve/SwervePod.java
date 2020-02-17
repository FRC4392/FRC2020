package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Preferences;


public class SwervePod {
    private static final int TICKS_PER_ROTATION = 360;
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mAzimuthMotor;
    private final Preferences mRobotPreferences;
    CANEncoder mEncoder;
    CANPIDController mPIDController;
    CANCoder mCanCoder;
    private boolean isInverted = false;
    private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setpoint;

    public SwervePod(CANSparkMax drive, CANSparkMax azimuth, CANCoder canCoder) {
        this.mDriveMotor = drive;
        this.mAzimuthMotor = azimuth;
        this.mCanCoder = canCoder;

        setpoint = 0.0;

        mAzimuthMotor.getEncoder().setPositionConversionFactor(25.08);

        mDriveMotor.setInverted(true);

        mRobotPreferences = Preferences.getInstance();

        mEncoder = mAzimuthMotor.getEncoder();
        mPIDController = mAzimuthMotor.getPIDController();

        kP = mRobotPreferences.getDouble("SwerveKP", 1);
        kI = mRobotPreferences.getDouble("SwerveKI", 0.0);
        kD = mRobotPreferences.getDouble("SwerveKD", 0.0);
        kIz = mRobotPreferences.getDouble("SwerveKIz", 0.0);
        kFF = mRobotPreferences.getDouble("SwerveKFF", 0.0);
        kMaxOutput = mRobotPreferences.getDouble("SwerveKMaxOutput", 1.0);
        kMinOutput = mRobotPreferences.getDouble("SwerveKMinOutput", -1.0);
        
        maxRPM = 5700.0;

        mPIDController.setP(kP);
        mPIDController.setI(kI);
        mPIDController.setD(kD);
        mPIDController.setIZone(kIz);
        mPIDController.setFF(kFF);
        mPIDController.setOutputRange(kMinOutput, kMaxOutput);

        setAzimuthZero();
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
        setpoint = azimuthError + azimuthPosition;
        mPIDController.setReference(setpoint, ControlType.kPosition);
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

    public void setAzimuthZero() {
        //calculate position to increments
        double position = mCanCoder.getAbsolutePosition();

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
    
    public double getSetpoint(){
        return setpoint;
    }

    public double getIncrementalPosition(){
        return mEncoder.getPosition();
    }
}