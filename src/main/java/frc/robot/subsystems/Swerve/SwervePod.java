package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;


public class SwervePod {
    private static final int TICKS_PER_ROTATION = 360;
    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mAzimuthMotor;
    private final Preferences mRobotPreferences;
    CANEncoder mAzimuthEncoder;
    CANEncoder mDriveEncoder;
    CANPIDController mPIDController;
    CANCoder mAbsoluteEncoder;
    private boolean isInverted = false;
    private Double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, setpoint;

    public SwervePod(CANSparkMax drive, CANSparkMax azimuth, CANCoder canCoder) {
        this.mDriveMotor = drive;
        this.mAzimuthMotor = azimuth;
        this.mAbsoluteEncoder = canCoder;

        setpoint = 0.0;

        mAzimuthMotor.getEncoder().setPositionConversionFactor(25.08);

        mDriveEncoder.getEncoder().setSpeed(0);

        mDriveMotor.setInverted(true);

        mRobotPreferences = Preferences.getInstance();

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveEncoder.getEncoder();
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
        
        double azimuthPosition = mAzimuthEncoder.getPosition();
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


    public void stop(){
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);

    }

    public void setAzimuthZero() {
        //calculate position to increments
        double position = mAbsoluteEncoder.getAbsolutePosition();

        mAzimuthEncoder.setPosition(position);
    }

    public double getAzimuthAbsolutePosition(){
        return mAbsoluteEncoder.getAbsolutePosition();
    }

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
        return mAzimuthEncoder.getPosition();
    }

    public SwerveModuleState getState(){
        SwerveModuleState state = new SwerveModuleState();
        state.angle = Rotation2d.fromDegrees(mAzimuthEncoder.getPosition());
        state.speedMetersPerSecond = 0; //this needs to be updated to get the speed from the robot.
        return new SwerveModuleState();
    }
}