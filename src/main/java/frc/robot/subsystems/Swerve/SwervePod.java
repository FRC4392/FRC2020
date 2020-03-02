package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;

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
    private String name = "";

    public SwervePod(CANSparkMax drive, CANSparkMax azimuth, CANCoder canCoder, String name) {
        this.mDriveMotor = drive;
        this.mAzimuthMotor = azimuth;
        this.mAbsoluteEncoder = canCoder;

        this.name = name;

        mDriveMotor.restoreFactoryDefaults();
        mAzimuthMotor.restoreFactoryDefaults();

        setpoint = 0.0;

        mAzimuthMotor.getEncoder().setPositionConversionFactor(25.08);
        mAzimuthMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(10);

        mDriveMotor.setInverted(true);

        mRobotPreferences = Preferences.getInstance();

        mAzimuthEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();
        mPIDController = mAzimuthMotor.getPIDController();

        mDriveEncoder.setVelocityConversionFactor(4.712);

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

        mAzimuthMotor.burnFlash();
        mDriveMotor.burnFlash();

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

  public void set(Rotation2d rotation, double speed){

      double azimuthPosition = mAzimuthEncoder.getPosition();
      double azimuthError = Math.IEEEremainder(rotation.getDegrees() - azimuthPosition, TICKS_PER_ROTATION);

      isInverted = Math.abs(azimuthError) > 0.25 * TICKS_PER_ROTATION;
      if (isInverted) {
          azimuthError -= Math.copySign(0.5 * TICKS_PER_ROTATION, azimuthError);
          speed = -speed;
      }
      setpoint = azimuthError + azimuthPosition;
      mPIDController.setReference(setpoint, ControlType.kPosition);
      mDriveMotor.set(speed);
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
        double position = getAzimuthAbsolutePosition();
        System.out.println("Setting Absolute Zero of Module: " + name + "; Absolute: " + position + " Incremental: " + mAzimuthEncoder.getPosition());
        CANError err = mAzimuthEncoder.setPosition(position);
        System.out.println(err.toString());
    }

    public void checkAzimuthZero() {
        if (Math.abs((mAzimuthEncoder.getPosition()%180.0) - mAbsoluteEncoder.getAbsolutePosition()) > 10){
            setAzimuthZero();
        }
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

    public double getVelocity() {
            return mDriveEncoder.getVelocity();
    }

    public SwerveModuleState getState(){
        SwerveModuleState state = new SwerveModuleState();
        state.angle = Rotation2d.fromDegrees(mAzimuthEncoder.getPosition());
        state.speedMetersPerSecond = getVelocity(); //this needs to be updated to get the speed from the robot.
        return state;
    }
}