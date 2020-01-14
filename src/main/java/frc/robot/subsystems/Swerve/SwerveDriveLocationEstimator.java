package frc.robot.swerve;

import frc.robot.deceiverslib.tasks.ITask;
import frc.robot.deceiverslib.tasks.ITaskRunner;

public class SwerveDriveLocationEstimator implements ITask{
    private static SwerveDriveLocationEstimator mInstance;
    private double mLeftEncoderPrevDistance = 0.0;
    private double mRightEncoderPrevDistance = 0.0;
    private double mPrevTimeStamp = -1.0;
    //private Rotation2d mPreviousHeading = null;

    public SwerveDriveLocationEstimator getInstance(){
        if (mInstance == null){
            mInstance = new SwerveDriveLocationEstimator();
        }

        return mInstance;
    }

    private SwerveDriveLocationEstimator() {}

    @Override
	public void onStart(double timeStamp) {

	}

	@Override
	public void onLoop(double timeStamp) {

	}

	@Override
	public void onStop(double timeStamp) {
        
    }
    
    public void RegisterTaskRunner(ITaskRunner runner){
        runner.register(this);
    }
}
