package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerIndexCommand extends CommandBase {
    private Indexer mIndexer;

    public IndexerIndexCommand(Indexer indexer){
        mIndexer = indexer;
        addRequirements(mIndexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (mIndexer.getStartEye() &! mIndexer.getEndEye()){
            mIndexer.setSpeed(-.35);
        } else {
            mIndexer.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
