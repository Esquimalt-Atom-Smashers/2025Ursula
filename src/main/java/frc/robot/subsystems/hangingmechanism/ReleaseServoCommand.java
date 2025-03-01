package frc.robot.subsystems.hangingmechanism;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ReleaseServoCommand extends Command {
    private HangingSubsystem hangingSubsystem;
    private boolean release;
    private Timer timer = new Timer();

    public ReleaseServoCommand(boolean release,HangingSubsystem hangingSubsystem) {
        this.hangingSubsystem = hangingSubsystem;
        this.release = release;
        addRequirements(hangingSubsystem);
    }


    @Override
    public void initialize() {
        if (release){
            hangingSubsystem.releaseServo();
        }else{
            hangingSubsystem.latchServo();
        }
        
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }
}
