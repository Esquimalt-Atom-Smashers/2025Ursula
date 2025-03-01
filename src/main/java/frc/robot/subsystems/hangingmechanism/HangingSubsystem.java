package frc.robot.subsystems.hangingmechanism;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HangingSubsystem extends SubsystemBase{
    //Spark Max
    private SparkMax winchMotor = new SparkMax(4, MotorType.kBrushless);
    private SparkMaxConfig winchConfig = new SparkMaxConfig();
    private SparkClosedLoopController winchController = winchMotor.getClosedLoopController();
    private RelativeEncoder winchEncoder = winchMotor.getEncoder();

    //Release Servo
    private Servo servo = new Servo(1); //Check value

    private double servoReleasePosition = 0.5;
    private double servoLatchedPosition = 1;

    private double winchExtendedPosition = 690;
    private double winchRetractedPosition = 0;

    public HangingSubsystem() {
        winchConfig.encoder.positionConversionFactor(1)
        .velocityConversionFactor(1);
        
        winchConfig.inverted(true);

        winchConfig.smartCurrentLimit(1,8,50);

        winchConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1).i(0.0).d(0.0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        winchMotor.configure(winchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        //winchController.setReference(0, SparkMax.ControlType.kPosition);

        servo.setPosition(servoReleasePosition);
    }

    protected void latchServo(){
        servo.setPosition(servoLatchedPosition);
    }
    protected void releaseServo(){
        servo.setPosition(servoReleasePosition);
    }

    protected void retractWinch() {
        winchController.setReference(winchRetractedPosition, ControlType.kPosition);
    }
    protected void extendWinch() {
        winchController.setReference(winchExtendedPosition, ControlType.kPosition);
    }
    protected void setWinchPosition(double position) {
        winchController.setReference(position, ControlType.kPosition);
    }

    //Getters

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }

    //Commands

    public SequentialCommandGroup extendHangingMechanismCommand() {
        return new SequentialCommandGroup(
            new ReleaseServoCommand(true,this),
            new WinchToPositionCommand(this, -18),
            new WinchToPositionCommand(this, winchExtendedPosition)
        );
    }

    public SequentialCommandGroup retractHangingMechanismCommand() {
        return new SequentialCommandGroup(
            new ReleaseServoCommand(false,this),
            new WinchToPositionCommand(this, winchRetractedPosition)  
        );
    }

    public Command manualRetractCommand() {
        return Commands.runOnce(() -> {
            winchController.setReference(-5, ControlType.kVoltage);
            latchServo();
        });    
    }

    public Command stopandZeroMotorCommand() {
        return Commands.runOnce(() -> {
            winchController.setReference(0, ControlType.kVoltage);
            winchMotor.getEncoder().setPosition(0);
            latchServo();
        });
    }

}