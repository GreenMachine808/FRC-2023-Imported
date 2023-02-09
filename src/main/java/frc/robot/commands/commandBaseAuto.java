package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.commands.runShooter;
import frc.robot.subsystems.ShooterSubsystem;
    
import java.time.Instant;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.runIntake;
import frc.robot.commands.runIntakeReverse;
import frc.robot.commands.runShooter;
import frc.robot.delayedAction;
import frc.robot.timedAction;

public class commandBaseAuto extends SequentialCommandGroup{
    private HangSubsystem hang;
    private ShooterSubsystem shooter;
    private SwerveSubsystem drive;
    private Timer timer = new Timer();

    
    public commandBaseAuto(HangSubsystem hang, SwerveSubsystem drive, ShooterSubsystem shooter){
        this.drive = drive;
        this.shooter = shooter;
        this.hang = hang;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        drive.initDrive();


        

        //deadline = new timedAction(10000, () -> {});

    } 

    

    @Override
    public void execute(){

        timer.delay(0.5);
        
        drive.driveSetDistance(-55);
        
        timer.delay(6);
        /* new ParallelDeadlineGroup( 
            new InstantCommand( () -> new Thread(new timedAction(10000, () -> {}) ).start() ), 
            new runIntake(shooter)
            
            );
        */
        /* new InstantCommand(() -> drive.driveSetDistance(-2));
        do {
            drive.drive(0, 0, 0.25);
          } while ((Math.abs(180.0 - drive.gyro.getAngle()) > 1.0) || !(timer.advanceIfElapsed(4)));
          */
    }    

        

    
    @Override
    public boolean isFinished(){
        return true;
    }
    
}
