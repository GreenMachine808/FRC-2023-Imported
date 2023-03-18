// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.commands.runIntake;
//import frc.robot.commands.runShooter;

public class ShooterSubsystem extends SubsystemBase {
  
  // Create variables specific to the subsystem, as well as the devices (new Motor m_motor)
  //private final CANSparkMax conveyor_m, shooter1_m, shooter2_m;
  //private final RelativeEncoder shooter_e;
  //private final SparkMaxPIDController shooter_mc, conveyor_mc;
  //private final TalonSRX intake_m;
  //private final DoubleSolenoid intake_p;
  //private final Command runShooterCo = new runShooter(this);
  //private final Command runIntakeCo = new runIntake(this);
  
  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {

    // Set value to variables and other initialization here
    //conveyor_m = new CANSparkMax(CONVEYOR_MOTOR, MotorType.kBrushed);
    //conveyor_mc = conveyor_m.getPIDController();
    //conveyor_m.restoreFactoryDefaults();

    //shooter1_m = new CANSparkMax(SHOOTER_1_MOTOR, MotorType.kBrushless);
    //shooter2_m = new CANSparkMax(SHOOTER_2_MOTOR, MotorType.kBrushless);
    //shooter_mc = shooter1_m.getPIDController();
    //shooter1_m.restoreFactoryDefaults();
    //shooter2_m.restoreFactoryDefaults();
    //shooter1_m.setInverted(true);

    //shooter_e = shooter1_m.getEncoder(); // To display values (?)

    //intake_m = new TalonSRX(INTAKE_MOTOR);
    /* intake_p = new DoubleSolenoid(PNEUMATICSTYPE, INTAKEFORWARDCHANNEL,
      INTAKEREVERSECHANNEL);
      */


    
    // PID Constants. CURRENTLY YOINKED FROM 2020 CODE
    /*shooter_mc.setP(shooterkP);
    shooter_mc.setI(shooterkI);
    shooter_mc.setD(shooterkD);
    shooter_mc.setIZone(shooterkIz);
    shooter_mc.setFF(shooterkFF);
    shooter_mc.setOutputRange(shooterkMinOutput, shooterkMaxOutput);*/
    //shooter2_m.follow(shooter1_m, true);

    /*conveyor_mc.setP(ballStoragekP);
    conveyor_mc.setI(ballStoragekI);
    conveyor_mc.setD(ballStoragekD);
    conveyor_mc.setIZone(ballStoragekIz);
    conveyor_mc.setFF(ballStoragekFF);
    conveyor_mc.setOutputRange(ballStoragekMinOutput, ballStoragekMaxOutput);*/

    //intake_p.set(DoubleSolenoid.Value.kReverse);
  }
  /*
  // method for pneumatics activation
  public void setIntakeArm(DoubleSolenoid.Value value) { intake_p.set(value); }
  
  // method for running conveyor belt
  public void runConveyorBelt(boolean isBackwards) { conveyor_m.set(isBackwards==true? 1 : -1); }

  public void runConveyorBeltTwo(boolean isBackwards) { conveyor_m.set(isBackwards==true? 0.7 : -0.2); }
  
  // method for running intake motor
  public void runIntakeMotor(boolean isBackwards) { intake_m.set(ControlMode.PercentOutput, isBackwards==true? -1 : 1); }

  public void runIntakeMotorTwo(boolean isBackwards) { intake_m.set(ControlMode.PercentOutput, isBackwards==true? -1 : 0.7); }
  
  // method for running shooter motor
  public void runShooter(double setPosition) {
    // Until the PID settings get configured, this is what we are dealing with.
    shooter1_m.set(setPosition);
    //shooter_mc.setReference(targetVelocities[setPosition], ControlType.kVelocity);
    SmartDashboard.putNumber("Velocity", shooter_e.getVelocity());
    SmartDashboard.putNumber("Output", shooter1_m.getAppliedOutput());
  }

  public Command getRunShooter(){
    return runShooterCo;
  }

  public Command getRunIntake(){
    return runIntakeCo;
  }

  // HALT.
  public void stop() {
    shooter1_m.set(0);
    shooter2_m.set(0);
    conveyor_m.set(0);
    intake_m.set(ControlMode.PercentOutput, 0);
  }
 */
}
