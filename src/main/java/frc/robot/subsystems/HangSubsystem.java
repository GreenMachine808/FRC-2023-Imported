// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Compressor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class HangSubsystem extends SubsystemBase {

  // Create variables specific to the subsystem, as well as the devices (new Motor m_motor)
  public final DoubleSolenoid elevator;
  public final CANSparkMax climb_l;
  public final CANSparkMax climb_r;
  public final Servo weightdropper;
  public final Compressor phCompressor = new Compressor(PNEUMATICSTYPE);

  /** Creates a new ExampleSubsystem. */
  public HangSubsystem() {
    
    elevator = new DoubleSolenoid(
      PNEUMATICSTYPE,
      ELEVATORFORWARDCHANNEL,
      ELEVATORREVERSECHANNEL);

    climb_l = new CANSparkMax(LEFT_CLIMB_MOTOR, MotorType.kBrushless);
    climb_r = new CANSparkMax(RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
    weightdropper = new Servo(WEIGHT_DROPPER_CHANNEL);
    
    // Maybe this work?
    elevator.set(Value.kReverse);
  }

  public void setElevatorState(Value state) { elevator.set(state); }
  public void toggleElevator() { elevator.toggle(); }

  public void moveElevator(double movement) {
    
    climb_l.set(-movement);
    climb_r.set(movement);
  }

  public void popWeightServo(boolean down) {
    if (down) {
      weightdropper.setAngle(90);
    } else {
      weightdropper.setAngle(180);
    }
  }


  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
