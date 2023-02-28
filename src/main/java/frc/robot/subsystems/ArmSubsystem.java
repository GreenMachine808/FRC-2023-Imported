// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Compressor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ArmSubsystem extends SubsystemBase {

  // Create variables specific to the subsystem, as well as the devices (new Motor m_motor)
  //public final DoubleSolenoid elevator;
  public final WPI_TalonFX elevator;
  public final WPI_TalonFX claw;
  //public final Servo weightdropper;
  //public final Compressor phCompressor = new Compressor(PNEUMATICSTYPE);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

    elevator = new WPI_TalonFX(elevatorMotor, "elevator");
    claw = new WPI_TalonFX(clawMotor, "claw");
    //weightdropper = new Servo(WEIGHT_DROPPER_CHANNEL);
    final TalonFXInvertType kFxInvertType = TalonFXInvertType.CounterClockwise;
    final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;



  }

  

  

  /* 
  public void popWeightServo(boolean down) {
    if (down) {
      weightdropper.setAngle(90);
    } else {
      weightdropper.setAngle(180);
    }
  }
*/

  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}
