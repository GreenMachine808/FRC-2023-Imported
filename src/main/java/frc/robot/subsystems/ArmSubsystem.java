// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.clawMotor;
import static frc.robot.Constants.elevator1;
import static frc.robot.Constants.elevator2;

import static frc.robot.Constants.kUnitsPerRevolution;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // Create variables specific to the subsystem, as well as the devices (new Motor m_motor)
  //public final DoubleSolenoid elevator;
  public final TalonSRX elevatorL;
  public final TalonSRX elevatorR;

  public final TalonFX claw;
  //public final Servo weightdropper;
  //public final Compressor phCompressor = new Compressor(PNEUMATICSTYPE);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

	
    elevatorL = new TalonSRX(elevator1);
	elevatorR = new TalonSRX(elevator2);

	claw = new TalonFX(clawMotor);
	initElevator();
		
}
public void initElevator(){
	final TalonFXInvertType Counter = TalonFXInvertType.CounterClockwise;
	final TalonFXInvertType Clock = TalonFXInvertType.Clockwise;



	/** electic brake during neutral */
	final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;
	  final NeutralMode kCoastDurNeutral = NeutralMode.Coast;

	/* newer config API */
  TalonFXConfiguration configs = new TalonFXConfiguration();
  /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
  configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
  
	/*
		 * status frame rate - user can speed up the position/velocity reporting if need
		 * be.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * status-groups Keep an eye on the CAN bus utilization in the DriverStation if
		 * this is used.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * can-bus-utilization-error-metrics
		 */
		elevatorL.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		elevatorR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

		elevatorL.configNeutralDeadband(0.001, 30);
		elevatorR.configNeutralDeadband(0.001, 30);


		/*
		 * choose which direction motor should spin during positive
		 * motor-output/sensor-velocity. Note setInverted also takes classic true/false
		 * as an input.
		 */
		//elevatorL.setInverted(true);
		//elevatorR.setInverted(invert: true);

		/* brake or coast during neutral */
		elevatorL.setNeutralMode(kBrakeDurNeutral);
		elevatorR.setNeutralMode(kBrakeDurNeutral);

		claw.setNeutralMode(kBrakeDurNeutral);
	}

    //weightdropper = new Servo(WEIGHT_DROPPER_CHANNEL);

	
	public void setArmPosition(double latSetPos) {
		double wheelDim = 3.013;
		//Conversion for lateral Set Position to added rotations needed to achieve correct position
		//double rotations = (latSetPos / Math.PI) / (12 * Math.cos(Math.toRadians(18.9)) * wheelDim);
		double rotations = (latSetPos / Math.cos(Math.toRadians(18.9))) / 12 * (wheelDim * Math.PI);
		
		double rawPos = (rotations * kUnitsPerRevolution);
        elevatorL.set(ControlMode.MotionMagic, rawPos);
		elevatorR.set(ControlMode.MotionMagic, elevatorL.getSelectedSensorPosition());
        SmartDashboard.putNumber("armSetpoint (rotations)", latSetPos);
        SmartDashboard.putNumber("armSetpoint (encoder ticks)", (latSetPos * kUnitsPerRevolution));
        SmartDashboard.putNumber("armPosition", elevatorL.getSelectedSensorPosition());
  }

  public void setElevatorOutput(double output) {
	elevatorL.set(ControlMode.PercentOutput, output);
	elevatorR.set(ControlMode.PercentOutput, output);
}

  public void clawOpen() {
	claw.set(ControlMode.PercentOutput, 0.25);
	}

  public void clawClose() {
	claw.set(ControlMode.PercentOutput, -0.25);
	}
	public void clawStop() {
	claw.set(ControlMode.PercentOutput, 0);
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

