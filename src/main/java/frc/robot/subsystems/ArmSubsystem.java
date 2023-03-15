// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.clawMotor;
import static frc.robot.Constants.elevatorMotor;
import static frc.robot.Constants.kUnitsPerRevolution;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // Create variables specific to the subsystem, as well as the devices (new Motor m_motor)
  //public final DoubleSolenoid elevator;
  public final TalonSRX elevator;
  public final CANSparkMax claw;
  //public final Servo weightdropper;
  //public final Compressor phCompressor = new Compressor(PNEUMATICSTYPE);

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

    elevator = new TalonSRX(elevatorMotor);
	claw = new CANSparkMax(clawMotor, MotorType.kBrushless);

    //weightdropper = new Servo(WEIGHT_DROPPER_CHANNEL);
   /*
	 * Talon FX has 2048 units per revolution
	 * 
	 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
	 */

	/**
	 * Decide if positive motor-output/sensor-velocity should be when motor spins
	 * clockwise or counter-clockwise.
	 */
	final TalonFXInvertType kInvertType = TalonFXInvertType.CounterClockwise; // <<< What direction you want "forward/up" to be.

	/** electic brake during neutral */
	final NeutralMode kBrakeDurNeutral = NeutralMode.Brake;
  final NeutralMode kCoastDurNeutral = NeutralMode.Coast;


    /** print every few loops */
	int _loops = 0;

	/* newer config API */
  TalonFXConfiguration configs = new TalonFXConfiguration();
  /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
  configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
  /* config all the settings */
  //elevator.configAllSettings(configs);
  //claw.configAllSettings(configs);

    /*
		 * status frame rate - user can speed up the position/velocity reporting if need
		 * be.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * status-groups Keep an eye on the CAN bus utilization in the DriverStation if
		 * this is used.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * can-bus-utilization-error-metrics
		 */
		elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    //claw.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);


		/*
		 * choose which direction motor should spin during positive
		 * motor-output/sensor-velocity. Note setInverted also takes classic true/false
		 * as an input.
		 */
		//elevator.setInverted(kInvertType);
    //claw.setInverted(kInvertType);

		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
		//_talon.setSensorPhase(true);

		/* brake or coast during neutral */
		elevator.setNeutralMode(kBrakeDurNeutral);
    //claw.setNeutralMode(kBrakeDurNeutral);

	

	}
	public void setArmPosition(double latSetPos) {
		double wheelDim = 3.013;
		//Conversion for lateral Set Position to added rotations needed to achieve correct position
		//double rotations = (latSetPos / Math.PI) / (12 * Math.cos(Math.toRadians(18.9)) * wheelDim);
		double rotations = (latSetPos / Math.cos(Math.toRadians(18.9))) / 12 * (wheelDim * Math.PI);
		
		double rawPos = (rotations * kUnitsPerRevolution);
        elevator.set(ControlMode.MotionMagic, rawPos);
        SmartDashboard.putNumber("armSetpoint (rotations)", latSetPos);
        SmartDashboard.putNumber("armSetpoint (encoder ticks)", (latSetPos * kUnitsPerRevolution));
        SmartDashboard.putNumber("armPosition", elevator.getSelectedSensorPosition());
  }

  public void clawOpen() {
	//claw.set(ControlMode.MotionMagic, kUnitsPerRevolution * 0.2);
	}

  public void clawClose() {
	//claw.set(ControlMode.MotionMagic, kUnitsPerRevolution * 0);
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
