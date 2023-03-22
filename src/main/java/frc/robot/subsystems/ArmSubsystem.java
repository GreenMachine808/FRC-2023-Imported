// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

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

  TalonSRXConfiguration conf2 = new TalonSRXConfiguration();
  /* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
  configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

conf2.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
	/*
		 * status frame rate - user can speed up the position/velocity reporting if need
		 * be.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * status-groups Keep an eye on the CAN bus utilization in the DriverStation if
		 * this is used.
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch18_CommonAPI.html#
		 * can-bus-utilization-error-metrics
		 */
		elevatorL.configFactoryDefault();
		elevatorR.configFactoryDefault();



		elevatorL.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
		//elevatorR.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

		elevatorL.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);


		elevatorL.configNeutralDeadband(0.001, 30);
		//elevatorR.configNeutralDeadband(0.001, 30);


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

		/* Set the peak and nominal outputs */
		elevatorL.configNominalOutputForward(0, 30);
		elevatorL.configNominalOutputReverse(0, 30);
		elevatorL.configPeakOutputForward(0.5, 30);
		elevatorL.configPeakOutputReverse(-0.5, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		elevatorL.selectProfileSlot(0, 0);
		elevatorL.config_kF(0, elevatorLkFF, 30);
		elevatorL.config_kP(0, elevatorLkP, 30);
		elevatorL.config_kI(0, elevatorLkI, 30);
		elevatorL.config_kD(0, elevatorLkD, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		elevatorL.configMotionCruiseVelocity(15000, 30);
		elevatorL.configMotionAcceleration(6000, 30);

		/* Zero the sensor once on robot boot up */
		elevatorL.setSelectedSensorPosition(0, 0, 30);


		/* Factory default hardware to prevent unexpected behavior */
		claw.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		claw.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		claw.configNeutralDeadband(0.001, 30);

		/*
		 * Choose which direction motor should spin during positive
		 * motor-output/sensor-velocity. Note setInverted also takes classic true/false
		 * as an input.
		 */
        claw.setInverted(TalonFXInvertType.CounterClockwise);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */

		/* Brake or coast during neutral */
        claw.setNeutralMode(NeutralMode.Brake);
        
        /* Set relevant frame periods to be at least as fast as periodic rate */
		claw.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 30);
		claw.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 30);

		/* Set the peak and nominal outputs */
		claw.configNominalOutputForward(0, 30);
		claw.configNominalOutputReverse(0, 30);
		claw.configPeakOutputForward(1, 30);
		claw.configPeakOutputReverse(-1, 30);

		/* Set Motion Magic gains in slot0 - see documentation */
		claw.selectProfileSlot(0, 0);
		claw.config_kF(0, clawkFF, 30);
		claw.config_kP(0, clawkP, 30);
		claw.config_kI(0, clawkI, 30);
		claw.config_kD(0, clawkD, 30);

		/* Set acceleration and vcruise velocity - see documentation */
		claw.configMotionCruiseVelocity(10000, 30);
		claw.configMotionAcceleration(10000, 30);

		/* Zero the sensor once on robot boot up */
		claw.setSelectedSensorPosition(0, 0, 30);

		//SmartDashboard.putNumber("claw set point", claw.getSelectedSensorPosition());

		//claw.set(ControlMode.MotionMagic, 0);


	}

    //weightdropper = new Servo(WEIGHT_DROPPER_CHANNEL);

	
	public void setArmPosition(double latSetPos) {
		//double wheelDim = 3.013;
		//Conversion for lateral Set Position to added rotations needed to achieve correct position
		//double rotations = (latSetPos / Math.PI) / (12 * Math.cos(Math.toRadians(18.9)) * wheelDim);
		//double rotations = (latSetPos / Math.cos(Math.toRadians(18.9))) / 12 * (wheelDim * Math.PI);
		
		//double rawPos = (rotations * kUnitsPerRevolution);
		double rawPos = latSetPos;
        elevatorL.set(ControlMode.MotionMagic, 50 * rawPos);
		//elevatorR.set(ControlMode.Follower, elevatorL);

		elevatorR.follow(elevatorL);
        SmartDashboard.putNumber("armSetpoint (rotations)", latSetPos);
        SmartDashboard.putNumber("armSetpoint (encoder ticks)", (latSetPos * FXUnitsPerRevolution));
        SmartDashboard.putNumber("armPosition", elevatorL.getSelectedSensorPosition());
  }

  public void setElevatorOutput(double output) {
	elevatorL.set(ControlMode.PercentOutput, 0.5 * output);
	elevatorR.follow(elevatorL);
}

  public void clawOpen() {
	//int add = 200;
	//if (activate){
	//	add += 100;
	//}
	
	claw.set(ControlMode.MotionMagic, 325);

	}

  public void clawClose() {
	claw.set(ControlMode.MotionMagic, 50);


	}
	public void clawStop() {
	claw.set(ControlMode.MotionMagic, claw.getSelectedSensorPosition());//claw.getSelectedSensorPosition());
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

