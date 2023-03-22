/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// Is this proper structure?
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static final).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int ELEVATORFORWARDCHANNEL = 0;
  public static final int ELEVATORREVERSECHANNEL = 1;
  public static final int INTAKEFORWARDCHANNEL = 2;
  public static final int INTAKEREVERSECHANNEL = 3;

  public static final PneumaticsModuleType PNEUMATICSTYPE = PneumaticsModuleType.CTREPCM;

  public static final class SwerveConstants {
    
    // Ratio
    public static final double robotLength = 1;
    public static final double robotWidth = 1;

    // Front Left
    public static final int FRONT_LEFT_ANGLE_MOTOR = 24;
    public static final int FRONT_LEFT_DRIVE_MOTOR = 14;
    public static final double FRONT_LEFT_OFFSET = 0;
    public static final int FRONT_LEFT_ENCODER = 3;

    // Front Right
    public static final int FRONT_RIGHT_ANGLE_MOTOR = 23;
    public static final int FRONT_RIGHT_DRIVE_MOTOR = 13;
    public static final double FRONT_RIGHT_OFFSET = -50;
    public static final int FRONT_RIGHT_ENCODER = 2;

    // Back Right
    public static final int BACK_RIGHT_ANGLE_MOTOR = 22;
    public static final int BACK_RIGHT_DRIVE_MOTOR = 12;
    public static final double BACK_RIGHT_OFFSET = 0;
    public static final int BACK_RIGHT_ENCODER = 1;

    // Back Left
    public static final int BACK_LEFT_ANGLE_MOTOR = 21;
    public static final int BACK_LEFT_DRIVE_MOTOR = 11;
    public static final double BACK_LEFT_OFFSET = -33;
    public static final int BACK_LEFT_ENCODER = 0;

    // Azimuth PID coefficients
    public static final double azimuthkP = 5e-5;
    public static final double azimuthkI = 1e-6;
    public static final double azimuthkD = 0;
    public static final double azimuthkIz = 0;
    public static final double azimuthkFF = 0;
    public static final double azimuthkMaxOutput = 1;
    public static final double azimuthkMinOutput = -1;

    // Drive PID coefficients
    public static final double drivekP = 0.2;
    public static final double drivekI = 0.0;
    public static final double drivekD = 0.0;
    public static final double drivekIz = 0;
    public static final double drivekFF = 0.2;
    public static final double drivekMaxOutput = 1;
    public static final double drivekMinOutput = -1;

    // Smart Motion Coefficients
    // I found max at 5720
    public static final double azimuthMaxVel = 5700; // rpm
    public static final double azimuthMaxAcc = 2500;
    public static final double azimuthMinVel = 0;
    
    public static final int driveTicks = 2048;

    public static final int visionServoPort = 0;

    public static final double visionUpperPosition = 0.8;
    public static final double visionLowerPosition = 0.2;
  }

  public static final class ArmConstants {
    //Claw PID coefficients, for changing if need be
  public static final double clawkP = 0.5;
  public static final double clawkI = 0.005;
  public static final double clawkD = 50;
  public static final double clawIz = 0;
  public static final double clawkFF = 0;
  public static final double clawkMaxOutput = 1;
  public static final double clawkMinOutput = -1;

  //Arm TalonFX Constants
  public static final int FXUnitsPerRevolution = 2048; /* this is constant for Talon FX */

  public static final double elevatorLkP = 0.2;
  public static final double elevatorLkI = 0.0;
  public static final double elevatorLkD = 0.0;
  public static final double elevatorLIz = 0;
  public static final double elevatorLkFF = 0.2;
  public static final double elevatorLkMaxOutput = 1;
  public static final double elevatorLkMinOutput = -1;

  public static final int elevator1 = 6;
  public static final int elevator2 = 5;
  public static final int clawMotor = 4;

  }
  
 


  // THIS IS PROBABLY WRONG

  // PID Constants for intake and shooter
  public static final double shooterkP = 3.0e-4;
  public static final double shooterkI = 0;
  public static final double shooterkD = 0.001;
  public static final double shooterkIz = 0;
  public static final double shooterkFF = 0;
  public static final double shooterkMaxOutput = 1;
  public static final double shooterkMinOutput = -1;
  public static final double shooterMaxRPM = 4000;

  public static final double ballStoragekP = 5e-5;
  public static final double ballStoragekI = 1e-6;
  public static final double ballStoragekD = 0;
  public static final double ballStoragekIz = 0;
  public static final double ballStoragekFF = 0;
  public static final double ballStoragekMaxOutput = 1;
  public static final double ballStoragekMinOutput = -1;

  public static final double[] targetVelocities = {3300, 5000, 5800.0};
  public static final double slowSpeed = 0.15;
  public static final double normalSpeed = 0.35;
  public static final double sprintSpeed = 0.8;
  public static final double fullSpeed = 1;

  public static final double normalTurn = 0.4;




}