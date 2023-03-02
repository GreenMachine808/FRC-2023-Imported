/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.SwerveConstants.*;

/**
 * FRC Team 2090's Swerve Drive Code.
 * 
 * <p>Swerve logic and math is based on Team 2767's <a 
 * href="https://github.com/strykeforce/thirdcoast/">Third Coast Java Libraries </a>
 * 
 * <p>Derivation of inverse kinematic equations are from Ether's <a
 * href="https://www.chiefdelphi.com/media/papers/2426">Swerve Kinematics and Programming</a>.
 *
 */
public class SwerveSubsystem extends SubsystemBase {
  private static final Wheel[] wheels = new Wheel[4];
  private final double[] ws = new double[4]; 
  private final double[] wa = new double[4]; 
  private boolean isFieldOriented;
  public final AHRS gyro;
  private final double kLengthComponent;
  private final double kWidthComponent;

  public boolean runSlow, runSprint, runFull, turnSlow, turnSprint;

  /**
   * This constructs the Swerve Subsystem with the navx and given constants 
   * including the ratio of the robot length to width. 
   */
  public SwerveSubsystem() {
    gyro = new AHRS();
    double radius = Math.hypot(robotLength, robotWidth);
    kLengthComponent = robotLength / radius;
    kWidthComponent = robotWidth / radius;
    setFieldOriented(true);
    generateWheels();
  }

  /**
   * Instantiates the wheels with the given ports for the drive motor, azimuth motor, encoder, and angle offset.
   * Wheels are an array numbered 0-3 from front to back, with even numbers on the left side when facing forward.
   * The wheels are initialized (setting up PID) and zeroed based on the offset.
   */
  public void generateWheels() {
    wheels[0] = new Wheel(FRONT_LEFT_ANGLE_MOTOR, FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_ENCODER, FRONT_LEFT_OFFSET);
    wheels[1] = new Wheel(FRONT_RIGHT_ANGLE_MOTOR, FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_ENCODER, FRONT_RIGHT_OFFSET);
    wheels[2] = new Wheel(BACK_LEFT_ANGLE_MOTOR, BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_ENCODER, BACK_LEFT_OFFSET);
    wheels[3] = new Wheel(BACK_RIGHT_ANGLE_MOTOR, BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_ENCODER, BACK_RIGHT_OFFSET);
    
    for (Wheel wheel : wheels) {
      wheel.initWheel();
      wheel.zero();
    }
  }

  /**
   * Returns an array of Wheels
   */
  public Wheel[] getWheels() {
    return wheels;
  }

  /**
   * 
   * Drive the robot in given field-relative direction and with given rotation.
   *
   * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
   * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
   * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
   */
  public void drive(double forward, double strafe, double yaw) {

    isFieldOriented = SmartDashboard.getBoolean("driveMode", true);
    /* If the robot is field oriented, the inputed values are modified to 
     * be with respect to the zero of the field.
    */
    if (isFieldOriented) {
      double angle = gyro.getAngle();
      SmartDashboard.putNumber("Gyro", gyro.getAngle());
      SmartDashboard.putNumber("Pitch", gyro.getPitch());
      SmartDashboard.putNumber("Yaw", gyro.getYaw());
      SmartDashboard.putNumber("Roll", gyro.getRoll());

      angle += gyro.getRate();
      angle = Math.IEEEremainder(angle, 360.0);

      // This is rotate by vector, the proof of which comes from the trig definitions for sin(a + b) and cos(a + b)
      angle = Math.toRadians(angle);
      final double temp = forward * Math.cos(angle) + strafe * Math.sin(angle);
      strafe = strafe * Math.cos(angle) - forward * Math.sin(angle);
      forward = temp;
    }

    final double a = strafe - yaw * kLengthComponent;
    final double b = strafe + yaw * kLengthComponent;
    final double c = forward - yaw * kWidthComponent;
    final double d = forward + yaw * kWidthComponent;

    // wheel speed
    ws[0] = Math.hypot(b, d);
    ws[1] = Math.hypot(b, c);
    ws[2] = Math.hypot(a, d);
    ws[3] = Math.hypot(a, c);

    // wheel yaw
    wa[0] = Math.atan2(b, d) * 0.5 / Math.PI;
    wa[1] = Math.atan2(b, c) * 0.5 / Math.PI;
    wa[2] = Math.atan2(a, d) * 0.5 / Math.PI;
    wa[3] = Math.atan2(a, c) * 0.5 / Math.PI;

    // normalize wheel speed
    final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
    if (maxWheelSpeed > 1.0) {
      for (int i = 0; i < 4; i++) {
        ws[i] /= maxWheelSpeed;
      }
    }
    // set wheels
    for (int i = 0; i < 4; i++) {
      wheels[i].set(wa[i], ws[i]);
    }   
  }

  /**
   * Set the drive mode for the robot
   *
   * @param enable true = field oriented driving, false = robot oriented
   */
  public void setFieldOriented(boolean enable) {
    isFieldOriented = enable;
    SmartDashboard.putBoolean("driveMode", enable);
  }

  /**
   * Set the wheels at a given target angle
   * @param angle the target angle for all swerve modules
   */
  public void setAllAzimuth(double angle, double delay) {
    for (Wheel wheel : wheels) {
      wheel.setTargetAngle(angle);
    }
    Timer.delay(delay);
  }

  /**
   * Drive a set distance
   * @param distance the target distance for the modules to drive
   */
  public void driveSetDistance(double distance, double delay) {
    for (Wheel wheel : wheels) {
      wheel.setTargetDistance(distance);
    }
  }

  public void gyroBalance() {
    double xVelocity = 0;
    double yVelocity = 0;

    if (gyro.getRoll() > -4 + 3) {
      xVelocity = -0.1;
    }
    else if (gyro.getRoll() < -4 - 3) {
      xVelocity = 0.1;
    }
    if (gyro.getPitch() > 3) {
      yVelocity = -0.1;
    }
    else if (gyro.getPitch() < -3) {
      yVelocity = 0.1;
    }
    drive(xVelocity, yVelocity, 0);
  }

  public void gyroBalanceAuto(double timeout) {
    double xVelocity = 0;
    double yVelocity = 0;
    long timeStamp = System.currentTimeMillis();

    do {
      if (gyro.getRoll() > -4 + 3) {
        xVelocity = -0.1;
      }
      while (gyro.getRoll() < -4 - 3) {
        xVelocity = 0.1;
      }
      while (gyro.getPitch() > 3) {
        yVelocity = -0.1;
      }
      while (gyro.getPitch() < -3) {
        yVelocity = 0.1;
      }
      drive(xVelocity, yVelocity, 0);
    } 
    while (xVelocity != 0 && yVelocity != 0 && System.currentTimeMillis() < timeStamp + timeout*1000);
  }

  /**
   * Stop all Swerve Modules
  */
  public void stop() {
    for (Wheel wheel : wheels) {
      wheel.stop();
    }
  }

  /**
   * Init all Swerve Modules in the zeroed position (azimuth position and drive output are set to 0)
   * Camera mode is set to normal (not vision)
   */
  public void initDrive() {
    gyro.zeroYaw();
    for (Wheel wheel : wheels) {
      wheel.zero();
    }
  }
}