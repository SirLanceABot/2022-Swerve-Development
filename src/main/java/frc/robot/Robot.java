// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{
  private final XboxController controller = new XboxController(0);
  private final Drivetrain Drivetrain = new Drivetrain();

  // TODO: Change to custom slewrate limiters or something to reduce "lag"
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public static BufferedWriter bw;
  public static Timer time = new Timer();

  private static double volts = 0.0;

  @Override
  public void robotInit()
  {
    // TODO Check for faults on all devices (Talon, PDP, etc.)

    try 
    {
      String filename = "/home/lvuser/characteristic.csv";
      bw = new BufferedWriter(new FileWriter(new File(filename)));
      System.out.println("writing data collected to roboRIO > " + filename + "<");    
    } 
    catch (Exception e)
    {
      e.printStackTrace();
    }

    SmartDashboard.putNumber("Turn P", 0.0);
    SmartDashboard.putNumber("Turn D", 0.0);
    SmartDashboard.putNumber("Turn angle", 0.0);
    SmartDashboard.putNumber("Drive P", 0.0);
    SmartDashboard.putNumber("Drive D", 0.0);
    SmartDashboard.putNumber("Drive speed", 0.0);

    System.out.println(Constants.FRONT_LEFT);
    System.out.println(Constants.FRONT_RIGHT);
    System.out.println(Constants.BACK_LEFT);
    System.out.println(Constants.BACK_RIGHT);
  }

  @Override
  public void robotPeriodic()
  {
    Drivetrain.printTurnEncoderPosition();
    Drivetrain.printNavX();    
  }

  @Override
  public void disabledInit()
  {
    try
    {
      Robot.bw.flush();
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }
  }

  @Override
  public void disabledPeriodic()
  {

  }

  @Override
  public void testInit()
  {

  }

  @Override
  public void testPeriodic()
  {

  }

  @Override
  public void autonomousInit()
  {
    time.reset();
    time.start();
    Drivetrain.resetEncoders();
    volts = 0.0;
    // swerve.drive(3.0, 0.0, 0.0, false);
    // driveWithJoystick(false);
    // swerve.updateOdometry();
  }

  @Override
  public void autonomousPeriodic()
  {
    volts = 10.0;
    // swerve.setMotorSpeeds(0.0, volts / Constants.MAX_BATTERY_VOLTAGE);
    // System.out.println("norm volts = " + volts + "   rate = " + swerve.getTurnEncoderRate());
    double driveSpeed = SmartDashboard.getNumber("Drive speed", 0.0);
    Drivetrain.drive(driveSpeed, 0.0, 0.0, true);
    // driveWithJoystick(false);
    // swerve.updateOdometry();
  }

  @Override
  public void teleopInit()
  {
    Drivetrain.resetEncoders();
  }

  @Override
  public void teleopPeriodic()
  {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative)
  {
    double powerLimit = 0.6;
    double leftTriggerPull = 0.0;

    double yLeft = -controller.getLeftY();
    double xLeft = -controller.getLeftX();
    double xRight = -controller.getRightX();
    double deadbandLeft = 0.15;
    double deadbandRight = 0.18;

    // System.out.printf("yLeft = %f, xLeft = %f, xRight = %f\n", yLeft, xLeft, xRight);
    yLeft = (Math.abs(yLeft) <= deadbandLeft) ? 0.0 : (yLeft - Math.signum(yLeft)*deadbandLeft) / (1.0 - deadbandLeft);
    xLeft = (Math.abs(xLeft) <= deadbandLeft) ? 0.0 : (xLeft - Math.signum(xLeft)*deadbandLeft) / (1.0 - deadbandLeft);
    xRight = (Math.abs(xRight) <= deadbandRight) ? 0.0 : (xRight - Math.signum(xRight)*deadbandRight) / (1.0 - deadbandRight);
    
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = xspeedLimiter.calculate(yLeft) * Constants.MAX_DRIVE_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = yspeedLimiter.calculate(xLeft) * Constants.MAX_DRIVE_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    // FIXME Changing radians to degrees, changed the constant in order to work
    double rot = rotLimiter.calculate(xRight) * Constants.MAX_ROBOT_TURN_SPEED;

    // FIXME Slewrate limiter tuning as it makes delay that can be used for equipment safety and driveability, 3 is probably to slow for deceleration as it feels sluggish

    // Scales down the input power
    if (controller.getLeftBumper())
    {
      powerLimit = 1.0;
    }

    powerLimit += controller.getLeftTriggerAxis() * (1.0 - powerLimit);

    xSpeed *= powerLimit;
    ySpeed *= powerLimit;
    rot *= powerLimit;

    // swerve.setMotorSpeeds(yLeft / 2.0, xRight / 5.0);
    Drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
    // System.out.printf("xSpeed = %f, ySpeed = %f, rot = %f\n", xSpeed, ySpeed, rot);
    // swerve.printNavX();
  }

}