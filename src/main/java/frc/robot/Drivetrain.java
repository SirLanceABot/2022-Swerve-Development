// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Fidelity;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain //extends RobotDriveBase
{
//   public static final double kMaxSpeed = 3.0; // 3 meters per second
//   public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
//   public static final double kInchesToMeters = 0.0254;
//   //inches, distance from center, robot is a square so length and width are the same
//   public static final double kRobotWidth = 23.5; //inches, y-coordinate
//   public static final double kRobotLength = 23.5; //inches, x-coordinate

  private static final Translation2d m_frontLeftLocation = new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2, Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d m_frontRightLocation = new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2, -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d m_backLeftLocation = new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2, Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d m_backRightLocation = new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2, -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);

  private static final SwerveModule m_frontLeft = new SwerveModule(Constants.FRONT_LEFT);
  private static final SwerveModule m_frontRight = new SwerveModule(Constants.FRONT_RIGHT);
  private static final SwerveModule m_backLeft = new SwerveModule(Constants.BACK_LEFT);
  private static final SwerveModule m_backRight = new SwerveModule(Constants.BACK_RIGHT);

  private static final AHRS m_navx = new AHRS(SerialPort.Port.kUSB);

  private static final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private static final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d());

  // TODO: Make final by setting to an initial stopped state
  private static SwerveModuleState[] previousSwerveModuleStates = null;

  public Drivetrain()
  {
    m_navx.reset();
    
    // System.out.println(m_frontLeft.getTurningEncoderPosition());
    // System.out.println(m_frontRight.getTurningEncoderPosition());
    // System.out.println(m_backLeft.getTurningEncoderPosition());
    // System.out.println(m_backRight.getTurningEncoderPosition());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
  {
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] swerveModuleStates;

    if(fieldRelative)
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
    swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    // var swerveModuleStates =
    //     m_kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_navx.getRotation2d())
    //             : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_DRIVE_SPEED);
    printDesiredStates(swerveModuleStates);

    // double ang = SmartDashboard.getNumber("Turn angle", 0.0);
    // for(int i = 0; i < swerveModuleStates.length; i++)
    // {
    //   swerveModuleStates[i].speedMetersPerSecond = 0.0;
    //   swerveModuleStates[i].angle = new Rotation2d(Math.toRadians(ang));
    // }

    if(xSpeed == 0 && ySpeed == 0 && rot == 0 && previousSwerveModuleStates != null)
    {
      for(int i = 0; i < swerveModuleStates.length; i++)
      {
        swerveModuleStates[i].angle = previousSwerveModuleStates[i].angle;
      }
    }
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    previousSwerveModuleStates = swerveModuleStates;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry()
  {
    m_odometry.update(
        m_navx.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void setMotorSpeeds(double driveSpeed, double turnSpeed)
  {
    m_frontLeft.setMotorSpeeds(driveSpeed, turnSpeed);
    m_frontRight.setMotorSpeeds(driveSpeed, turnSpeed);
    m_backLeft.setMotorSpeeds(driveSpeed, turnSpeed);
    m_backRight.setMotorSpeeds(driveSpeed, turnSpeed);
    try
    {
      Robot.bw.newLine();
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }
  }

  public void resetEncoders()
  {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public double getDriveMotorPosition()
  {
    return m_backRight.getDriveMotorPosition();
  }

  public double getTurnEncoderPosition()
  {
    return m_backRight.getTurningEncoderPosition();
  }

  public double getDriveMotorRate()
  {
    return m_backRight.getDrivingEncoderRate();
  }
  // FIXME Changing radians to degrees, commented out to make sure it breaks or doesn't
  // public double getTurnEncoderRate()
  // {
  //   return m_backRight.getTurnEncoderRate();
  // }

  // @Override
  // public void stopMotor()
  // {
  //   m_frontLeft
  // }

  // @Override
  // public String getDescription()
  // {
  //   return "Swerve Drivetrain";
  // }

  public void printNavX()
  {
    // System.out.println("Yaw = " + m_navx.getYaw() + "Rot2d = " + m_navx.getRotation2d());
    SmartDashboard.putNumber("NavX", m_navx.getRotation2d().getRadians());
  }

  public void printTurnEncoderPosition()
  {
    SmartDashboard.putNumber("frontLeft  Turn Encoder", m_frontLeft.getTurningEncoderPosition());
    SmartDashboard.putNumber("frontRight Turn Encoder", m_frontRight.getTurningEncoderPosition());
    SmartDashboard.putNumber("backLeft   Turn Encoder", m_backLeft.getTurningEncoderPosition());
    SmartDashboard.putNumber("backRight  Turn Encoder", m_backRight.getTurningEncoderPosition());
  }

  public void printDesiredStates(SwerveModuleState[] sms)
  {
    SmartDashboard.putNumber("frontLeft  Desired State Angle", sms[0].angle.getRadians());
    SmartDashboard.putNumber("frontRight Desired State Angle", sms[1].angle.getRadians());
    SmartDashboard.putNumber("backLeft   Desired State Angle", sms[2].angle.getRadians());
    SmartDashboard.putNumber("backRight  Desired State Angle", sms[3].angle.getRadians());

    SmartDashboard.putNumber("frontLeft  Desired State Speed", sms[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight Desired State Speed", sms[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft   Desired State Speed", sms[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight  Desired State Speed", sms[3].speedMetersPerSecond);    
  }
}