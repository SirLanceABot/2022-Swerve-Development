// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Fidelity;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends RobotDriveBase
{
//   public static final double kMaxSpeed = 3.0; // 3 meters per second
//   public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
//   public static final double kInchesToMeters = 0.0254;
//   //inches, distance from center, robot is a square so length and width are the same
//   public static final double kRobotWidth = 23.5; //inches, y-coordinate
//   public static final double kRobotLength = 23.5; //inches, x-coordinate

  private static final Translation2d frontLeftLocation = new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2, Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d frontRightLocation = new Translation2d(Constants.DRIVETRAIN_WHEELBASE_METERS / 2, -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d backLeftLocation = new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2, Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);
  private static final Translation2d backRightLocation = new Translation2d(-Constants.DRIVETRAIN_WHEELBASE_METERS / 2, -Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2);

  private static final SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT);
  private static final SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT);
  private static final SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT);
  private static final SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT);

  private static final AHRS navx = new AHRS(SerialPort.Port.kUSB);

  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private static final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, navx.getRotation2d());

  // TODO: Make final by setting to an initial stopped state
  private static SwerveModuleState[] previousSwerveModuleStates = null;

  public Drivetrain()
  {
    super();

    navx.reset();
    
    // System.out.println(frontLeft.getTurningEncoderPosition());
    // System.out.println(frontRight.getTurningEncoderPosition());
    // System.out.println(backLeft.getTurningEncoderPosition());
    // System.out.println(backRight.getTurningEncoderPosition());
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
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d());
    else
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
    swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    // var swerveModuleStates =
    //     kinematics.toSwerveModuleStates(
    //         fieldRelative
    //             ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
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
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    previousSwerveModuleStates = swerveModuleStates;

    feedWatchdog();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry()
  {
    odometry.update(
        navx.getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  // private void setMotorSpeeds(double driveSpeed, double turnSpeed)
  // {
  //   frontLeft.setMotorSpeeds(driveSpeed, turnSpeed);
  //   frontRight.setMotorSpeeds(driveSpeed, turnSpeed);
  //   backLeft.setMotorSpeeds(driveSpeed, turnSpeed);
  //   backRight.setMotorSpeeds(driveSpeed, turnSpeed);
  //   feedWatchdog();

  //   try
  //   {
  //     Robot.bw.newLine();
  //   }
  //   catch (Exception e)
  //   {
  //     e.printStackTrace();
  //   }
  // }

  public void resetEncoders()
  {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  public double getDriveMotorPosition()
  {
    return backRight.getDriveMotorPosition();
  }

  public double getTurnEncoderPosition()
  {
    return backRight.getTurningEncoderPosition();
  }

  public double getDriveMotorRate()
  {
    return backRight.getDrivingEncoderRate();
  }
  // FIXME Changing radians to degrees, commented out to make sure it breaks or doesn't
  // public double getTurnEncoderRate()
  // {
  //   return backRight.getTurnEncoderRate();
  // }

  @Override
  public void stopMotor()
  {
    frontLeft.stopModule();
    frontRight.stopModule();
    backLeft.stopModule();
    backRight.stopModule();
    feedWatchdog();
  }

  @Override
  public String getDescription()
  {
    return "Swerve Drivetrain";
  }

  public void printNavX()
  {
    // System.out.println("Yaw = " + navx.getYaw() + "Rot2d = " + navx.getRotation2d());
    SmartDashboard.putNumber("NavX", navx.getRotation2d().getRadians());
  }

  public void printTurnEncoderPosition()
  {
    SmartDashboard.putNumber("frontLeft  Turn Encoder", frontLeft.getTurningEncoderPosition());
    SmartDashboard.putNumber("frontRight Turn Encoder", frontRight.getTurningEncoderPosition());
    SmartDashboard.putNumber("backLeft   Turn Encoder", backLeft.getTurningEncoderPosition());
    SmartDashboard.putNumber("backRight  Turn Encoder", backRight.getTurningEncoderPosition());
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