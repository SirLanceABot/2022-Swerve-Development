// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule
{
  // TODO: Remove these when go to RobotDevelopment
  //FIXME Change Wheel Radius and EncoderResolution
  // public static final double kInchesToMeters = 0.0254;
  // private static final double kWheelRadius = 2 * kInchesToMeters;
  // private static final int kEncoderResolution = 2048;
  // FIXME Use these variables probably after we removed in a merge conflict
  // private static final double kWheelRadiusInches = 2;
  // private static final double kInchesToMeters = 0.0254;
  // private static final double kWheelRadiusMeters = kWheelRadiusInches * kInchesToMeters;
  // private static final int kDriveMotorEncoderResolution = 2048;
  // private static final int kTurningMotorEncoderResolution = 4096;
  // private static final double kDriveMotorGearRatio = 8.14;
  // private static final double kTurningMotorGearRatio = 12.8;

  // private static final double kModuleMaxAngularVelocity = Constants.MAX_TURN_SPEED;
  // private static final double kModuleMaxAngularAcceleration =
  //     2 * Math.PI; // radians per second squared

  private final String moduleName;
  private final TalonFX driveMotor;
  private final boolean driveMotorInverted;
  private final CANCoder turnEncoder; //= new CANCoder();
  private final double turnEncoderOffset;
  private final TalonFX turnMotor;

  private final PIDController drivePIDController = new PIDController(3.5, 0, 0.09); // volts / (m/s) for P and volts / (m/s^2) for D
  // private final PIDController turningPIDController = new PIDController(1.0, 0, 0);

  private final ProfiledPIDController turningPIDController =
      new ProfiledPIDController(
          0.0789, 0.0, 0.000877,
          // FIXME Changing radians to degrees, divided by 57 roughly
          // 4.5, 0.0, 0.05, //4.5, 0.0, 0.05,
          new TrapezoidProfile.Constraints(Constants.MAX_MODULE_TURN_SPEED, Constants.MAX_MODULE_TURN_ACCELERATION));

  //FIXME: Gains are for example purposes only - must be determined for your own robot!
  //First parameter is static gain (how much voltage it takes to move)
  //Second parameters is veloctiy gain (how much additional speed you get per volt)
  
  // 
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.165, 2.1, 0.0);
  
  // TODO Changing radians to degrees, Changed volts per radian to volts per degree by multiplying by 2 pi and dividing by 360
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.0035, 0.0052, 0.0);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0.2, 0.3, 0.0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turnMotorChannel ID for the turning motor.
   */
  // Old using enum constructor
  /*
  public SwerveModule(Constants.SwerveModule smc)
  {
    driveMotor = new TalonFX(smc.driveMotorChannel);
    turnEncoder = new CANCoder(smc.turnMotorEncoder);  
    turnMotor = new TalonFX(smc.turnMotorChannel);
    moduleName = smc.moduleName;
    turnEncoderOffset = smc.turnMotorEncoderOffset;

    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    configTalon(driveMotor, smc.driveMotorInverted);
    // Do not invert any of the turning motors
    configTalon(turnMotor, false);
    configCANCoder();

    // When deploy code set the integrated encoder to the absolute encoder on the CANCoder
    turnEncoder.setPosition(turnEncoder.getAbsolutePosition());

    // resetTurningMotorEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    // turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // FIXME Changing radians to degrees, replaced PI with 180
    turningPIDController.enableContinuousInput(-180, 180);
  }
  */

  
  /**
   * Constructs a SwerveModule.
   *
   * @param smd SwerveModuleData for making SwerveModules
   */
  public SwerveModule(SwerveModuleData smd)
  {
    moduleName = smd.moduleName;
    driveMotor = new TalonFX(smd.driveMotorChannel);
    driveMotorInverted = smd.driveMotorInverted;
    turnEncoder = new CANCoder(smd.turnEncoderChannel);  
    turnEncoderOffset = smd.turnEncoderOffset;
    turnMotor = new TalonFX(smd.turnMotorChannel);

    driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    configDriveTalon();
    configCANCoder();
    configTurnTalon(); // Do not invert any of the turning motors

    // When deploy code set the integrated encoder to the absolute encoder on the CANCoder
    turnEncoder.setPosition(turnEncoder.getAbsolutePosition());

    // TODO: Cleanup old commented out code

    // resetTurningMotorEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    // turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // FIXME Changing radians to degrees, replaced PI with 180
    turningPIDController.enableContinuousInput(-180, 180);
  }

  private void configDriveTalon()
  {
    driveMotor.configFactoryDefault();
    driveMotor.setInverted(driveMotorInverted);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.configForwardSoftLimitEnable(false);
    driveMotor.configReverseSoftLimitEnable(false);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); //TODO Convert to newer config API
    // driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    // driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    // driveMotor.configOpenloopRamp(openLoopRamp);
    driveMotor.configNeutralDeadband(0.001);
    driveMotor.configVoltageCompSaturation(Constants.MAX_BATTERY_VOLTAGE);
    driveMotor.enableVoltageCompensation(true);

    // Setup PID through TalonFX
    // Old values were 3.5, 0, 0.09
    // volts / (m/s) for P
    // volts / (m/s^2) for D
    // kP Converting v / (process unit) to volts
    // Assuming kI is 0
    // kD Converting v / (rate of change of process unit per time base) to volts

    // Old before pdf explanation?
    // final double kPTalonFX = (3.5 * 10) / Constants.DRIVE_ENCODER_RATE_IN_MOTOR_TICKS_PER_100MS;
    // // 10th of a second to 50th of a second or something
    // final double kDTalonFX = (0.09 * (10 * 5)) / Constants.DRIVE_ENCODER_RATE_IN_MOTOR_TICKS_PER_100MS;

    /*
    // Cancelled
    // TODO: Move these constants somewhere else or remove calculations
    final double kPTalonFX = 200 * (3.5 / (1 / Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC));
    final double kITalonFX = 0.0;
    // 10th of a second to 50th of a second or something
    final double kDTalonFX = (0.09 * (0.2 / (1 / Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC)));

    // Probably the wrong number but is my backup
    // final double kFTalonFX = (1023 * 0.75) / (2.1 / (1 / Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC));
    // Was 2.1 originally
    final double kFTalonFX = (3 / (1 / Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC));

    // TODO: Finish moving PID to TalonFX
    driveMotor.config_kP(0, kPTalonFX);
    driveMotor.config_kI(0, kITalonFX);
    driveMotor.config_kD(0, kDTalonFX);

    driveMotor.config_kF(0, kFTalonFX);
    */
  }

  private void configTurnTalon()
  {
    turnMotor.configFactoryDefault();
    turnMotor.setInverted(false);
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.configForwardSoftLimitEnable(false);
    turnMotor.configReverseSoftLimitEnable(false);
    turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); //TODO Convert to newer config API
    // turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    // turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    // turnMotor.configOpenloopRamp(openLoopRamp);
    turnMotor.configNeutralDeadband(0.001);
    turnMotor.configVoltageCompSaturation(Constants.MAX_BATTERY_VOLTAGE);
    turnMotor.enableVoltageCompensation(true);
  }

  private void configCANCoder()
  {
    // TODO Add all the config settings
    /**
       * Confgure the CANCoders
       */
      CANCoderConfiguration CANCoderConfigs = new CANCoderConfiguration();

      // Configurations all have in common
      CANCoderConfigs.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
      CANCoderConfigs.velocityMeasurementWindow = 64;
      CANCoderConfigs.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
      CANCoderConfigs.sensorDirection = false; // CCW
      CANCoderConfigs.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition; // error on get "On boot up, set position to zero.";
      // CANCoderConfigs.sensorCoefficient = 0.0015339776873588562; // 4096 ticks to radians
      CANCoderConfigs.sensorCoefficient = 0.087890625; // 4096 ticks to degrees
      CANCoderConfigs.unitString = "degrees";
      CANCoderConfigs.sensorTimeBase = SensorTimeBase.PerSecond;
      CANCoderConfigs.customParam0 = 0;
      CANCoderConfigs.customParam1 = 0;

      // Individual Settings
      System.out.println(moduleName);
      // frontLeftEncoder = new CANCoder(Constants.SwerveModule.frontLeft.turnMotorEncoder);
      // System.out.println("setStatusFramePeriod " + turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, (int)(steerAdjustPeriod*1000.*.8)));
      CANCoderConfigs.magnetOffsetDegrees = turnEncoderOffset;
      System.out.println("configAllSettings " + turnEncoder.configAllSettings(CANCoderConfigs));
      System.out.println(CANCoderConfigs.toString());

      /*
      System.out.println("backLeftEncoder");
      backLeftEncoder = new CANCoder(Constants.SwerveModule.backLeft.turnMotorEncoder);
      System.out.println("setStatusFramePeriod " + backLeftEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, (int)(steerAdjustPeriod*1000.*.8)));
      CANCoderConfigs.magnetOffsetDegrees = -348.75;
      System.out.println("configAllSettings " + backLeftEncoder.configAllSettings(CANCoderConfigs));
      System.out.println(CANCoderConfigs.toString());

      System.out.println("frontRightEncoder");
      frontRightEncoder = new CANCoder(Constants.SwerveModule.frontRight.turnMotorEncoder);
      System.out.println("setStatusFramePeriod " + frontRightEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, (int)(steerAdjustPeriod*1000.*.8)));
      CANCoderConfigs.magnetOffsetDegrees = -305.947265625;
      System.out.println("configAllSettings " + frontRightEncoder.configAllSettings(CANCoderConfigs));
      System.out.println(CANCoderConfigs.toString());

      System.out.println("backRightEncoder");
      backRightEncoder = new CANCoder(Constants.SwerveModule.backRight.turningMotorEncoder);
      System.out.println("setStatusFramePeriod " + backRightEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, (int)(steerAdjustPeriod*1000.*.8)));    
      CANCoderConfigs.magnetOffsetDegrees = -101.953125;
      System.out.println("configAllSettings " + backRightEncoder.configAllSettings(CANCoderConfigs));
      System.out.println(CANCoderConfigs.toString());
      */
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState()
  {
    // FIXME Changing radians to degrees
    return new SwerveModuleState(getDrivingEncoderRate(), Rotation2d.fromDegrees(getTurningEncoderPosition()));
    // return new SwerveModuleState(getDrivingEncoderRate(), new Rotation2d(getTurningEncoderPosition())); // Using radian line
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState)
  {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // FIXME Changing radians to degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderPosition()));
        
    // SwerveModuleState state =
    // SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderPosition()));

    double driveP = SmartDashboard.getNumber("Drive P", 0.0);
    double driveD = SmartDashboard.getNumber("Drive D", 0.0);
    drivePIDController.setP(driveP);
    drivePIDController.setD(driveD);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(getDrivingEncoderRate(), state.speedMetersPerSecond);

    final double driveFeedforwardValue = driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.

    // double p = SmartDashboard.getNumber("Turn P", 0.0);
    // double d = SmartDashboard.getNumber("Turn D", 0.0);
    // turningPIDController.setP(p);
    // turningPIDController.setD(d);
    // FIXME Changing radians to degrees, changed PID to take degrees
    final double turnOutput = turningPIDController.calculate(getTurningEncoderPosition(), state.angle.getDegrees());

    // final double turnOutput =
        // turningPIDController.calculate(Math.toRadians(getTurningEncoderPosition()), state.angle.getRadians());

    // FIXME Changing radians to degrees, Changed volts per radian to volts per degree for kS, kV, kA
    final double turnFeedforwardValue =
        turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    //FIXME Convert to Talon FX
    var normalizedDriveVoltage = normalizeVoltage(driveOutput + driveFeedforwardValue);
    var normalizedTurnVoltage = normalizeVoltage(turnOutput + turnFeedforwardValue);
    driveMotor.set(ControlMode.PercentOutput, normalizedDriveVoltage);
    // Used for running PIDF on TalonFX
    // var ticksPer100MS = state.speedMetersPerSecond * (1 / Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC);
    // System.out.println("M/S: " + state.speedMetersPerSecond + ", Ticks/100MS: " + ticksPer100MS);
    // driveMotor.set(ControlMode.Velocity, ticksPer100MS);
    // getDrivingEncoderRate();
    turnMotor.set(ControlMode.PercentOutput, normalizedTurnVoltage);


    // FIXME Changing radians to degrees
    SmartDashboard.putNumber(moduleName + " Optimized Angle Radians", state.angle.getRadians());
    SmartDashboard.putNumber(moduleName + " Optimized Angle Degrees", state.angle.getDegrees());
    SmartDashboard.putNumber(moduleName + " Optimized Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(moduleName + " Turn Output", turnOutput);
    SmartDashboard.putNumber(moduleName + " Turn Feedforward", turnFeedforwardValue);
    SmartDashboard.putNumber(moduleName + " Normalized Turn Percent", normalizedTurnVoltage);
    SmartDashboard.putNumber(moduleName + " Drive Output", driveOutput);
    SmartDashboard.putNumber(moduleName + " Drive Feedforward", driveFeedforwardValue);
    SmartDashboard.putNumber(moduleName + " Normalized Drive Percent", normalizedDriveVoltage);
    SmartDashboard.putNumber(moduleName + " Drive Encoder Rate", getDrivingEncoderRate());
  }

  public double getDrivingEncoderRate()
  {
    // FIXME Changing drivePID, changed nothing yet and probably do not need to because this will still return ticks / 100 ms
    double velocity = driveMotor.getSelectedSensorVelocity() * Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC;
    // FIXME Units conversion?
    // System.out.println(driveMotor.getDeviceID() + " " + velocity);
    return velocity;
  }

  public double getTurningEncoderPosition()
  {
    // FIXME Changing radians to degrees included making encoder return degrees
    // Used the Phoenix tuner to change the return value to radians
    return turnEncoder.getAbsolutePosition(); 
    // Reset facory default in Phoenix Tuner to make the 0 go forward 
    // while wheel bolts facing in, then save, then get absolute value and put in enum
    // return turningEncoder.getAbsolutePosition() - turningEncoderOffset; 
  }

  // FIXME Fix reset turning motor encoder
  public void resetTurningMotorEncoder()
  {
    // turningEncoder.setPosition(turningEncoderOffset);
  }

  /**
   * Normalizes voltage from -1 to 1 using current battery voltage
   * 
   * @param outputVolts
   * @return normalizedVoltage
   */
  public static double normalizeVoltage(double outputVolts)
  {
    return MathUtil.clamp(outputVolts / Constants.MAX_BATTERY_VOLTAGE, -1.0, 1.0); //RobotController.getBatteryVoltage();
  }

  public void resetEncoders()
  {
    driveMotor.setSelectedSensorPosition(0.0);
    turnMotor.setSelectedSensorPosition(0.0);
    turnEncoder.setPosition(turnEncoder.getAbsolutePosition());
  }

  // public void setMotorSpeeds(double driveSpeed, double turnSpeed)
  // {
  //   try
  //   {
  //     // var data = String.format("\"%s Turn\", %f, %f, %f, ", moduleName, Timer.getFPGATimestamp(), 
  //     //   turnEncoder.getVelocity(), turnEncoder.getPosition());
  //     var data = String.format("\"%s Drive\", %f, %f, %f, ", moduleName, Robot.time.get(), 
  //       getDrivingEncoderRate(), driveMotor.getSelectedSensorPosition() * Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC / 10.0);
  //     Robot.bw.write(data);
  //   }
  //   catch (Exception e)
  //   {
  //     e.printStackTrace();
  //   }

  //   driveMotor.set(ControlMode.PercentOutput, driveSpeed);
  //   turnMotor.set(ControlMode.PercentOutput, turnSpeed);
  // }

  public double getDriveMotorPosition()
  {
    return driveMotor.getSelectedSensorPosition();
  }

  // FIXME Changing radians to degrees, commented out to make sure it breaks or doesn't
  // public double getTurnEncoderRate()
  // {
  //   return turnEncoder.getVelocity();
  // }

  public void stopModule()
  {
    driveMotor.set(ControlMode.PercentOutput, 0.0);
    turnMotor.set(ControlMode.PercentOutput, 0.0);
  }

}