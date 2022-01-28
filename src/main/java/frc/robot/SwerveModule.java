// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

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

  //FIXME Convert to Talon FX
  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;

  //FIXME Convert to Talon FX
  // private Encoder m_driveEncoder;
  private final CANCoder m_turnEncoder; //= new CANCoder();

  private final String m_moduleName;
  private final double m_turnEncoderOffset;

  private final PIDController m_drivePIDController = new PIDController(3.5, 0, 0.09);
  // private final PIDController m_turningPIDController = new PIDController(1.0, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.0789, 0.0, 0.000877,
          // FIXME Changing radians to degrees, divided by 57 roughly
          // 4.5, 0.0, 0.05, //4.5, 0.0, 0.05,
          new TrapezoidProfile.Constraints(Constants.MAX_MODULE_TURN_SPEED, Constants.MAX_MODULE_TURN_ACCELERATION));

  //FIXME: Gains are for example purposes only - must be determined for your own robot!
  //First parameter is static gain (how much voltage it takes to move)
  //Second parameters is veloctiy gain (how much additional speed you get per volt)
  
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.165, 2.1, 0.0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.2, 0.3, 0.0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turnMotorChannel ID for the turning motor.
   */
  public SwerveModule(Constants.SwerveModule smc)
  {
    m_driveMotor = new TalonFX(smc.driveMotorChannel);
    m_turnEncoder = new CANCoder(smc.turnMotorEncoder);  
    m_turnMotor = new TalonFX(smc.turnMotorChannel);
    m_moduleName = smc.moduleName;
    m_turnEncoderOffset = smc.turnMotorEncoderOffset;

    m_driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    configTalon(m_driveMotor, smc.driveMotorInverted);
    // Do not invert any of the turning motors
    configTalon(m_turnMotor, false);
    configCANCoder();

    // When deploy code set the integrated encoder to the absolute encoder on the CANCoder
    m_turnEncoder.setPosition(m_turnEncoder.getAbsolutePosition());

    // resetTurningMotorEncoder();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // FIXME Changing radians to degrees, replaced PI with 180
    m_turningPIDController.enableContinuousInput(-180, 180);
  }

  // TODO Separate for the drive motor and turn motor
  private static void configTalon(TalonFX motor, boolean inverted)
  {
    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configForwardSoftLimitEnable(false);
    motor.configReverseSoftLimitEnable(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); //TODO Convert to newer config API
    // motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 25, 1.0));
    // motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
    // motor.configOpenloopRamp(openLoopRamp);
    motor.configNeutralDeadband(0.001);
    motor.configVoltageCompSaturation(Constants.MAX_BATTERY_VOLTAGE);
    motor.enableVoltageCompensation(true);
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
      System.out.println(m_moduleName);
      // frontLeftEncoder = new CANCoder(Constants.SwerveModule.frontLeft.turnMotorEncoder);
      // System.out.println("setStatusFramePeriod " + m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, (int)(steerAdjustPeriod*1000.*.8)));
      CANCoderConfigs.magnetOffsetDegrees = m_turnEncoderOffset;
      System.out.println("configAllSettings " + m_turnEncoder.configAllSettings(CANCoderConfigs));
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
    m_drivePIDController.setP(driveP);
    m_drivePIDController.setD(driveD);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDrivingEncoderRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.

    // double p = SmartDashboard.getNumber("Turn P", 0.0);
    // double d = SmartDashboard.getNumber("Turn D", 0.0);
    // m_turningPIDController.setP(p);
    // m_turningPIDController.setD(d);
    // FIXME Changing radians to degrees, changed PID to take degrees
    final double turnOutput = m_turningPIDController.calculate(getTurningEncoderPosition(), state.angle.getDegrees());

    // final double turnOutput =
        // m_turningPIDController.calculate(Math.toRadians(getTurningEncoderPosition()), state.angle.getRadians());

    // FIXME Changing radians to degrees, need to change this probably as it turns way to fast?
    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //FIXME Convert to Talon FX
    var normalizedDriveVoltage = normalizeVoltage(driveOutput + driveFeedforward);
    var normalizedTurnVoltage = normalizeVoltage(turnOutput + turnFeedforward);
    m_driveMotor.set(ControlMode.PercentOutput, normalizedDriveVoltage);
    m_turnMotor.set(ControlMode.PercentOutput, normalizedTurnVoltage);


    // FIXME Changing radians to degrees
    SmartDashboard.putNumber(m_moduleName + " Optimized Angle Radians", state.angle.getRadians());
    SmartDashboard.putNumber(m_moduleName + " Optimized Angle Degrees", state.angle.getDegrees());
    SmartDashboard.putNumber(m_moduleName + " Optimized Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(m_moduleName + " Turn Output", turnOutput);
    SmartDashboard.putNumber(m_moduleName + " Turn Feedforward", turnFeedforward);
    SmartDashboard.putNumber(m_moduleName + " Normalized Turn Percent", normalizedTurnVoltage);
    SmartDashboard.putNumber(m_moduleName + " Drive Output", driveOutput);
    SmartDashboard.putNumber(m_moduleName + " Drive Feedforward", driveFeedforward);
    SmartDashboard.putNumber(m_moduleName + " Normalized Drive Percent", normalizedDriveVoltage);
    SmartDashboard.putNumber(m_moduleName + " Drive Encoder Rate", getDrivingEncoderRate());
  }

  public double getDrivingEncoderRate()
  {
    double velocity = m_driveMotor.getSelectedSensorVelocity() * Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC;
    // FIXME Units conversion?
    // System.out.println(m_driveMotor.getDeviceID() + " " + velocity);
    return velocity;
  }

  public double getTurningEncoderPosition()
  {
    // FIXME Changing radians to degrees included making encoder return degrees
    // Used the Phoenix tuner to change the return value to radians
    return m_turnEncoder.getAbsolutePosition(); 
    // Reset facory default in Phoenix Tuner to make the 0 go forward 
    // while wheel bolts facing in, then save, then get absolute value and put in enum
    // return m_turningEncoder.getAbsolutePosition() - m_turningEncoderOffset; 
  }

  // FIXME Fix reset turning motor encoder
  public void resetTurningMotorEncoder()
  {
    // m_turningEncoder.setPosition(m_turningEncoderOffset);
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
    m_driveMotor.setSelectedSensorPosition(0.0);
    m_turnMotor.setSelectedSensorPosition(0.0);
    m_turnEncoder.setPosition(m_turnEncoder.getAbsolutePosition());
  }

  public void setMotorSpeeds(double driveSpeed, double turnSpeed)
  {
    try
    {
      // var data = String.format("\"%s Turn\", %f, %f, %f, ", m_moduleName, Timer.getFPGATimestamp(), 
      //   m_turnEncoder.getVelocity(), m_turnEncoder.getPosition());
      var data = String.format("\"%s Drive\", %f, %f, %f, ", m_moduleName, Robot.time.get(), 
        getDrivingEncoderRate(), m_driveMotor.getSelectedSensorPosition() * Constants.DRIVE_ENCODER_RATE_TO_METERS_PER_SEC / 10.0);
      Robot.bw.write(data);
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }

    m_driveMotor.set(ControlMode.PercentOutput, driveSpeed);
    m_turnMotor.set(ControlMode.PercentOutput, turnSpeed);
  }

  public double getDriveMotorPosition()
  {
    return m_driveMotor.getSelectedSensorPosition();
  }

  // FIXME Changing radians to degrees, commented out to make sure it breaks or doesn't
  // public double getTurnEncoderRate()
  // {
  //   return m_turnEncoder.getVelocity();
  // }
}