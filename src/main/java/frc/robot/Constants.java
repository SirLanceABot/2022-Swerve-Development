// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *  
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static final String test = "Test to push code";
    public static final double INCHES_TO_METERS = 0.0254;

    // FIXME Check that this is y coord and the other is x coord
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     * Measured in inches, y-coordinate
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 23.5 * INCHES_TO_METERS;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     * Measured in inches, x-coordinate
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 23.5 * INCHES_TO_METERS;

    public static final double WHEEL_RADIUS_METERS = 2.0 * INCHES_TO_METERS;

    public static final int DRIVETRAIN_NAVX_ID = 0; // FIXEDME Set Pigeon ID

    // Drivetrain constants
    // FIXME Check the max speeds and accelerations
    public static final double MAX_DRIVE_SPEED = 4.4; // meters per second
    public static final double MAX_MODULE_TURN_SPEED = 1080.0; // degrees per second, this is 3.0 rev/sec, used to be 1980 and 5.5 rev/sec
    public static final double MAX_ROBOT_TURN_SPEED = 360.0; // FIXME to more accurate degrees per second, this is 2 rev/sec, used to be 1080 and 3 rev/sec
    public static final double MAX_MODULE_TURN_ACCELERATION = 1728.0; // degrees per second per second, this is 4.8 rev/sec^2, used to be 17280 and 48 rev/sec^2
    // FIXME Changing radians to degrees, same measurements but in radians
    // public static final double MAX_MODULE_TURN_SPEED = 35.0; // radians per second
    // public static final double MAX_ROBOT_TURN_SPEED = 18.0; // FIXME radians per second
    // public static final double MAX_MODULE_TURN_ACCELERATION = 300.0; // radians per second per second

    public static final int DRIVE_MOTOR_ENCODER_RESOLUTION = 2048;
    public static final int TURN_MOTOR_ENCODER_RESOLUTION = 4096;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14;
    public static final double TURN_MOTOR_GEAR_RATIO = 12.8;

    public static final double DRIVE_ENCODER_RATE_TO_METERS_PER_SEC = 
        ((10.0 / DRIVE_MOTOR_ENCODER_RESOLUTION) / DRIVE_MOTOR_GEAR_RATIO) * (2.0 * Math.PI * WHEEL_RADIUS_METERS);
    public static final double TURN_ENCODER_RATE_TO_RADIANS_PER_SEC = 
        (1.0 / TURN_MOTOR_ENCODER_RESOLUTION) * (2.0 * Math.PI);

    public static final double MAX_BATTERY_VOLTAGE = 12.0;

    /* Correct values that have been moved to enum
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXEDME Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 9; // FIXEDME Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8; // FIXEDME Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 1899;//-Math.toRadians(166.904296875); // FIXEDME Measure and set front left steer offset
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; // FIXEDME Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 12; // FIXEDME Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11; // FIXEDME Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 3473;//-Math.toRadians(305.244140625 + 180.0); // FIXEDME Measure and set front right steer offset
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXEDME Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // FIXEDME Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; // FIXEDME Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 3976;//-Math.toRadians(349.453125); // FIXEDME Measure and set back left steer offset
    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1; // FIXEDME Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 3; // FIXEDME Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 2; // FIXEDME Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 1177;//-Math.toRadians(103.447265625 + 180.0); // FIXEDME Measure and set back right steer offset
    */
  
    /*
    Old enum for SwerveModule creation
    static enum SwerveModule
    {
        frontLeft("Front Left", 7, true, 8, -167.255859375, 9){},
        frontRight("Front Right", 10, false, 11, -305.947265625, 12){},
        backLeft("Back Left", 4, true, 5, -348.75, 6){},
        backRight("Back Right", 1, false, 2, -101.953125, 3){};

        String moduleName;
        int driveMotorChannel;
        boolean driveMotorInverted;
        int turnMotorEncoder;
        double turnMotorEncoderOffset;
        int turnMotorChannel;

        /**
         * @param driveMotorChannel
         * @param driveMotorInverted
         * @param turnMotorEncoder
         * @param turnMotorEncoderOffset
         * @param turnMotorChannel
         */
    /*
        private SwerveModule( String moduleName,
                            int driveMotorChannel, 
                            boolean driveMotorInverted, 
                            int turnMotorEncoder, 
                            double turnMotorEncoderOffset, 
                            int turnMotorChannel)
        {
            this.moduleName = moduleName;
            this.driveMotorChannel = driveMotorChannel;
            this.driveMotorInverted = driveMotorInverted;
            this.turnMotorEncoder = turnMotorEncoder;
            // FIXME make not strange conversion
            this.turnMotorEncoderOffset = turnMotorEncoderOffset;
            this.turnMotorChannel = turnMotorChannel;
        }

        public static String dump()
        {
            StringBuilder sb = new StringBuilder(400);
            
            sb.append("   Name       DriveMotor     Turn Encoder  Turn Motor\n");
            sb.append("           Channel Inverted Channel Offset   Channel\n");

            for(SwerveModule smc:SwerveModule.values())
            {
                sb.append(String.format("%10s  %3d     %5b    %3d    %6.1f    %3d\n",
                    smc.name(),
                    smc.driveMotorChannel,
                    smc.driveMotorInverted,
                    smc.turnMotorEncoder,
                    smc.turnMotorEncoderOffset,
                    smc.turnMotorChannel));
            }
            return sb.toString();
        }
    }
    */

    public static final SwerveModuleData FRONT_LEFT = new SwerveModuleData("Front Left", 7, true, 8, -167.255859375, 9);
    public static final SwerveModuleData FRONT_RIGHT = new SwerveModuleData("Front Right", 10, false, 11, -305.947265625, 12);
    public static final SwerveModuleData BACK_LEFT = new SwerveModuleData("Back Left", 4, true, 5, -348.75, 6);
    public static final SwerveModuleData BACK_RIGHT = new SwerveModuleData("Back Right", 1, false, 2, -101.953125, 3);
}
