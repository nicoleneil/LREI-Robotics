package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.ADIS16470_IMU; // Example gyro class
import edu.wpi.first.math.geometry.Rotation2d;

public class Robot extends TimedRobot {
    // Joystick and motor declarations
    private Joystick joystick = new Joystick(0); // Joystick on port 0
    private PWMSparkMax frontLeftMotor = new PWMSparkMax(0); // Front-left motor on PWM 0
    private PWMSparkMax frontRightMotor = new PWMSparkMax(1); // Front-right motor on PWM 1
    private PWMSparkMax rearLeftMotor = new PWMSparkMax(2); // Rear-left motor on PWM 2
    private PWMSparkMax rearRightMotor = new PWMSparkMax(3); // Rear-right motor on PWM 3

    private MecanumDrive drive; // WPILib's built-in MecanumDrive class
    private ADIS16470_IMU gyro = new ADIS16470_IMU(); // Example gyro sensor

    // Tuning constant for rotation sensitivity
    private double rotationSensitivity = 0.6;

    @Override
    public void robotInit() {
        // Initialize MecanumDrive with the motor controllers
        drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

        // Invert motors if necessary
        frontRightMotor.setInverted(true); // Typical for mecanum drives
        rearRightMotor.setInverted(true);
    }

    @Override
    public void teleopPeriodic() {
        // Read joystick inputs
        double forward = -joystick.getY(); // Forward/backward motion
        double right = joystick.getX();    // Left/right strafing
        double clockwise = joystick.getZ() * rotationSensitivity; // Rotation

        // Field-centric control using gyro
        boolean fieldCentricEnabled = true; // Toggle this as needed
        if (fieldCentricEnabled) {
            Rotation2d gyroAngle = Rotation2d.fromDegrees(gyro.getAngle()); // Gyro angle in degrees
            double thetaRad = gyroAngle.getRadians(); // Convert to radians
            double temp = forward * Math.cos(thetaRad) + right * Math.sin(thetaRad);
            right = -forward * Math.sin(thetaRad) + right * Math.cos(thetaRad);
            forward = temp;
        }

        // Drive the robot
        drive.driveCartesian(forward, right, clockwise, 0.0); // Gyro angle can replace 0.0 if desired
    }
}



/* deadband */  
if (Math.abs(forward) < 0.05) forward = 0;
if (Math.abs(right) < 0.05) right = 0;
if (Math.abs(clockwise) < 0.05) clockwise = 0;
