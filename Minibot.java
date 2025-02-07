package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="MiniBot-Drive", group="Robot")

public class Minibot extends LinearOpMode {

    // todo: write your code here    
    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null; //the right drivetrain motor
    
    public void runOpMode() {
        
        /* These variables are private to the OpMode, and are used to control the drivetrain. */
        double left;
        double right;
        double forward;
        double rotate;
        double max;        
        
        leftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right drivetrain motor
    
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = gamepad1.left_stick_y;
            rotate  = -gamepad1.right_stick_x;

            /* Here we "mix" the input channels together to find the power to apply to each motor.
            The both motors need to be set to a mix of how much you're retesting the robot move
            forward, and how much you're requesting the robot turn. When you ask the robot to rotate
            the right and left motors need to move in opposite directions. So we will add rotate to
            forward for the left motor, and subtract rotate from forward for the right motor. */

            left  = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }
            
            /* Limit max speed */
            //left *=0.50;
            //right *=0.50;

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            rightDrive.setPower(right);

        }
    }
}
