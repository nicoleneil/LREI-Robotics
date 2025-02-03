package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto NetSample + Park by Time", group="Robot")

public class AutoNetSample_andPark_byTime extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.4;
    static final double     REVERSE_SPEED = -0.4;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset Runtime - Used for time-based calculations in driving 
        runtime.reset();
        
        // Step 1: In 3rd Arena Square, Facing Net Zone, Drive forward for 2.5 seconds to score sample
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
            telemetry.addData("Forward Drive", "Observation Zone Park", runtime.seconds());
            telemetry.update();
        }

      // Step 2: Drive to the right for 1.15 seconds to clear alliance bot
        leftFrontDrive.setPower(FORWARD_SPEED);
        leftBackDrive.setPower(REVERSE_SPEED);
        rightFrontDrive.setPower(REVERSE_SPEED);
        rightBackDrive.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 2.5) && (runtime.seconds() < 3.65)) {
            telemetry.addData("Right Drive", "Observation Zone Park", runtime.seconds());
            telemetry.update();
      }

        // Step 3: Drive backwards for 5.1 seconds to reach Arena Back Wall
        leftFrontDrive.setPower(REVERSE_SPEED);
        leftBackDrive.setPower(REVERSE_SPEED);
        rightFrontDrive.setPower(REVERSE_SPEED);
        rightBackDrive.setPower(REVERSE_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 3.65) && (runtime.seconds() < 8.75)) {
            telemetry.addData("Backward Drive", "Observation Zone Park", runtime.seconds());
            telemetry.update();
      }
 
        // Step 4: Drive left for 2.25 seconds to Park in Observation Zone
        leftFrontDrive.setPower(REVERSE_SPEED);
        leftBackDrive.setPower(FORWARD_SPEED);
        rightFrontDrive.setPower(FORWARD_SPEED);
        rightBackDrive.setPower(REVERSE_SPEED);

        while (opModeIsActive() && (runtime.seconds() > 8.75) && (runtime.seconds() < 11)) {
            telemetry.addData("Left Drive", "Observation Zone Park", runtime.seconds());
            telemetry.update();
      }
        
        // Step 5: Stop Robot
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
