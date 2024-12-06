package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class drive extends OpMode {

    private Motor front_left, front_right, back_left, back_right;
    private MecanumDrive mecanumDrive;
    private BNO055IMU imu;

    @Override
    public void init() {
        front_left = new Motor(hardwareMap, "motorLF");
        front_right = new Motor(hardwareMap, "motorRF");
        back_left = new Motor(hardwareMap, "motorLR");
        back_right = new Motor(hardwareMap, "motorRR");



        mecanumDrive = new MecanumDrive(front_left, front_right, back_left, back_right);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");
    }

    @Override
    public void loop() {
        // Get joystick inputs
        double strafeSpeed = gamepad1.left_stick_x;  //strafe
        double forwardSpeed = -gamepad1.left_stick_y; //forqard
        double turn = gamepad1.right_stick_x;        // rotate


        double heading = imu.getAngularOrientation().firstAngle;

        // field centric
        mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);

        telemetry.addData("strafe", strafeSpeed);
        telemetry.addData("forward", forwardSpeed);
        telemetry.addData("turn", turn);
        telemetry.addData("heading", heading);
        telemetry.update();
    }
}

@TeleOp
class drive2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motorLF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motorRF");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motorLR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motorRR");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
