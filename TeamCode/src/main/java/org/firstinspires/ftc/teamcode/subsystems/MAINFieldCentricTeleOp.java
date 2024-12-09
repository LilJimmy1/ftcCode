package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Unified Field-Centric TeleOp", group = "Linear Opmode")
public class MAINFieldCentricTeleOp extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("motorLF");
        backLeftMotor = hardwareMap.dcMotor.get("motorLR");
        frontRightMotor = hardwareMap.dcMotor.get("motorRF");
        backRightMotor = hardwareMap.dcMotor.get("motorRR");

        // Reverse necessary motors (adjust based on your robot's configuration)
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Get joystick inputs
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafe (scaled for strafing inefficiency)
            double rx = gamepad1.right_stick_x; // Rotation

            // Reset IMU yaw with the "Options" button
            if (gamepad1.options) {
                imu.initialize(parameters);
                telemetry.addLine("IMU Yaw Reset!");
                telemetry.update();
            }

            // Get robot heading from IMU
            double botHeading = imu.getAngularOrientation().firstAngle;

            // Field-centric transformation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate motor powers
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // Apply motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Heading (rad)", botHeading);
            telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", x, y, rx);
            telemetry.addData("Motor Powers (FL, BL, FR, BR)", "%.2f, %.2f, %.2f, %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.update();
        }
    }
}
