package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp(name = "Linear Slide Control", group = "TeleOp")
public class LinearSlideControl extends OpMode {

    // Motor controlling the horizontal linear slide
    private DcMotorEx slideMotor;

    // Constants for slide positions (in encoder ticks)
    private final int SLIDE_MAX_POSITION = 4200; // Adjust for your motor and setup
    private final int SLIDE_MIN_POSITION = 0;

    @Override
    public void init() {
        // Initialize the motor
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        // Set motor direction and zero power behavior
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor mode to use encoders
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Extend the slide when "X" is pressed
        if (gamepad1.x) { // "X" button on PS5 controller
            if (slideMotor.getCurrentPosition() < SLIDE_MAX_POSITION) {
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor.setPower(1.0); // Adjust speed as necessary
            } else {
                slideMotor.setPower(0); // Stop motor if max position is reached
            }
        }
        // Retract the slide when "Circle" is pressed
        else if (gamepad1.circle) { // "Circle" button on PS5 controller
            if (slideMotor.getCurrentPosition() > SLIDE_MIN_POSITION) {
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                slideMotor.setPower(-1.0); // Adjust speed as necessary
            } else {
                slideMotor.setPower(0); // Stop motor if min position is reached
            }
        }
        // Stop the motor if no button is pressed
        else {
            slideMotor.setPower(0);
        }

        // Telemetry for debugging
        telemetry.addData("Slide Position", slideMotor.getCurrentPosition());
        telemetry.addData("Max Position", SLIDE_MAX_POSITION);
        telemetry.addData("Min Position", SLIDE_MIN_POSITION);
        telemetry.update();
    }
}
