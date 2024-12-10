package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class HorizontalArm extends SubsystemBase {

    private final DcMotor motorHA; // Motor for horizontal slide
    private final DigitalChannel limitSwitchHA; // Limit switch for resetting
    private final PIDController pidController;

    // PID Coefficients
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Target positions in encoder ticks
    private static final int SLIDE_MAX_POSITION = 4200;
    private static final int SLIDE_MIN_POSITION = 0;

    public HorizontalArm(RobotHardware robotHardware) {
        this.motorHA = robotHardware.motorHA;
        this.limitSwitchHA = robotHardware.limitSwitchHA;
        this.pidController = new PIDController(kP, kI, kD);

        // Reset encoder and set motor to use encoders
        motorHA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorHA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void extendArm() {
        moveToPosition(SLIDE_MAX_POSITION);
    }

    public void retractArm() {
        moveToPosition(SLIDE_MIN_POSITION);
    }

    private void moveToPosition(int targetPosition) {
        double currentPosition = motorHA.getCurrentPosition();
        double power = pidController.calculate(currentPosition, targetPosition);

        // Ensure power is within [-1.0, 1.0]
        power = Math.max(-1.0, Math.min(1.0, power));

        motorHA.setPower(power);
    }

    public void stopArm() {
        motorHA.setPower(0);
    }

    public void checkAndResetEncoder() {
        if (!limitSwitchHA.getState()) { // Limit switch pressed
            motorHA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorHA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void periodic() {
        checkAndResetEncoder();
    }
}
