package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

@TeleOp
public class drive2 extends OpMode {

    private Motor fL, fR, bL, bR;
    private MecanumDrive drive;
    private GamepadEx driverOp;

    @Override
    public void init() {
        /* instantiate motors */
        fL = new Motor(hardwareMap, "motorLF", Motor.GoBILDA.RPM_312);
        fR = new Motor(hardwareMap, "motorRF", Motor.GoBILDA.RPM_312);
        bL = new Motor(hardwareMap, "motorLB", Motor.GoBILDA.RPM_312);
        bR = new Motor(hardwareMap, "motorRB", Motor.GoBILDA.RPM_312);

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drive.driveRobotCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightY()
        );
    }

}
