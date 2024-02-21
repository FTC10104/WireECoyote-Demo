package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public final DcMotor intake;
    private final Servo lift;
    public boolean intakeActive = false;

    private double intakeSpeed;
    private double ejectSpeed;

    private final double intakePickupPosition;
    private final double intakeStowedPosition;
    private final boolean liftAvailable;

    public Intake(DcMotor intake, Servo lift, String activeConfigName) {
        this.intake = intake;
        this.lift = lift;
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(Servo.Direction.REVERSE);

        if (activeConfigName.equals(CENTERSTAGE_Bot.CONFIG_BOT_16464)) {
            intakeSpeed = 1;
            ejectSpeed = 0.5;

            // No Intake lift on 16464 bot
            liftAvailable = false;
            intakePickupPosition = 0;
            intakeStowedPosition = 0;
        } else {
            intakeSpeed = 0.75;
            ejectSpeed = 0.25;

            liftAvailable = true;
            intakePickupPosition = 0.8;
            intakeStowedPosition = 1;
        }

    }

    public void stopIntake() {
        intake.setPower(0);
        intakeActive = false;
    }

    public void togglePickup() {
        if (intakeActive) {
            stopIntake();
        } else {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(intakeSpeed);
            intakeActive = true;
        }
    }

    public void toggleEject() {
        if (intakeActive) {
            stopIntake();
        } else {
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(ejectSpeed);
            intakeActive = true;
        }
    }

    public void togglePosition() {
        if (!liftAvailable) {
            return;
        }
    }
    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }
    public void setEjectSpeed(double speed){
        ejectSpeed = speed;
    }
}
