package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {

    private Servo clawServo;
    private boolean isOpen = false;
    private double lastTime = 0;
    private final double TIME_DELAY = 0.15;

    // Servo limits
    private final double LEFT_LIMIT = 0.0; // Corresponds to -135° cuz it is the The input pulse range for the servo is
    // from 500μs to 2500μs so t 0 to 1
    private final double RIGHT_LIMIT = 1.0; // Corresponds to +135°
    private final double CENTER_LEFT = 0.2; // Corresponds to -20°
    private final double CENTER_RIGHT = 0.8; // Corresponds to +20°

    private final double HOME_POINT = 0.5;

    public void init(@NonNull HardwareMap hardwareMap, String name) {
        clawServo = hardwareMap.get(Servo.class, name);

        clawServo.setPosition(HOME_POINT);
    }

    // Scale the range of the servo to only operate between defined limits
    public void scaleRange(double min, double max) {
        clawServo.scaleRange(min, max);
    }

    public void toggle(ElapsedTime time) {
        if (isOpen && time.time() > lastTime + TIME_DELAY) {
            // Move the servo to the left limit (e.g., -135°)

            isOpen = false;
            clawServo.setPosition(0);
        } else if (!isOpen && time.time() > lastTime + TIME_DELAY) {
            // Move the servo to the right limit (e.g., +135°)
            clawServo.setPosition(1.0);
            isOpen = true;
        }

        lastTime = time.time();
    }

//    public void setPositionWithDeadband(double position) {
//
//        if (position >= DEAD_BAND_LEFT && position <= DEAD_BAND_RIGHT) {
//// to check if the angle or position of the servo lies within the deadband or in
//// ryans words it is the center slice where the servo is not allowed to move
//            clawServo.setPosition(position < 0.5 ? LEFT_LIMIT : RIGHT_LIMIT); // Move to the closest limit
//        } else {
//            clawServo.setPosition(position);
//        }
//    }


    public boolean getState() {
        return isOpen;
    }

    public String getTelemetry() {
        return String.format("Open %B", isOpen);
    }
}

// (kevin's) (sus) sources/inspo of where i got my code from: servodoc:
// https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Servo.html#MAX_POSITION

