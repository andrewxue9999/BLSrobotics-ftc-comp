// two servos that turn; takes difference of two servos to make change the height of claw
// opposite spin: spins in place
// same spin: moves up/down

package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Locale;

public class Wrist {
    private Servo leftWrist;
    private Servo rightWrist;
    private Servo centralServo;
    private final double intakePos = 0.44;
    private final double scoringPos = 0.9;
    private final double dropPos = 1.0;
    private final double hangPos = 0.3;

    private final double central_intake = 0.0;
    private final double central_scoring = 0.5;

    private String currentPos;

    public void init(@NonNull HardwareMap hardwareMap, String leftWristName, String rightWristName, String centralServoName) {
        leftWrist = hardwareMap.get(Servo.class, leftWristName);
        rightWrist = hardwareMap.get(Servo.class, rightWristName);
        centralServo = hardwareMap.get(Servo.class, centralServoName);

        leftWrist.scaleRange(intakePos, dropPos);
        rightWrist.scaleRange(intakePos, dropPos);
    }

    public void goToPos(String command) {
        if (command.equals("intake")) {
            leftWrist.setPosition(intakePos);
            rightWrist.setPosition(intakePos);

            centralServo.setPosition(central_intake);

            currentPos = command;
        } else if (command.equals("score")) {
            leftWrist.setPosition(scoringPos);
            rightWrist.setPosition(scoringPos);

            centralServo.setPosition(central_scoring);

            currentPos = command;
        } else if (command.equals("drop")) {
            leftWrist.setPosition(dropPos);
            rightWrist.setPosition(dropPos);

            currentPos = command;
        } else if (command.equals("hang")) {
            leftWrist.setPosition(hangPos);
            rightWrist.setPosition(hangPos);

            currentPos = command;
        }
    }


    public String returnPos() {
        return currentPos;
    }

    public String getTelemetry() {
        return String.format(Locale.ENGLISH, "Position: %S, left wrist position: %.2f, right wrist position: %.2f, central servo position: %.2f", currentPos, leftWrist.getPosition(), rightWrist.getPosition(), centralServo.getPosition());
    }
}
