// two servos that turn; takes difference of two servos to make change the height of claw
// opposite spin: spins in place
// same spin: moves up/down

package org.firstinspires.ftc.teamcode.robot.hardware;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialWrist {
    private Servo leftWrist;
    private Servo rightWrist;

    private final double intakePos = 0.1;
    private final double scoringPos = 0.9;
    private final double dropPos = 1.0;
    private final double hangPos = 0.3;

    private String currentPos;

    public void init(@NonNull HardwareMap hardwareMap, String leftWristName, String rightWristName) {
        leftWrist = hardwareMap.get(Servo.class, leftWristName);
        rightWrist = hardwareMap.get(Servo.class, rightWristName);

        leftWrist.scaleRange(intakePos, dropPos);
        rightWrist.scaleRange(intakePos, dropPos);
    }

    public void goToPos(String command) {
        if (command.equals("intake")) {
            leftWrist.setPosition(intakePos);
            rightWrist.setPosition(intakePos);

            currentPos = command;
        } else if (command.equals("score")) {
            leftWrist.setPosition(scoringPos);
            rightWrist.setPosition(scoringPos);

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

    public void rotate(double angle) {

        // s = r * theta
        // .9 .0805
    }

    public String returnPos() {
        return currentPos;
    }

    public String getTelemetry() {
        return String.format("Position: %S", currentPos);
    }
}
