// set points for arm extension

package org.firstinspires.ftc.teamcode.robot.hardware;

public class Arm {
    private DcMotor arm;
    public void init(@NonNull HardwareMap hardwareMap, String name) {
        arm = hardwareMap.get(DcMotor.class, name);
    }
    public String getTelemetry(String name) {
        return String.format("Open %S", name);
    }
}
