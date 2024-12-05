// pivot for the arm
// motor that makes it go in between certain angles; limits
// gear ratios
// calculations for limits to not exceed 42 inch horizontal limit
// account for how far and back it can go; limits
// do height (pythagorean theorem to calculate)
// set points (limits)

package org.firstinspires.ftc.teamcode.robot.hardware;

public class Pivot {
  private DcMotor pivot; 
  public void init(@NonNull HardwareMap hardwareMap, String name) {
       pivot = hardwareMap.get(DcMotor.class, name);
  }
  public String getTelemetry(String name) {
      return String.format("Open %S", name);
  }
}
