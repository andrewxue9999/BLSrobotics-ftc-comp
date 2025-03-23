package org.firstinspires.ftc.teamcode.robot.hardware;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotGamepadCommand extends CommandBase {

    private final PivotSubsystem pivot;
    double speed;

    public PivotGamepadCommand(PivotSubsystem p, double speed) {
        this.pivot = p;
        this.speed = speed;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.telemetry.addData("Pivot command high started!", "");
    }

    @Override
    public void execute() {
        pivot.movePivot(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.movePivot(0);
        pivot.telemetry.addData("Pivot command high ended.", "");

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}