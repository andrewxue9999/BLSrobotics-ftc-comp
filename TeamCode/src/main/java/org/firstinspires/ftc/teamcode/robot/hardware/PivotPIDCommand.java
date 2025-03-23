package org.firstinspires.ftc.teamcode.robot.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

public class PivotPIDCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final PIDController pid;

    public PivotPIDCommand(PivotSubsystem p, double sp) {
        this.pivot = p;
        this.pid = new PIDController(0,0, 0);

        pid.setSetPoint(sp);

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {
        double speed = pid.calculate(pivot.getPivotPos());
        pivot.movePivot(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivot.movePivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
