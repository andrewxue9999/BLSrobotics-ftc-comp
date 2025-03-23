package org.firstinspires.ftc.teamcode.robot.hardware;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

public class PivotPIDCommand extends CommandBase {
    private final PivotSubsystem pivot;
    private final PIDFController pid;

    public static double kP= 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0; //TODO: tune

    public PivotPIDCommand(PivotSubsystem p, double sp) {
        this.pivot = p;
        this.pid = new PIDFController(kP, kI, kD, kF);

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
