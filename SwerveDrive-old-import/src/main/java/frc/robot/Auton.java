package frc.robot;


import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class Auton {

    static double goalPosBL = 0;
    static double goalPosBR = 0;
    static double goalPosFL = 0;
    static double goalPosFR = 0;
    static double goalPosSL = 0;
    static double goalPosSR = 0;

    static double MaxTime = 0;

    static Timer timerAuton = new Timer();

    public static void run() {

        double startTime;

        Timer timer = new Timer();
        timer.start();

    }

    public static void resetZero() {
        Components.sparkWheelBL.rezero();
        Components.sparkWheelBR.rezero();
        Components.sparkWheelFL.rezero();
        Components.sparkWheelFR.rezero();
        Components.sparkWheelTurnFR.rezero();
        Components.sparkWheelTurnFL.rezero();
        Components.sparkWheelTurnBR.rezero();
        Components.sparkWheelTurnBL.rezero();
    }

    public static void rotation(double rotDegree, double maxTime) {

    }

    public static boolean canGo() {
        double errorFR = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);
        double errorFL = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);
        double errorBL = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);
        double errorBR = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);
        double errorSL = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);
        double errorSR = Math.abs(Components.sparkWheelFR.motorPos - goalPosFR);

        if ((errorFR < Constants.Auton.tolerance) && (errorFL < Constants.Auton.tolerance) &&
                (errorBR < Constants.Auton.tolerance) && (errorSL < Constants.Auton.tolerance) &&
                (errorSR < Constants.Auton.tolerance) && (errorBL < Constants.Auton.tolerance)) {
            return true;
        } else {
            return (timerAuton.get() > MaxTime);
        }
    }
}
