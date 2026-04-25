package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple, correct PIDF controller.
 *
 * Design decisions:
 *  - Timer is read and reset at the TOP of every calculate() call, unconditionally.
 *    This means deltaTime is always valid and the derivative never blows up.
 *  - Derivative is on the MEASUREMENT (processVariable), not the error.
 *    This eliminates "derivative kick" when the setpoint changes every loop.
 *  - Integral has clamping anti-windup — the sum is clamped to a range that
 *    prevents it from growing beyond what the output limits can use.
 *  - Tolerance dead-band returns 0 and clears state cleanly.
 */
public class PIDFController {

    private double kP, kI, kD, kF;
    private double minOutput, maxOutput;
    private double tolerance = 0.0;

    private double setpoint        = 0.0;
    private double integralSum     = 0.0;
    private double lastMeasurement = 0.0;
    private boolean firstRun       = true;

    private final ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF,
                          double minOutput, double maxOutput) {
        this.kP        = kP;
        this.kI        = kI;
        this.kD        = kD;
        this.kF        = kF;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setTarget(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Call once per loop with the current sensor reading.
     * Returns motor power in [minOutput, maxOutput].
     */
    public double calculate(double measurement) {

        // Always read dt first — never gate this behind a tolerance check
        double dt = timer.seconds();
        timer.reset();

        // On the very first call dt will be huge — clamp it
        if (firstRun) {
            dt       = 0.02; // assume one normal loop length
            firstRun = false;
            lastMeasurement = measurement;
        }
        dt = Math.max(0.001, Math.min(0.5, dt));

        double error = setpoint - measurement;

        // Dead-band — inside tolerance, stop and reset integrator
        if (Math.abs(error) <= tolerance) {
            integralSum     = 0.0;
            lastMeasurement = measurement;
            return 0.0;
        }

        // P term
        double P = kP * error;

        // I term with simple clamp anti-windup
        integralSum += error * dt;
        // Clamp integral contribution so it can never exceed half the output range
        double maxIntegral = (maxOutput - minOutput) / 2.0;
        integralSum = Math.max(-maxIntegral / (kI == 0 ? 1 : kI),
                Math.min( maxIntegral / (kI == 0 ? 1 : kI), integralSum));
        double I = kI * integralSum;

        // D term — derivative on MEASUREMENT, not error
        // This prevents a spike when setpoint changes every loop
        double dMeasurement = (measurement - lastMeasurement) / dt;
        double D = -kD * dMeasurement;   // negative because rising measurement
        // with positive error means converging

        // F term
        double F = kF * setpoint;

        double output = P + I + D + F;

        // Clamp output
        output = Math.max(minOutput, Math.min(maxOutput, output));

        lastMeasurement = measurement;
        return output;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    /** Call this whenever you change targets or re-enable the controller. */
    public void reset() {
        integralSum     = 0.0;
        firstRun        = true;
        timer.reset();
    }
}