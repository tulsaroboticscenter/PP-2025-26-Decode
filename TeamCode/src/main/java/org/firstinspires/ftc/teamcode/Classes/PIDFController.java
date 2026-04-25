package org.firstinspires.ftc.teamcode.Classes;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Fixed PIDFController — replaces both the original PIDFController AND
 * PIDFControllerCTS. Use this single file everywhere in the codebase.
 *
 * Three bugs fixed vs. the original:
 *
 *  FIX 1 — Derivative computed on error delta, not raw error.
 *           Original: Kd * error / dt  → spikes every loop on heading control
 *           Fixed:    Kd * (error - previousError) / dt  → smooth damping
 *
 *  FIX 2 — Timer no longer resets inside the tolerance dead-band.
 *           Original: timer reset on tolerance exit → dt ≈ 0 → derivative explosion
 *           Fixed:    timer always resets at top of calculate(), dt is always valid
 *
 *  FIX 3 — Integral anti-windup.
 *           Original: integralSum grows unbounded → overshoot → oscillation
 *           Fixed:    integral only accumulates when output is not saturated
 */
public class PIDFController
{
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;

    private double setpoint;
    private double previousError;
    private double integralSum;

    private double tolerance = 0;

    private double minOutput;
    private double maxOutput;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double MAX_DELTA_TIME = 0.5;   // seconds — clamps stale first read
    private static final double MIN_DELTA_TIME = 1e-6;  // prevents divide-by-zero

    public PIDFController(double Kp, double Ki, double Kd, double Kf,
                          double minOutput, double maxOutput)
    {
        this.Kp        = Kp;
        this.Ki        = Ki;
        this.Kd        = Kd;
        this.Kf        = Kf;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        reset();
    }

    public void setTarget(double setpoint)
    {
        this.setpoint = setpoint;
    }

    public double calculate(double processVariable)
    {
        // FIX 2: always read and reset timer here — never inside the tolerance block
        double deltaTime = timer.seconds();
        timer.reset();
        deltaTime = Math.max(MIN_DELTA_TIME, Math.min(MAX_DELTA_TIME, deltaTime));

        double error = setpoint - processVariable;

        // Tolerance dead-band — keep previousError current so derivative is
        // smooth when the system leaves the band
        if (Math.abs(error) <= tolerance)
        {
            previousError = error;
            integralSum   = 0;
            return 0;
        }

        // Proportional
        double proportionalTerm = Kp * error;

        // Integral (speculative accumulation for anti-windup check)
        double prospectiveIntegral = integralSum + error * deltaTime;
        double integralTerm        = Ki * prospectiveIntegral;

        // FIX 1: derivative on error delta, not raw error
        double derivativeTerm = Kd * ((error - previousError) / deltaTime);

        // Feed-forward
        double feedforwardTerm = Kf * setpoint;

        double output = proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;

        // FIX 3: only commit integral when output is not saturated
        boolean saturated = (output > maxOutput && error > 0)
                || (output < minOutput && error < 0);
        if (!saturated)
        {
            integralSum = prospectiveIntegral;
        }

        output = Math.max(minOutput, Math.min(maxOutput, output));

        previousError = error;

        return output;
    }

    public void setTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public void setPIDFCoefficients(double p, double i, double d, double f)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        Kf = f;
    }

    public void reset()
    {
        integralSum   = 0;
        previousError = 0;
        timer.reset();
    }
}