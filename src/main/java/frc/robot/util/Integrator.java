package frc.robot.util;

public class Integrator {
    private final double[] lastVelocity = new double[2];
    private final double[] displacement = new double[2];

    public Integrator() {
        this.resetDisplacement();
    }

    public void updateDisplacement(double accelXG, double accelYG,
                                   int updateRateHz, boolean isMoving) {
        if (isMoving) {
            double[] accelG = new double[2];
            double[] accelMS2 = new double[2];
            double[] curVelocityMS = new double[2];
            double sampleTime = (1.0d / updateRateHz);
            accelG[0] = accelXG;
            accelG[1] = accelYG;
            for (int i = 0; i < 2; i++) {
                accelMS2[i] = accelG[i] * 9.80665d;
                curVelocityMS[i] = this.lastVelocity[i] + (accelMS2[i] * sampleTime);
                this.displacement[i] += this.lastVelocity[i] + (0.5d * accelMS2[i] * sampleTime * sampleTime);
                this.lastVelocity[i] = curVelocityMS[i];
            }
        } else {
            this.lastVelocity[0] = 0.0f;
            this.lastVelocity[1] = 0.0f;
        }
    }

    public void resetDisplacement() {
        for (int i = 0; i < 2; i++) {
            this.lastVelocity[i] = 0.0f;
            this.displacement[i] = 0.0f;
        }
    }

    public double getVelocityX() {
        return this.lastVelocity[0];
    }

    public double getVelocityY() {
        return this.lastVelocity[1];
    }

    public float getVelocityZ() {
        return 0;
    }

    public double getDisplacementX() {
        return this.displacement[0];
    }

    public double getDisplacementY() {
        return this.displacement[1];
    }

    public float getDisplacementZ() {
        return 0;
    }
}

