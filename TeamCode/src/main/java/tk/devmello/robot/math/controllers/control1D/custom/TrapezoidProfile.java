package tk.devmello.robot.math.controllers.control1D.custom;

public class TrapezoidProfile {
    private double maxVelocity;
    private double maxAcceleration;
    private double maxDeceleration;

    public TrapezoidProfile(double maxVelocity, double maxAcceleration, double maxDeceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
    }





    public double getMaxVelocity() {
        return maxVelocity;
    }

    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public double getMaxDeceleration() {
        return maxDeceleration;
    }

    public void setMaxDeceleration(double maxDeceleration) {
        this.maxDeceleration = maxDeceleration;
    }

    public double calculate(double currentVelocity, double targetVelocity) {
        double acceleration = 0;
        if (currentVelocity < targetVelocity) {
            acceleration = maxAcceleration;
        } else if (currentVelocity > targetVelocity) {
            acceleration = -maxDeceleration;
        }
        return acceleration;
    }
    public double calculate(double currentVelocity, double targetVelocity, double dt) {
        double acceleration = 0;
        if (currentVelocity < targetVelocity) {
            acceleration = maxAcceleration;
        } else if (currentVelocity > targetVelocity) {
            acceleration = -maxDeceleration;
        }
        return acceleration * dt;
    }


    public static class State{
        private double position;
        private double velocity;

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        public double getPosition() {
            return position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

        public double getVelocity() {
            return velocity;
        }

        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }


    }

    public static class Constraints {
        private double maxVelocity;
        private double maxAcceleration;
        private double maxDeceleration;

        public Constraints(double maxVelocity, double maxAcceleration, double maxDeceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
        }

        public double getMaxVelocity() {
            return maxVelocity;
        }

        public void setMaxVelocity(double maxVelocity) {
            this.maxVelocity = maxVelocity;
        }

        public double getMaxAcceleration() {
            return maxAcceleration;
        }

        public void setMaxAcceleration(double maxAcceleration) {
            this.maxAcceleration = maxAcceleration;
        }

        public double getMaxDeceleration() {
            return maxDeceleration;
        }

        public void setMaxDeceleration(double maxDeceleration) {
            this.maxDeceleration = maxDeceleration;
        }
    }
}
