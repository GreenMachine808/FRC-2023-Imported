package frc.robot;

public class timedAction implements Runnable {

    public interface action {
        void act();
    }

    long desired_millis;
    action Action;

    public timedAction(long desired_millis, action Action) {
        this.desired_millis = desired_millis;
        this.Action = Action;
    }

    @Override
    public void run() {
        Action.act();
        try {
            Thread.sleep(desired_millis);
        } catch(InterruptedException e) {
            System.out.println("Error: desired_millis was negative");
        }
    }
}
