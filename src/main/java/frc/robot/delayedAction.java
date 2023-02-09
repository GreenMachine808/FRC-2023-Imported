package frc.robot;

public class delayedAction implements Runnable {

    public interface action {
        void act();
    }

    long desired_millis;
    action Action;

    public delayedAction(long desired_millis, action Action) {
        this.desired_millis = desired_millis;
        this.Action = Action;
    }

    @Override
    public void run() {
        try {
            Thread.sleep(desired_millis);
        } catch(InterruptedException e) {
            System.out.println("Error: desired_millis was negative");
        }
        Action.act();

        //interrupt(); (?)
    }
}
