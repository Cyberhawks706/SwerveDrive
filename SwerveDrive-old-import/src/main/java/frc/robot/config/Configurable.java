package frc.robot.config;

public abstract class Configurable<T> {

    public String name;
    public volatile T value; //Voltatile modifier ensures value is updated for all threads

    protected T lastValue;


    public Configurable(String name, T defaultValue) {

        this.value = defaultValue;
        this.name = name;
        this.push();
    }

    public void setCallback(Runnable func) {
        func.run();
    }

    protected void detectChange() {
        if (this.value != this.lastValue) {
            this.onChangeCallback.run();
        }
    }

    public abstract boolean pull();

    public abstract boolean push();

    public Runnable onChangeCallback = ()->{};

    

}