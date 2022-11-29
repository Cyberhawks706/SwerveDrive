package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ConfigurableBool extends Configurable<Boolean> {

    public ConfigurableBool(String name, boolean defaultValue) {
        super(name, defaultValue);
    }

    public boolean pull() {
       this.value = SmartDashboard.getBoolean(this.name, false);
       this.detectChange();
       return (this.value != false);
    }

    public boolean push() {
        boolean retVal = SmartDashboard.putBoolean(this.name, this.value);
        this.onChangeCallback.run();
        return retVal;
    }

}