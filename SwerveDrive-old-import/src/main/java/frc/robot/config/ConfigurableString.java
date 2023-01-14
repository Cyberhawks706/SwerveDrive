package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ConfigurableString extends Configurable<String> {

    public ConfigurableString(String name, String defaultValue) {
        super(name, defaultValue);
    }

    public boolean pull() {
       this.value = SmartDashboard.getString(this.name, "");
       return (this.value != "");
    }

    public boolean push() {
        boolean retVal = SmartDashboard.putString(this.name, this.value);
        this.onChangeCallback.run();
        return  retVal;
    }

}