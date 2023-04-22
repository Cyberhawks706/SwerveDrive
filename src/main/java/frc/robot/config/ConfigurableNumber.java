package frc.robot.config;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ConfigurableNumber extends Configurable<Double> {

    public ConfigurableNumber(String string, double d) {
        super(string, d);
    }

    public boolean pull() {
       this.value = SmartDashboard.getNumber(this.name, 0.0);
       this.detectChange();
       return (this.value != 0.0);
    }

    public boolean push() {
        boolean retVal = SmartDashboard.putNumber(this.name, this.value);
        this.onChangeCallback.run();
        return retVal;
    }

}