import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import frc.robot.subsystems.Intake;
import static org.junit.jupiter.api.Assertions.assertEquals;

class IntakeTest {
	Intake intake;
	REVPhysicsSim sim;
	CANSparkMax spark;

	@BeforeEach
	void setup() {
		intake = new Intake();
		sim = REVPhysicsSim.getInstance();
		spark = intake.getSpark();
		sim.addSparkMax(spark, 0.97f, 11000);
	}

	@AfterEach
	void shutdown() throws Exception{
		intake.close();
	}

	@Test
	void testSet() {
		intake.set(1);
		assertEquals(1, spark.get());
		intake.set(0);
		assertEquals(0, spark.get());
		intake.set(-1);
		assertEquals(-1, spark.get());
	}

	@Test
	void testStop() {
		intake.set(1);
		intake.stop();
		assertEquals(0, spark.get());
		intake.set(-1);
		intake.stop();
		assertEquals(0, spark.get());
		intake.set(0);
		intake.stop();
		assertEquals(0, spark.get());
	}
}
