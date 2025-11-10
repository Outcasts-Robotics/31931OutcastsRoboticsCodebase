package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;

public class PIDControlTest {

    @Mock
    private ElapsedTime timer;

    @Before
    public void setUp() throws Exception {
        MockitoAnnotations.openMocks(this);
    }

    @Test
    public void calculate() {
        Mockito.when(timer.seconds()).thenReturn(1.0);
        PIDController pidController = new PIDController(1.0, 0.5, 0.2, timer);
        double v = 4.0;
        double t = 5.0;
        for (int i = 0; i < 100; i++) {
            double input = pidController.calculate(v, t);
            v = v + input;
            System.out.println(v);
        }
    }
}