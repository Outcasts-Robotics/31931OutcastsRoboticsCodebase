package org.firstinspires.ftc.teamcode.utils;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.mockito.Mock;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;

public class SlewRateLimiterTest {
    @Mock
    ElapsedTime timer;

    @Before
    public void setUp() {
        MockitoAnnotations.openMocks(this);
    }

    @Test
    public void testSlewRateLimiter() {
        Mockito.when(timer.seconds()).thenReturn(1.0);
        SlewRateLimiter limiter = new SlewRateLimiter(1.0, timer);
        double result = limiter.apply(1.0);
        Assert.assertEquals(1.0, result, 0.001);
        result = limiter.apply(3.0);
        Assert.assertEquals(2.0, result, 0.001);
        result = limiter.apply(3.0);
        Assert.assertEquals(3.0, result, 0.001);
        result = limiter.apply(0.0);
        Assert.assertEquals(2.0, result, 0.001);
        result = limiter.apply(0.0);
        Assert.assertEquals(1.0, result, 0.001);
        result = limiter.apply(0.0);
        Assert.assertEquals(0.0, result, 0.001);
        result = limiter.apply(0.0);
        Assert.assertEquals(0.0, result, 0.001);
    }

}
