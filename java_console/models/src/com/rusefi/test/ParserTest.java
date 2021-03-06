package com.rusefi.test;

import com.fathzer.soft.javaluator.DoubleEvaluator;
import org.junit.Test;

import java.text.ParseException;

import static org.junit.Assert.assertEquals;

public class ParserTest {
    @Test
    public void testBooleanConversion() throws ParseException {
        assertParse("1 2 + 3 -", "1 + 2 - 3");
        assertParse("1 2 | 3 |", "1 | 2 | 3");

        assertParse("2 1 >", "2 > 1");
        assertParse("rpm 0 >", "rpm > false");
        assertParse("rpm 0 >", "(rpm > false)");
        assertParse("rpm user0 > clt user2 > |", "(rpm > user0) or (clt > user2)");
        assertParse("1 2 | 3 |", "1 | 2 | 3");
        assertParse("rpm user0 > clt user2 > | vbatt user1 > |", "(rpm > user0) or (clt > user2) or (vbatt > user1)");
    }

    private void assertParse(String rpn, String expression) {
        DoubleEvaluator evaluator = new DoubleEvaluator();
        evaluator.evaluate(expression.toLowerCase());

        assertEquals(rpn, evaluator.getRusEfi());
    }


    @Test
    public void test() {
        assertValue("2 3 6 ^ negate +", "2+-3^6", -727.0);
    }

    @Test
    public void testBooleanNot2() throws Exception {
        assertValue("0 !", "! 0", 1);
        assertValue("0 !", "not 0", 1);
        assertValue("1 !", "not(1)", 0);
        assertValue("0 !", "not(0)", 1);
//        assertValue("1 0 | not 0 & 1 0 | |", "(((true | false) & not(false)) | (true | false))", 1);
    }

    private void assertValue(String expectedRpn, String expression, double expectedValue) {
        DoubleEvaluator evaluator = new DoubleEvaluator();
        assertEquals(expectedValue, evaluator.evaluate(expression), 0.001);
        assertParse(expectedRpn, expression);
    }

    @Test
    public void testRusEfi() {
        assertParse("1 4 |", "1 or 4");
        assertParse("1 4 |", "1 OR 4");

        assertParse("time_since_boot 4 <", "(time_since_boot < 4)");
        assertParse("1 4 |", "1 | 4");
        assertParse("time_since_boot 4 < rpm 0 > |", "(time_since_boot < 4) | (rpm > 0)");

        assertParse("1 4 |", "1 or 4");
        assertParse("1 4 |", "1 OR 4");

        assertParse("1 4 &", "1 & 4");

        assertParse("coolant fan_off_setting >", "(coolant > fan_off_setting)");
        assertParse("1 3 |", "1 OR 3");
        assertParse("1 3 &", "1 and 3");
        assertParse("1 coolant fan_on_setting > |", "1 OR (coolant > fan_on_setting)");
        assertParse("fan coolant fan_off_setting > & coolant fan_on_setting > |", "(fan and (coolant > fan_off_setting)) OR (coolant > fan_on_setting)");

        assertParse("time_since_boot 4 <= rpm 0 > |", "(time_since_boot <= 4) | (rpm > 0)");

        assertParse("time_since_boot 4 <= rpm 0 > |", "(time_since_boot <= 4) | (rpm > 0)");

        assertParse("time_since_boot 4 <= rpm 0 > |", "(time_since_boot <= 4) OR (rpm > 0)");

        assertParse("self rpm 4800 >= & rpm 5000 > |", "(self and (rpm >= 4800)) OR (rpm > 5000)");


    }
}