#include "DiceRoller.h"
#include <gtest/gtest.h>

TEST(TestDiceRoller, thresholdInstantiatesProperly) {

    DiceRoller roller = DiceRoller(480);
    EXPECT_EQ(480, roller.threshold);    
}

TEST(TestDiceRoller, thresholdUpdatesProperly) {

    const int NUM_SIDES = 20;
    DiceRoller roller = DiceRoller(NUM_SIDES);

    for (int i = 0; i < 200; i++) {

        int prevThreshold = roller.threshold;
        bool result = roller.roll();

        if (result) {
            EXPECT_EQ(roller.threshold, NUM_SIDES);
        }
        else {
            EXPECT_EQ(roller.threshold, prevThreshold-1);
        }
    }
}

TEST(TestDiceRoller, thresholdResetsWhenAt1) {

    const int NUM_SIDES = 2;
    DiceRoller roller = DiceRoller(NUM_SIDES);
    roller.threshold = 1;
    roller.roll();
    EXPECT_EQ(roller.threshold, NUM_SIDES);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


