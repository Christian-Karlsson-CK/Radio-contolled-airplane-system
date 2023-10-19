#include <gtest/gtest.h>
#include "fff.h"

DEFINE_FFF_GLOBALS;

extern "C" {
    #include "ReadBatteryVoltage.c"
}


class ReadBatteryVoltage_test : public testing::Test {
protected:
	void SetUp() override {
		//game_initialize();	/* Without this the Tests could break*/
	}
};

FAKE_VALUE_FUNC(uint16_t ,analogRead, uint8_t );





void setup()
{
    // Register resets
    RESET_FAKE(analogRead);
    FFF_RESET_HISTORY();
}


TEST_F(ReadBatteryVoltage_test, analogRead_should_be_called_once)
{
    //ARRANGE
    uint8_t TxData[32];
    //ACT
    ReadBatteryVoltage(TxData);

    //ASSERT
    
    ASSERT_EQ(analogRead_fake.call_count, 1);
}

TEST_F(ReadBatteryVoltage_test, TxData_should_contain_correct_data)
{
    //ARRANGE
    uint8_t TxData[32];
    analogRead_fake.return_val = 925;
    //ACT
    ReadBatteryVoltage(TxData);

    //ASSERT
    
    ASSERT_EQ(TxData[BAT_VOLTAGE_WHOLE], 12);
    ASSERT_EQ(TxData[BAT_VOLTAGE_DECIMAL], 74);
}