#include <gtest/gtest.h>

#include "depthai_bridge/Process.hpp"

TEST(Process, ArgumentsAreLiteralAndExitFailuresPropagate) {
    EXPECT_EQ(depthai_bridge::runProcess({"/bin/echo", "a b", "$(literal)", "semi;colon"}), "a b $(literal) semi;colon\n");
    EXPECT_THROW(depthai_bridge::runProcess({"/bin/false"}), std::runtime_error);
    EXPECT_THROW(depthai_bridge::runProcess({"/no/such/program"}), std::runtime_error);
}
TEST(Process, QuotedXacroValuesStayTogether) {
    const auto args = depthai_bridge::splitArguments("a:='a b' c:=\"c d\" e:=f\\ g");
    EXPECT_EQ(args, (std::vector<std::string>{"a:=a b", "c:=c d", "e:=f g"}));
    EXPECT_THROW(depthai_bridge::splitArguments("a:='missing"), std::invalid_argument);
}
