#include <catch2/catch.hpp>
#include <pathplanning/pathplanning.h>

using namespace pathplanning;
TEST_CASE("arithmetic operations on cost", "[cost]") {

  Cost cost(10, Cost::NORMAL);

  REQUIRE(cost.type() == Cost::NORMAL);

  SECTION("adding UNKNOWN cost to existing cost") {
    Cost unknown_cost(1, Cost::UNKNOWN);

    Cost c1(1, Cost::NORMAL);
    c1 = c1 + unknown_cost;
    REQUIRE(c1.type() == Cost::UNKNOWN);

    Cost c2(1, Cost::IMPASSABLE);
    c2 = c2 + unknown_cost;
    REQUIRE(c2.type() == Cost::IMPASSABLE);

    Cost c3(1, Cost::FREE);
    c3 = c3 + unknown_cost;
    REQUIRE(c3.type() == Cost::UNKNOWN);
  }
}
