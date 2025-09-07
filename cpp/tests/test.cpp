#include <catch2/catch_test_macros.hpp>

#include "../inc/add.hpp"

TEST_CASE( "Addition (fail)", "[add]" ) {
    REQUIRE( add(0,1) == 1 );
    REQUIRE( add(0,3) == 0 );
    REQUIRE( add(0,4) == 0 );
}

TEST_CASE( "Addition (pass)", "[add]" ) {
    REQUIRE( add(0,0) == 0 );
}