#include <catch2/catch_all.hpp>

#include <memory>

#include "libslic3r/GCode.hpp"

using namespace Slic3r;

SCENARIO("Origin manipulation", "[GCode]") {
	Slic3r::GCode gcodegen;
	WHEN("set_origin to (10,0)") {
    	gcodegen.set_origin(Vec2d(10,0));
    	REQUIRE(gcodegen.origin() == Vec2d(10, 0));
    }
	WHEN("set_origin to (10,0) and translate by (5, 5)") {
		gcodegen.set_origin(Vec2d(10,0));
		gcodegen.set_origin(gcodegen.origin() + Vec2d(5, 5));
		THEN("origin returns reference to point") {
    		REQUIRE(gcodegen.origin() == Vec2d(15,5));
    	}
    }
}

SCENARIO("Resonance Avoidance Unified Checks", "[GCode]") {
    Slic3r::GCode gcodegen;
    FullPrintConfig config;
    config.printer_structure.value = psCoreXY;
    config.resonance_min_segment_length.value = 0.0;
    gcodegen.apply_print_config(config);

    // Motor A resonance zone: [50, 60]
    // Motor B resonance zone: [20, 30]
    gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

    WHEN("Direction with factor_A = 0.9 and factor_B = 0.5, toolhead_speed = 60") {
        Vec2d dir(0.9899, 0.2828);
        dir.normalize();

        // Let's compute actual factors with this dir:
        double cos_theta = dir.x();
        double sin_theta = dir.y();
        constexpr double inv_sqrt2 = 0.7071067811865475;
        double factor_A = std::abs(cos_theta + sin_theta) * inv_sqrt2;
        double factor_B = std::abs(cos_theta - sin_theta) * inv_sqrt2;

        auto bounds = gcodegen.compute_resonance_speeds_for_test(60.0, dir, 10.0);

        THEN("Toolhead speed of 60.0 is in danger") {
            REQUIRE(bounds.is_in_danger == true);
            // Combined danger zone should be [20/factor_B, 60/factor_A]
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(20.0 / factor_B, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(60.0 / factor_A, 0.0001));
        }
    }
}
