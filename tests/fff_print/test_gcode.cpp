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

        double cos_theta = dir.x();
        double sin_theta = dir.y();
        constexpr double inv_sqrt2 = 0.7071067811865475;
        double factor_A = std::abs(cos_theta + sin_theta) * inv_sqrt2;
        double factor_B = std::abs(cos_theta - sin_theta) * inv_sqrt2;

        auto bounds = gcodegen.compute_resonance_speeds_for_test(60.0, dir, 10.0);

        THEN("Toolhead speed of 60.0 is in danger") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(20.0 / factor_B, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(60.0 / factor_A, 0.0001));
        }
    }
}

SCENARIO("Resonance Avoidance — safe and boundary cases", "[GCode]") {
    // CoreXY at 0° gives factor_A = factor_B = 0.70710678
    constexpr double inv_sqrt2 = 0.7071067811865475;
    constexpr double a_proj_lo = 50.0 / inv_sqrt2; // 70.7107
    constexpr double a_proj_hi = 60.0 / inv_sqrt2; // 84.8528
    constexpr double b_proj_lo = 20.0 / inv_sqrt2; // 28.2843
    constexpr double b_proj_hi = 30.0 / inv_sqrt2; // 42.4264
    // Zones [28.28,42.43] and [70.71,84.85] do NOT overlap → two separate zones

    WHEN("Toolhead speed is below all merged zones") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(10.0, dir, 10.0);

        THEN("Toolhead is NOT in danger, no below zone stored") {
            REQUIRE(bounds.is_in_danger == false);
            REQUIRE(bounds.danger_lo < 0.0);
            REQUIRE(bounds.danger_hi < 0.0);
        }
    }

    WHEN("Toolhead speed is above all merged zones") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(100.0, dir, 10.0);

        THEN("Toolhead is NOT in danger, highest below zone is stored for volumetric cap") {
            REQUIRE(bounds.is_in_danger == false);
            // Highest zone below 100 is A's [70.71,84.85]
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(a_proj_lo, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(a_proj_hi, 0.0001));
        }
    }
}

SCENARIO("Resonance Avoidance — zone merging edge cases", "[GCode]") {
    constexpr double inv_sqrt2 = 0.7071067811865475;

    WHEN("Overlapping zones from both motors are merged into one") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        // A:[40,80] B:[50,70] → projected: A=[56.57,113.14] B=[70.71,98.99] → merged [56.57,113.14]
        gcodegen.set_resonance_speeds_for_test({40.0, 80.0}, {50.0, 70.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(75.0, dir, 10.0);

        THEN("Merged zone covers the union of both zones") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(40.0 / inv_sqrt2, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(80.0 / inv_sqrt2, 0.0001));
        }
    }

    WHEN("Factor is zero for one motor (CoreXY 45°)") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(inv_sqrt2, inv_sqrt2); // 45° → factor_A=1.0, factor_B=0.0
        auto bounds = gcodegen.compute_resonance_speeds_for_test(55.0, dir, 10.0);

        THEN("Only motor A's zones are checked") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(50.0, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(60.0, 0.0001));
        }
    }

    WHEN("Multiple danger zones per motor") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        // A:[30,35, 65,75] B:[10,15]
        // projected: A=[42.43,49.50] [91.92,106.07] B=[14.14,21.21]
        gcodegen.set_resonance_speeds_for_test({30.0, 35.0, 65.0, 75.0}, {10.0, 15.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(46.0, dir, 10.0);

        THEN("All zones are projected and the correct merged zone is identified") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(30.0 / inv_sqrt2, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(35.0 / inv_sqrt2, 0.0001));
        }
    }
}

SCENARIO("Resonance Avoidance — legacy and kinematics paths", "[GCode]") {
    WHEN("No per-motor config — legacy global fallback, toolhead inside zone") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.resonance_min_segment_length.value = 0.0;
        config.min_resonance_avoidance_speed.value = 40.0;
        config.max_resonance_avoidance_speed.value = 80.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({}, {});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(60.0, dir, 10.0);

        THEN("Legacy [40,80] danger zone is used") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(40.0, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(80.0, 0.0001));
        }
    }

    WHEN("No per-motor config — toolhead above zone, volumetric cap guard") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.resonance_min_segment_length.value = 0.0;
        config.min_resonance_avoidance_speed.value = 40.0;
        config.max_resonance_avoidance_speed.value = 80.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({}, {});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(90.0, dir, 10.0);

        THEN("Zone is stored as below_lo/below_hi for volumetric cap") {
            REQUIRE(bounds.is_in_danger == false);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(40.0, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(80.0, 0.0001));
        }
    }

    WHEN("No per-motor config — toolhead below zone") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.resonance_min_segment_length.value = 0.0;
        config.min_resonance_avoidance_speed.value = 40.0;
        config.max_resonance_avoidance_speed.value = 80.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({}, {});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(20.0, dir, 10.0);

        THEN("No bounds set") {
            REQUIRE(bounds.is_in_danger == false);
            REQUIRE(bounds.danger_lo < 0.0);
            REQUIRE(bounds.danger_hi < 0.0);
        }
    }

    WHEN("I3 kinematics is configured") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psI3;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(0.8, 0.6); // factor_A=0.8, factor_B=0.6
        constexpr double factor_A = 0.8;
        constexpr double factor_B = 0.6;
        // Projected: A=[62.5,75] B=[33.33,50] — no overlap, gap at 50-62.5
        // toolhead=70 falls inside A's zone
        auto bounds = gcodegen.compute_resonance_speeds_for_test(70.0, dir, 10.0);

        THEN("I3 kinematics correctly projects motor speeds") {
            REQUIRE(bounds.is_in_danger == true);
            REQUIRE_THAT(bounds.danger_lo, Catch::Matchers::WithinRel(50.0 / factor_A, 0.0001));
            REQUIRE_THAT(bounds.danger_hi, Catch::Matchers::WithinRel(60.0 / factor_A, 0.0001));
        }
    }

    WHEN("Delta kinematics — always returns empty bounds") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psDelta;
        config.resonance_min_segment_length.value = 0.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(55.0, dir, 10.0);

        THEN("Delta returns no danger") {
            REQUIRE(bounds.is_in_danger == false);
            REQUIRE(bounds.danger_lo < 0.0);
            REQUIRE(bounds.danger_hi < 0.0);
        }
    }

    WHEN("Segment is shorter than the minimum threshold") {
        Slic3r::GCode gcodegen;
        FullPrintConfig config;
        config.printer_structure.value = psCoreXY;
        config.resonance_min_segment_length.value = 5.0;
        gcodegen.apply_print_config(config);
        gcodegen.set_resonance_speeds_for_test({50.0, 60.0}, {20.0, 30.0});

        Vec2d dir(1.0, 0.0);
        auto bounds = gcodegen.compute_resonance_speeds_for_test(55.0, dir, 3.0);

        THEN("Short segments are skipped (empty bounds)") {
            REQUIRE(bounds.is_in_danger == false);
            REQUIRE(bounds.danger_lo < 0.0);
            REQUIRE(bounds.danger_hi < 0.0);
        }
    }
}

SCENARIO("apply_resonance_clamp variations", "[GCode]") {
    WHEN("Not in danger") {
        ResonanceSpeedBounds bounds{};
        bounds.is_in_danger = false;
        bounds.danger_lo = 50.0;
        bounds.danger_hi = 100.0;

        double result_lower = GCode::apply_resonance_clamp_for_test(60.0, bounds, ResonanceAvoidanceMode::ClampLower);
        double result_nearest = GCode::apply_resonance_clamp_for_test(60.0, bounds, ResonanceAvoidanceMode::Nearest);

        THEN("Returns base_spd unchanged in any mode") {
            REQUIRE_THAT(result_lower, Catch::Matchers::WithinRel(60.0, 0.0001));
            REQUIRE_THAT(result_nearest, Catch::Matchers::WithinRel(60.0, 0.0001));
        }
    }

    WHEN("ClampLower mode") {
        ResonanceSpeedBounds bounds{};
        bounds.is_in_danger = true;
        bounds.danger_lo = 40.0;
        bounds.danger_hi = 100.0;

        double result = GCode::apply_resonance_clamp_for_test(60.0, bounds, ResonanceAvoidanceMode::ClampLower);

        THEN("Speed is reduced to danger_lo") {
            REQUIRE_THAT(result, Catch::Matchers::WithinRel(40.0, 0.0001));
        }
    }

    WHEN("Nearest mode — lower boundary is closer") {
        ResonanceSpeedBounds bounds{};
        bounds.is_in_danger = true;
        bounds.danger_lo = 50.0;
        bounds.danger_hi = 100.0;

        double result = GCode::apply_resonance_clamp_for_test(60.0, bounds, ResonanceAvoidanceMode::Nearest);

        THEN("Speed goes down to danger_lo") {
            REQUIRE_THAT(result, Catch::Matchers::WithinRel(50.0, 0.0001));
        }
    }

    WHEN("Nearest mode — upper boundary is closer") {
        ResonanceSpeedBounds bounds{};
        bounds.is_in_danger = true;
        bounds.danger_lo = 50.0;
        bounds.danger_hi = 100.0;

        double result = GCode::apply_resonance_clamp_for_test(90.0, bounds, ResonanceAvoidanceMode::Nearest);

        THEN("Speed goes UP to danger_hi") {
            REQUIRE_THAT(result, Catch::Matchers::WithinRel(100.0, 0.0001));
        }
    }
}
