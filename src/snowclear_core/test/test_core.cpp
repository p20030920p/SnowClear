// =====================================================================
// Unit tests for the ROS-free core
// ---------------------------------------------------------------------
// These run with no dataset and no ROS installation, so they are the fast
// feedback loop; the byte-exact comparison against the released reference
// output is a separate, data-dependent gate (snowclear_runner --mode all_checks).
// =====================================================================

#include "snowclear/ablation_switches.hpp"
#include "snowclear/cloud_operations_types.hpp"
#include "snowclear/feature_extractor.hpp"
#include "snowclear/logging.hpp"
#include "snowclear/param_source.hpp"
#include "snowclear/snow_detector.hpp"
#include "snowclear/system_config.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace {

using snowclear::AblationSwitches;
using snowclear::CloudFeatures;
using snowclear::CloudPoint;
using snowclear::CloudPtr;
using snowclear::FeatureExtractor;
using snowclear::FilterParameters;
using snowclear::MapParamSource;
using snowclear::RangeIntensityThreshold;
using snowclear::SnowDetector;
using snowclear::SystemConfig;

// Build a threshold identical to the released configuration's smooth form at
// the reference scene's Q1 (Q1 = 8 -> Tg = clamp(0.8*8) = 6.4).
RangeIntensityThreshold released_threshold() {
    RangeIntensityThreshold t;
    t.global_threshold = 6.4f;
    t.use_smooth_range = true;
    t.smooth_scale = 0.8f;
    t.smooth_rho = 3.0f;
    t.smooth_slope = 0.0f;
    t.smooth_r0 = 5.0f;
    t.smooth_k = 2.15f;
    t.smooth_theta = 2.38f;
    t.build_lut();
    return t;
}

// Single flat layer, far from the near-field exemption radius.
CloudPtr make_cloud(int n, float intensity, float z) {
    CloudPtr cloud(new pcl::PointCloud<CloudPoint>);
    cloud->reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        CloudPoint p;
        const float angle = 6.2831853f * static_cast<float>(i) / static_cast<float>(n);
        const float radius = 8.0f + 6.0f * static_cast<float>(i % 7) / 7.0f;
        p.x = radius * std::cos(angle);
        p.y = radius * std::sin(angle);
        p.z = z;
        p.intensity = intensity;
        cloud->push_back(p);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
}

// Two layers: a dense ground slab at z = kGroundZ (>= 5 points per 1 m cell, so
// the ground grid is actually estimated rather than falling back to global min z)
// plus query points at a fixed radius and a given height above that slab.
// Intensity of the slab is 1.0, which is NOT a support point (that needs > 1.0),
// so the surface-suppression veto stays out of the way and this isolates the
// relative-height term.
constexpr float kGroundZ = -2.0f;
constexpr float kQueryRadius = 10.0f;

CloudPtr make_layered_cloud(float query_height, float query_intensity) {
    CloudPtr cloud(new pcl::PointCloud<CloudPoint>);
    for (int ix = -12; ix <= 12; ++ix) {
        for (int iy = -12; iy <= 12; ++iy) {
            for (int k = 0; k < 6; ++k) {
                CloudPoint p;
                p.x = static_cast<float>(ix) + 0.1f * static_cast<float>(k);
                p.y = static_cast<float>(iy);
                p.z = kGroundZ;
                p.intensity = 1.0f;
                cloud->push_back(p);
            }
        }
    }
    for (int i = 0; i < 360; ++i) {
        CloudPoint p;
        const float a = 6.2831853f * static_cast<float>(i) / 360.0f;
        p.x = kQueryRadius * std::cos(a);
        p.y = kQueryRadius * std::sin(a);
        p.z = kGroundZ + query_height;
        p.intensity = query_intensity;
        cloud->push_back(p);
    }
    cloud->width = cloud->size();
    cloud->height = 1;
    return cloud;
}

// A ground grid is cached per FeatureExtractor, so each independent scenario
// needs its own. (The pipeline guarantees analyze() is called on the same cloud
// that detect() will see; reusing one extractor across different clouds would
// reuse a stale grid.)
std::vector<int> detect_indices(const CloudPtr& cloud, float global_threshold) {
    AblationSwitches switches;
    SystemConfig config;
    const FilterParameters params = config.to_filter_parameters();
    FeatureExtractor extractor(switches);
    SnowDetector detector(switches, extractor, config.detector_knn,
                          config.detector_min_intensity_score,
                          config.planarity_filter_threshold, config.direct_planarity_gate,
                          config.zero_intensity_support_radius,
                          config.zero_intensity_support_min_intensity,
                          config.zero_intensity_support_range_floor);
    RangeIntensityThreshold thresholds = released_threshold();
    thresholds.global_threshold = global_threshold;
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    detector.detect(cloud, thresholds, params, indices, 1024);
    return indices->indices;
}

struct DetectorFixture {
    AblationSwitches switches;   // defaults == released configuration
    SystemConfig config;
    FilterParameters params;
    FeatureExtractor extractor{switches};
    SnowDetector detector;

    DetectorFixture()
        : params(config.to_filter_parameters()),
          detector(switches, extractor, config.detector_knn,
                   config.detector_min_intensity_score, config.planarity_filter_threshold,
                   config.direct_planarity_gate, config.zero_intensity_support_radius,
                   config.zero_intensity_support_min_intensity,
                   config.zero_intensity_support_range_floor) {}
};

}  // namespace

// ---------------------------------------------------------------------
// Parameter plumbing
// ---------------------------------------------------------------------
TEST(ParamSource, NormalisesRosStyleKeys) {
    MapParamSource source;
    source.set("_score_threshold", "0.5");
    EXPECT_TRUE(source.has("score_threshold"));
    EXPECT_TRUE(source.has("~score_threshold"));
    EXPECT_TRUE(source.has("/node/score_threshold"));
    EXPECT_EQ(source.raw("score_threshold"), "0.5");
}

TEST(ParamSource, BooleansAcceptCommonSpellings) {
    MapParamSource source;
    source.set("a", "TRUE");
    source.set("b", "Off");
    source.set("c", "yes");
    source.set("d", "nonsense");
    EXPECT_TRUE(source.get_bool("a", false));
    EXPECT_FALSE(source.get_bool("b", true));
    EXPECT_TRUE(source.get_bool("c", false));
    EXPECT_FALSE(source.get_bool("d", false));   // unparsable -> default
}

TEST(ParamSource, MissingKeyFallsBackToDefault) {
    MapParamSource source;
    EXPECT_EQ(source.get_int("nope", 7), 7);
    EXPECT_FLOAT_EQ(source.get_float("nope", 1.5f), 1.5f);
    EXPECT_EQ(source.get_string("nope", "x"), "x");
}

TEST(Config, DefaultsAreTheReleasedConfiguration) {
    const SystemConfig config;
    EXPECT_FLOAT_EQ(config.score_threshold, 0.75f);
    EXPECT_DOUBLE_EQ(config.xy_threshold, 17.0);
    EXPECT_DOUBLE_EQ(config.height_threshold, 2.6);
    EXPECT_FLOAT_EQ(config.idsor_scale, 0.8f);
    EXPECT_FLOAT_EQ(config.idsor_rho, 3.0f);
    EXPECT_FLOAT_EQ(config.zero_intensity_support_min_intensity, 1.0f);

    const AblationSwitches switches;
    EXPECT_TRUE(switches.use_idsor_intensity_threshold);
    EXPECT_TRUE(switches.enable_zero_intensity_surface_suppression);
    EXPECT_TRUE(switches.enable_local_threshold_adjustment);
    EXPECT_TRUE(switches.enable_relative_height);
    // The paper describes these, but the released configuration disables them.
    EXPECT_FALSE(switches.enable_planarity_calculation);
    EXPECT_FALSE(switches.enable_density_calculation);
    EXPECT_FALSE(switches.enable_feature_entropy);
    EXPECT_FALSE(switches.enable_grid_search_optimization);
}

TEST(Config, ToMapCoversEveryKeyTheLoaderReads) {
    const SystemConfig config;
    const AblationSwitches switches;
    // 57 numeric/path parameters + 52 switches, as verified by param_check.
    EXPECT_EQ(config.to_map().size(), 57u);
    EXPECT_EQ(switches.to_map().size(), 52u);
    EXPECT_EQ(config.to_map().at("score_threshold"), "0.75");
    EXPECT_EQ(switches.to_map().at("use_idsor_intensity_threshold"), "true");
}

TEST(Config, ValidateClampsIllegalValues) {
    MapParamSource source;
    source.set("detector_type", "not_a_detector");
    source.set("block_size", "0");
    source.set("xy_threshold", "-3");
    SystemConfig config;
    config.load(source);
    config.validate();
    EXPECT_EQ(config.detector_type, "feature_fusion");
    EXPECT_GE(config.block_size, 1);
    EXPECT_GT(config.xy_threshold, 0.0);
}

// ---------------------------------------------------------------------
// Decision-function invariants — the documented behaviour of METHOD.md
// ---------------------------------------------------------------------
TEST(DetectorInvariants, NoPointWithIntensityTwoOrMoreIsEverSnow) {
    DetectorFixture f;
    ASSERT_FALSE(f.switches.enable_planarity_calculation);

    const RangeIntensityThreshold thresholds = released_threshold();

    for (float intensity : {2.0f, 3.0f, 5.0f, 8.0f, 40.0f, 255.0f}) {
        const CloudPtr cloud = make_cloud(500, intensity, -0.5f);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        f.detector.detect(cloud, thresholds, f.params, indices, 1024);
        EXPECT_TRUE(indices->indices.empty())
            << "intensity " << intensity << " produced " << indices->indices.size()
            << " detections; the released threshold makes this impossible";
    }
}

TEST(DetectorInvariants, ZeroIntensityPointsSurviveTheScoreTest) {
    DetectorFixture f;
    const RangeIntensityThreshold thresholds = released_threshold();

    // Bright surface points far from the query points, so the near-field
    // exemption applies (r <= 7 m is exempt from surface suppression).
    const CloudPtr cloud = make_cloud(500, 0.0f, -0.5f);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    f.detector.detect(cloud, thresholds, f.params, indices, 1024);
    EXPECT_EQ(indices->indices.size(), cloud->size())
        << "I = 0 with no support points must all be classified as snow";
}

TEST(DetectorInvariants, GroundLevelIntensityOneIsNeverSnow) {
    // I = 1 sitting on the ground needs S > 0.9643, i.e. I/T < 0.0298, i.e.
    // T > 33. The released configuration caps T at 0.8 * 8.0 = 6.4.
    const std::vector<int> hits = detect_indices(make_layered_cloud(0.0f, 1.0f), 6.4f);
    EXPECT_TRUE(hits.empty()) << "ground-level I = 1 produced " << hits.size() << " detections";
}

TEST(DetectorInvariants, DetectionCountIsMonotoneInHeightAboveGround) {
    // The score is C = 0.7*S + 0.15*hag while the threshold depends only on S,
    // so raising a point above the local ground can only ever help it. This must
    // hold pointwise, hence also in the aggregate.
    std::vector<size_t> counts;
    for (float height : {0.0f, 0.5f, 1.0f, 1.6f, 2.5f}) {
        counts.push_back(detect_indices(make_layered_cloud(height, 1.0f), 6.4f).size());
    }
    for (size_t i = 1; i < counts.size(); ++i) {
        EXPECT_LE(counts[i - 1], counts[i])
            << "raising the query layer reduced detections at height step " << i
            << " (" << counts[i - 1] << " -> " << counts[i] << ")";
    }
    EXPECT_GT(counts.back(), 0u)
        << "no height made I = 1 recoverable, so the height term is inert";
}

TEST(DetectorInvariants, SerialAndOpenMpAgreeBitForBit) {
    const RangeIntensityThreshold thresholds = released_threshold();

    std::vector<int> serial, parallel;
    for (const bool use_openmp : {false, true}) {
        AblationSwitches switches;
        switches.enable_openmp_parallel = use_openmp;
        SystemConfig config;
        FilterParameters params = config.to_filter_parameters();
        FeatureExtractor extractor(switches);
        SnowDetector detector(switches, extractor, config.detector_knn,
                              config.detector_min_intensity_score,
                              config.planarity_filter_threshold, config.direct_planarity_gate,
                              config.zero_intensity_support_radius,
                              config.zero_intensity_support_min_intensity,
                              config.zero_intensity_support_range_floor);

        // Mixed intensities so the decision actually branches.
        CloudPtr cloud(new pcl::PointCloud<CloudPoint>);
        for (int i = 0; i < 4000; ++i) {
            CloudPoint p;
            const float a = 6.2831853f * static_cast<float>(i) / 4000.0f;
            const float r = 3.0f + 12.0f * static_cast<float>(i % 11) / 11.0f;
            p.x = r * std::cos(a);
            p.y = r * std::sin(a);
            p.z = -0.8f + 0.4f * static_cast<float>(i % 5);
            p.intensity = static_cast<float>(i % 4);   // 0,1,2,3
            cloud->push_back(p);
        }
        cloud->width = cloud->size();
        cloud->height = 1;

        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        detector.detect(cloud, thresholds, params, indices, 512);
        (use_openmp ? parallel : serial) = indices->indices;
    }

    EXPECT_EQ(serial, parallel) << "OpenMP must not change the detection set";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // The pipeline logs a line or two per call by design; keep the test output
    // readable and assert-free by installing a discard sink.
    snowclear::set_log_sink([](snowclear::LogLevel, const std::string&) {});
    return RUN_ALL_TESTS();
}
