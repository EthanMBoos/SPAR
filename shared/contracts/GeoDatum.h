#pragma once
#include <cmath>

// Boundary between SPAR's canonical local metric frame and global coordinates.
//
// SPAR reasons, plans, and trains in a local metric ENU frame (x east, y north,
// z up; meters). Latitude/longitude is a boundary concern only: GPS waypoints are
// converted local at ingestion, and local poses are converted back for
// georeferenced logging or Tower. A GeoDatum anchors the local frame to the globe.
//
//   valid == false  → local-only operation (indoor / no georeference). Conversions
//                     are undefined and must not be called.
//   valid == true   → origin set from a GPS fix, navsat_transform, or a survey.
//
// Flat-earth (equirectangular) tangent-plane projection about the origin: correct
// to sub-meter over robot-scale distances, which is all the local frame spans.
struct GeoDatum {
    double origin_lat_deg = 0.0;
    double origin_lon_deg = 0.0;
    float  origin_alt_m   = 0.0f;
    bool   valid          = false;
};

struct LocalXYZ { float x_m = 0.0f; float y_m = 0.0f; float z_m = 0.0f; };
struct GeoPoint { double lat_deg = 0.0; double lon_deg = 0.0; float alt_m = 0.0f; };

namespace geo_detail {
constexpr double kMetersPerDegLat = 111320.0;
constexpr double kDegRad          = 3.14159265358979323846 / 180.0;
}

inline LocalXYZ lla_to_local(const GeoDatum& d, double lat_deg, double lon_deg, float alt_m) {
    using namespace geo_detail;
    double cos_lat = std::cos(d.origin_lat_deg * kDegRad);
    LocalXYZ out;
    out.y_m = static_cast<float>((lat_deg - d.origin_lat_deg) * kMetersPerDegLat);
    out.x_m = static_cast<float>((lon_deg - d.origin_lon_deg) * kMetersPerDegLat * cos_lat);
    out.z_m = alt_m - d.origin_alt_m;
    return out;
}

inline GeoPoint local_to_lla(const GeoDatum& d, float x_m, float y_m, float z_m) {
    using namespace geo_detail;
    double cos_lat = std::cos(d.origin_lat_deg * kDegRad);
    GeoPoint out;
    out.lat_deg = d.origin_lat_deg + static_cast<double>(y_m) / kMetersPerDegLat;
    out.lon_deg = d.origin_lon_deg +
                  static_cast<double>(x_m) / (kMetersPerDegLat * (cos_lat > 1e-9 ? cos_lat : 1e-9));
    out.alt_m   = d.origin_alt_m + z_m;
    return out;
}
