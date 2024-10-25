/**
 * @file wgs84.h
 *
 * @brief World Geodetic System 1984 (WGS84) reference ellipsoid parameters.
 *
 * @author Kendra Hale, <kendrahale98@gmail.com>
 *
 * @date 10/25/24
 *
 * Reference: Datum Transformation of GPS Positions,
 *            https://community.ptc.com/sejnu66972/attachments/sejnu66972/PTCMathcad/57177/2/Datum%20Transformations.pdf
 */

#include <cmath>

/**
 * @class WGS84Params
 * @brief Class of constants containing WGS84 reference ellipsoid parameters.
 */
class WGS84Params {
    public:
        // Semi-major axis, a [m]; and semi-minor axis, b [m]
        static constexpr double A = 6378137;
        static constexpr double B = 6356752.31424518;
        
        // Flattenning factor, f [unitless]
        static constexpr double F = 1 / 298.257223563;

        // For calculating eccentricities
        static constexpr double a_squared = std::pow(A, 2);
        static constexpr double b_squared = std::pow(B, 2);
        static constexpr double ab_squared_diff = a_squared - b_squared;

        // First eccentricity, e [m]
        static constexpr double E = std::sqrt(ab_squared_diff / a_squared);

        // Second eccentricity, e' [m]
        static constexpr double E2 = std::sqrt(ab_squared_diff / b_squared);
};
