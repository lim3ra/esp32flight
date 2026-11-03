#include "geo_math.h"
#include <math.h>

#define EARTH_R_KM 6371.0088
#define DEG2RAD(d) ((d) * M_PI / 180.0)

double geo_haversine_km(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = DEG2RAD(lat2 - lat1);
    double dlon = DEG2RAD(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(DEG2RAD(lat1)) * cos(DEG2RAD(lat2)) *
               sin(dlon / 2) * sin(dlon / 2);
    return 2 * EARTH_R_KM * atan2(sqrt(a), sqrt(1 - a));
}

double geo_bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double p1 = DEG2RAD(lat1), p2 = DEG2RAD(lat2);
    double dl = DEG2RAD(lon2 - lon1);
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) * 180.0 / M_PI;
    return fmod(b + 360.0, 360.0);
}

void geo_gc_point(double lat1, double lon1, double lat2, double lon2,
                  double f, double *lat, double *lon)
{
    double p1 = DEG2RAD(lat1), l1 = DEG2RAD(lon1);
    double p2 = DEG2RAD(lat2), l2 = DEG2RAD(lon2);
    double d = 2 * asin(sqrt(pow(sin((p2 - p1) / 2), 2) +
                             cos(p1) * cos(p2) * pow(sin((l2 - l1) / 2), 2)));
    if (d < 1e-9) {
        *lat = lat1;
        *lon = lon1;
        return;
    }
    double a = sin((1 - f) * d) / sin(d);
    double b = sin(f * d) / sin(d);
    double x = a * cos(p1) * cos(l1) + b * cos(p2) * cos(l2);
    double y = a * cos(p1) * sin(l1) + b * cos(p2) * sin(l2);
    double z = a * sin(p1) + b * sin(p2);
    *lat = atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI;
    *lon = atan2(y, x) * 180.0 / M_PI;
}

double geo_elevation_deg(double dist_km, int alt_ft)
{
    double alt_km = alt_ft * 0.0003048;
    if (dist_km < 0.05) {
        return 90.0;
    }
    return atan2(alt_km, dist_km) * 180.0 / M_PI;
}

bool geo_cpa(double home_lat, double home_lon,
             double ac_lat, double ac_lon, double track_deg, double gs_kts,
             double *t_s, double *cpa_km)
{
    /* local flat-earth approximation around home, km */
    double coslat = cos(home_lat * M_PI / 180.0);
    double rx = (ac_lon - home_lon) * 111.32 * coslat;
    double ry = (ac_lat - home_lat) * 110.57;
    double v_kms = gs_kts * 1.852 / 3600.0;
    double tr = track_deg * M_PI / 180.0;
    double vx = sin(tr) * v_kms;
    double vy = cos(tr) * v_kms;
    double v2 = vx * vx + vy * vy;
    if (v2 < 1e-9) {
        return false;
    }
    double t = -(rx * vx + ry * vy) / v2;
    if (t <= 0) {
        return false;   /* already past the closest point */
    }
    double cx = rx + vx * t;
    double cy = ry + vy * t;
    *t_s = t;
    *cpa_km = sqrt(cx * cx + cy * cy);
    return true;
}

bool geo_route_plausible(double orig_lat, double orig_lon,
                         double dest_lat, double dest_lon,
                         double cur_lat, double cur_lon)
{
    double direct = geo_haversine_km(orig_lat, orig_lon, dest_lat, dest_lon);
    double detour = geo_haversine_km(orig_lat, orig_lon, cur_lat, cur_lon) +
                    geo_haversine_km(cur_lat, cur_lon, dest_lat, dest_lon);
    /* Real flights fly close to the great circle. The slack scales with
     * the leg: 22% covers weather reroutes on long hauls, the 60 km floor
     * covers patterns around short domestic hops - a flat +150 km used to
     * make the corridor wider than Poland on WRO-GDN-class routes. */
    double slack = direct * 0.22;
    if (slack < 60.0) {
        slack = 60.0;
    }
    return detour <= direct + slack;
}

int geo_route_fit_dir(double orig_lat, double orig_lon,
                      double dest_lat, double dest_lon,
                      double cur_lat, double cur_lon,
                      float track_deg, float gs_kts, int vrate_fpm)
{
    if (!geo_route_plausible(orig_lat, orig_lon, dest_lat, dest_lon,
                             cur_lat, cur_lon)) {
        return GEO_FIT_NO;
    }
    /* Vertical asymmetry: nobody descends onto their claimed ORIGIN or
     * climbs out of their claimed DESTINATION. Catches the reversed
     * shuttle leg right at the airport, where the track test must stay
     * quiet because of SIDs and patterns. */
    double from_orig = geo_haversine_km(cur_lat, cur_lon, orig_lat, orig_lon);
    double to_dest = geo_haversine_km(cur_lat, cur_lon, dest_lat, dest_lon);
    if (vrate_fpm < -400 && from_orig < 60.0 && to_dest > 150.0) {
        return GEO_FIT_NO;
    }
    if (vrate_fpm > 400 && to_dest < 60.0 && from_orig > 150.0) {
        return GEO_FIT_NO;
    }
    /* The corridor test cannot tell the outbound leg from the return leg
     * (same great circle, opposite direction) - the classic stale-database
     * failure on shuttle routes. En route, the ground track has to point
     * roughly at the destination. Skip the check at low speed (holds,
     * approaches), close to either airport (SIDs/patterns) and when no
     * track is known. */
    if (gs_kts < 100.0f || track_deg < 0.0f) {
        return GEO_FIT_WEAK;
    }
    if (geo_haversine_km(cur_lat, cur_lon, dest_lat, dest_lon) < 80.0 ||
        geo_haversine_km(cur_lat, cur_lon, orig_lat, orig_lon) < 50.0) {
        return GEO_FIT_WEAK;
    }
    double want = geo_bearing_deg(cur_lat, cur_lon, dest_lat, dest_lon);
    double diff = fabs((double)track_deg - want);
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    return diff <= 100.0 ? GEO_FIT_CONFIRMED : GEO_FIT_NO;
}

bool geo_route_plausible_dir(double orig_lat, double orig_lon,
                             double dest_lat, double dest_lon,
                             double cur_lat, double cur_lon,
                             float track_deg, float gs_kts, int vrate_fpm)
{
    return geo_route_fit_dir(orig_lat, orig_lon, dest_lat, dest_lon,
                             cur_lat, cur_lon, track_deg, gs_kts,
                             vrate_fpm) != GEO_FIT_NO;
}

double geo_progress(double orig_lat, double orig_lon,
                    double dest_lat, double dest_lon,
                    double cur_lat, double cur_lon)
{
    double flown = geo_haversine_km(orig_lat, orig_lon, cur_lat, cur_lon);
    double remaining = geo_haversine_km(cur_lat, cur_lon, dest_lat, dest_lon);
    double total = flown + remaining;
    if (total < 1.0) {
        return 0.0;
    }
    double p = flown / total;
    return p < 0.0 ? 0.0 : (p > 1.0 ? 1.0 : p);
}

double geo_lon_unwrap(double ref, double lon)
{
    double d = fmod(lon - ref + 540.0, 360.0);
    if (d < 0) {
        d += 360.0;
    }
    return ref + d - 180.0;
}

bool geo_sun_times(double lat, double lon, long long epoch_utc, int tz_off_s,
                   int *rise_min, int *set_min)
{
    /* NOAA solar-position approximation, good to a minute or two */
    double days = (double)epoch_utc / 86400.0 + 2440587.5 - 2451545.0;
    double g = fmod(357.529 + 0.98560028 * days, 360.0) * M_PI / 180.0;
    double q = fmod(280.459 + 0.98564736 * days, 360.0);
    double l = fmod(q + 1.915 * sin(g) + 0.020 * sin(2 * g), 360.0) * M_PI / 180.0;
    double e = (23.439 - 0.00000036 * days) * M_PI / 180.0;
    double decl = asin(sin(e) * sin(l));

    double latr = lat * M_PI / 180.0;
    double h0 = -0.833 * M_PI / 180.0;   /* standard refraction horizon */
    double cosh0 = (sin(h0) - sin(latr) * sin(decl)) / (cos(latr) * cos(decl));
    if (cosh0 < -1.0 || cosh0 > 1.0) {
        return false;   /* midnight sun or polar night */
    }
    double ha = acos(cosh0) * 180.0 / M_PI;   /* half day arc in degrees */

    /* equation of time, minutes */
    double ra = atan2(cos(e) * sin(l), cos(l)) * 180.0 / M_PI;
    double eqt = q - fmod(ra + 360.0, 360.0);
    while (eqt > 20) eqt -= 360;
    while (eqt < -20) eqt += 360;
    eqt *= 4.0;

    double noon_utc_min = 720.0 - 4.0 * lon - eqt;
    double rise_utc = noon_utc_min - ha * 4.0;
    double set_utc = noon_utc_min + ha * 4.0;
    int off_min = tz_off_s / 60;
    int r = (int)(rise_utc + off_min);
    int st = (int)(set_utc + off_min);
    while (r < 0) r += 1440;
    while (r >= 1440) r -= 1440;
    while (st < 0) st += 1440;
    while (st >= 1440) st -= 1440;
    *rise_min = r;
    *set_min = st;
    return true;
}

double geo_sun_elevation(double lat, double lon, long long epoch_utc)
{
    double days = (double)epoch_utc / 86400.0 + 2440587.5 - 2451545.0;
    double g = fmod(357.529 + 0.98560028 * days, 360.0) * M_PI / 180.0;
    double q = fmod(280.459 + 0.98564736 * days, 360.0);
    double l = fmod(q + 1.915 * sin(g) + 0.020 * sin(2 * g), 360.0) * M_PI / 180.0;
    double e = (23.439 - 0.00000036 * days) * M_PI / 180.0;
    double decl = asin(sin(e) * sin(l));

    double ra = atan2(cos(e) * sin(l), cos(l)) * 180.0 / M_PI;
    double eqt = q - fmod(ra + 360.0, 360.0);
    while (eqt > 20) eqt -= 360;
    while (eqt < -20) eqt += 360;
    eqt *= 4.0;

    /* hour angle from true solar time, degrees */
    double utc_min = fmod((double)epoch_utc, 86400.0) / 60.0;
    double h = ((utc_min + 4.0 * lon + eqt) / 4.0 - 180.0) * M_PI / 180.0;
    double latr = lat * M_PI / 180.0;
    return asin(sin(latr) * sin(decl) + cos(latr) * cos(decl) * cos(h)) * 180.0 / M_PI;
}
