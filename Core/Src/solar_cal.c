#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ----------------------------------------------------
// 태양 고도값 (deg)
// sunrise/sunset: 대기굴절 포함한 -0.833°
// civil dawn/dusk: 시민박명 -6°
// ----------------------------------------------------
#define SUN_ALTITUDE_SUNRISE_SUNSET  (-0.833)
#define SUN_ALTITUDE_CIVIL           (-6.0)

// ----------------------------------------------------
// REGION TABLE (한국 14개 지역)
// ----------------------------------------------------
typedef struct {
    int code;
    double lat;
    double lon;
} region_t;

static const region_t REGION_TABLE[] = {
    {0, 37.5665, 126.9780},  // Default (Seoul)
    {1, 37.5665, 126.9780},
    {2, 35.1796, 129.0756},
    {3, 35.8714, 128.6014},
    {4, 37.4563, 126.7052},
    {5, 35.1595, 126.8526},
    {6, 36.3504, 127.3845},
    {7, 35.5384, 129.3114},
    {8, 37.2636, 127.0286},
    {9, 37.8813, 127.7298},
    {10, 35.1802, 128.1076},
    {11, 36.6424, 127.4890},
    {12, 35.8242, 127.1480},
    {13, 33.4996, 126.5312},
};

// ----------------------------------------------------
// region_code → lat/lon 자동 매핑
// ----------------------------------------------------
void get_region_latlon(int region_code, double *lat, double *lon)
{
    for (int i = 0; i < (int)(sizeof(REGION_TABLE) / sizeof(region_t)); i++) {
        if (REGION_TABLE[i].code == region_code) {
            *lat = REGION_TABLE[i].lat;
            *lon = REGION_TABLE[i].lon;
            return;
        }
    }

    // fallback (Default)
    *lat = REGION_TABLE[0].lat;
    *lon = REGION_TABLE[0].lon;
}

// ----------------------------------------------------
// 각도/라디안 변환
// ----------------------------------------------------
static inline double deg2rad(double d) { return d * (M_PI / 180.0); }
static inline double rad2deg(double r) { return r * (180.0 / M_PI); }

// ----------------------------------------------------
// 태양고도 기반 사건 시각 계산 (NOAA 알고리즘)
// UTC 기준으로 계산된 시(hour)를 반환
// altitude_deg = -0.833 → sunrise/sunset
// altitude_deg = -6     → civil dawn/dusk
// ----------------------------------------------------
static double calc_solar_event_utc(
    int year, int month, int day,
    double latitude, double longitude,
    double altitude_deg)
{
    // 1) 요일 번호 계산
    int N1 = floor(275 * month / 9);
    int N2 = floor((month + 9) / 12);
    int N3 = (1 + floor((year - 4 * floor(year / 4) + 2) / 3));
    int N = N1 - (N2 * N3) + day - 30;

    // 2) 경도 보정
    double lng_hour = longitude / 15.0;

    // 3) 근사 시각 (sunrise 기준 6h)
    double t = N + ((6 - lng_hour) / 24.0);

    // 4) 평균 근점이각
    double M = (0.9856 * t) - 3.289;

    // 5) 태양의 진위각
    double L = M + (1.916 * sin(deg2rad(M))) +
                    (0.020 * sin(2 * deg2rad(M))) + 282.634;
    while (L < 0) L += 360.0;
    while (L >= 360) L -= 360.0;

    // 6) 태양 적위
    double RA = rad2deg(atan(0.91764 * tan(deg2rad(L))));
    while (RA < 0) RA += 360.0;
    while (RA >= 360) RA -= 360.0;

    double Lquadrant = floor(L / 90) * 90;
    double RAquadrant = floor(RA / 90) * 90;
    RA = (RA + (Lquadrant - RAquadrant)) / 15.0;

    // 적위 계산
    double sinDec = 0.39782 * sin(deg2rad(L));
    double cosDec = cos(asin(sinDec));

    // 8) 시각각(H)
    double cosH = (sin(deg2rad(altitude_deg)) - (sinDec * sin(deg2rad(latitude))))
                  / (cosDec * cos(deg2rad(latitude)));

    if (cosH > 1 || cosH < -1)
        return -1;  // 극야/극일

    double H = rad2deg(acos(cosH)) / 15.0;

    // 9) Local Mean Time
    double T = RA - H + (0.06571 * t) - 6.622;

    // 10) UTC로 변환
    double UT = T - lng_hour;
    while (UT < 0) UT += 24.0;
    while (UT >= 24) UT -= 24.0;

    return UT;
}

// ----------------------------------------------------
// sunrise / sunset / civil dawn / civil dusk 계산
// 결과는 "한국시간 기준 분 단위"로 반환
// ----------------------------------------------------
void compute_sun_times(
    int year, int month, int day,
    int region_code,
    int *sunrise_min,
    int *sunset_min,
    int *dawn_min,
    int *dusk_min)
{
    double lat, lon;
    get_region_latlon(region_code, &lat, &lon);

    // sunrise
    double sr_utc = calc_solar_event_utc(year, month, day,
                                         lat, lon, SUN_ALTITUDE_SUNRISE_SUNSET);

    // sunset (sunrise + 12h 근사)
    double sunset_utc = sr_utc + 12.0;
    if (sunset_utc >= 24) sunset_utc -= 24;

    // civil dawn
    double cd_utc = calc_solar_event_utc(year, month, day,
                                         lat, lon, SUN_ALTITUDE_CIVIL);

    // civil dusk = dawn + 12h
    double cs_utc = cd_utc + 12.0;
    if (cs_utc >= 24) cs_utc -= 24;

    // 한국시간 (UTC+9)
    sr_utc += 9;
    sunset_utc += 9;
    cd_utc += 9;
    cs_utc += 9;

    // 0~24h 범위 보정
    while (sr_utc < 0) sr_utc += 24;
    while (sunset_utc < 0) sunset_utc += 24;
    while (cd_utc < 0) cd_utc += 24;
    while (cs_utc < 0) cs_utc += 24;

    // 분 단위로 반환
    *sunrise_min = (int)(sr_utc * 60);
    *sunset_min  = (int)(sunset_utc * 60);
    *dawn_min    = (int)(cd_utc * 60);
    *dusk_min    = (int)(cs_utc * 60);
}
