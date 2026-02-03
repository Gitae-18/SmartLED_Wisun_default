#ifndef SOLAR_CALC_H
#define SOLAR_CALC_H

#include <stdint.h>

// ----------------------------------------------------
// 한국 지역 region code table (0~13)
// ----------------------------------------------------
typedef struct {
    int code;
    double lat;
    double lon;
} region_t;

// 지역 코드 → 위도/경도 반환
void get_region_latlon(int region_code, double *lat, double *lon);

// 태양 시간 계산 (한국 기준, 분 단위)
// - year: YYYY
// - month: 1~12
// - day: 1~31
// - region_code: 0~13 (한국 14개 지역)
// 출력 (분 단위):
// - sunrise_min: 일출
// - sunset_min:  일몰
// - dawn_min:    시민박명 시작 (-6°)
// - dusk_min:    시민박명 종료 (-6°)
void compute_sun_times(
    int year,
    int month,
    int day,
    int region_code,
    int *sunrise_min,
    int *sunset_min,
    int *dawn_min,
    int *dusk_min
);

#endif // SOLAR_CALC_H
