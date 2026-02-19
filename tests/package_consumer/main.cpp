#include "dtm2020/dtm2020_operational.hpp"

int main() {
  dtm2020::OperationalInputs in{};
  in.altitude_km = 400.0;
  in.latitude_deg = 0.0;
  in.longitude_deg = 0.0;
  in.local_time_h = 12.0;
  in.day_of_year = 180.0;
  in.f107 = 120.0;
  in.f107m = 120.0;
  in.kp_delayed_3h = 3.0;
  in.kp_mean_24h = 2.0;
  return (in.altitude_km > 0.0) ? 0 : 1;
}
