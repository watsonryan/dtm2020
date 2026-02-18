// Author: Watson
// Purpose: Minimal CLI for single-point DTM2020 operational evaluation.

#include <cstdlib>
#include <iomanip>
#include <iostream>

#include "dtm2020/dtm2020_operational.hpp"

int main(int argc, char** argv) {
  if (argc != 11) {
    std::cerr << "usage: dtm2020_cli <coeff_file> <alt_km> <lat_deg> <lon_deg> <lt_h> <doy> <f107> <f107m> <kp3h> <kp24h>\n";
    return EXIT_FAILURE;
  }

  dtm2020::Error error;
  auto model = dtm2020::Dtm2020Operational::LoadFromFile(argv[1], error);
  if (!model) {
    std::cerr << "load error: " << error.message << "\n";
    return EXIT_FAILURE;
  }

  dtm2020::OperationalInputs in{};
  in.altitude_km = std::atof(argv[2]);
  in.latitude_deg = std::atof(argv[3]);
  in.longitude_deg = std::atof(argv[4]);
  in.local_time_h = std::atof(argv[5]);
  in.day_of_year = std::atof(argv[6]);
  in.f107 = std::atof(argv[7]);
  in.f107m = std::atof(argv[8]);
  in.kp_delayed_3h = std::atof(argv[9]);
  in.kp_mean_24h = std::atof(argv[10]);

  auto out = model->Evaluate(in, error);
  if (error.code != dtm2020::ErrorCode::kNone) {
    std::cerr << "evaluate error: " << error.message << "\n";
    return EXIT_FAILURE;
  }

  std::cout << std::setprecision(10)
            << "temperature_k=" << out.temperature_k << "\n"
            << "exospheric_temp_k=" << out.exospheric_temp_k << "\n"
            << "density_g_cm3=" << out.density_g_cm3 << "\n"
            << "mean_mol_mass_g=" << out.mean_mol_mass_g << "\n";
  return EXIT_SUCCESS;
}
