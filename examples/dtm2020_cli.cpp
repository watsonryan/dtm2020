// Author: Watson
// Purpose: Minimal CLI for single-point DTM2020 operational evaluation.

#include <cstdlib>
#include <cerrno>
#include <iomanip>
#include <iostream>

#include "dtm2020/dtm2020_operational.hpp"

namespace {

bool ParseDoubleArg(const char* text, double& out) {
  errno = 0;
  char* end = nullptr;
  out = std::strtod(text, &end);
  return errno == 0 && end != text && end != nullptr && *end == '\0';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 11) {
    std::cerr << "usage: dtm2020_cli <coeff_file> <alt_km> <lat_deg> <lon_deg> <lt_h> <doy> <f107> <f107m> <kp3h> <kp24h>\n";
    return EXIT_FAILURE;
  }

  auto model = dtm2020::Dtm2020Operational::LoadFromFile(argv[1]);
  if (!model) {
    std::cerr << "load error: " << model.error().message << "\n";
    return EXIT_FAILURE;
  }

  dtm2020::OperationalInputs in{};
  if (!ParseDoubleArg(argv[2], in.altitude_km) || !ParseDoubleArg(argv[3], in.latitude_deg) ||
      !ParseDoubleArg(argv[4], in.longitude_deg) || !ParseDoubleArg(argv[5], in.local_time_h) ||
      !ParseDoubleArg(argv[6], in.day_of_year) || !ParseDoubleArg(argv[7], in.f107) ||
      !ParseDoubleArg(argv[8], in.f107m) || !ParseDoubleArg(argv[9], in.kp_delayed_3h) ||
      !ParseDoubleArg(argv[10], in.kp_mean_24h)) {
    std::cerr << "parse error: all numeric arguments must be valid floating-point values\n";
    return EXIT_FAILURE;
  }

  auto out = model.value().Evaluate(in);
  if (!out) {
    std::cerr << "evaluate error: " << out.error().message << "\n";
    return EXIT_FAILURE;
  }

  std::cout << std::setprecision(10)
            << "temperature_k=" << out.value().temperature_k << "\n"
            << "exospheric_temp_k=" << out.value().exospheric_temp_k << "\n"
            << "density_g_cm3=" << out.value().density_g_cm3 << "\n"
            << "mean_mol_mass_g=" << out.value().mean_mol_mass_g << "\n";
  return EXIT_SUCCESS;
}
