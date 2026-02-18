// Author: Watson
// Purpose: Operational model shell with input validation and sigma forwarding.

#include "dtm2020/dtm2020_operational.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "dtm2020/sigma_uncertainty.hpp"

namespace dtm2020 {
namespace {

constexpr double kPi = 3.14159265358979323846;

double NormalizeLongitudeDeg(double longitude_deg) {
  double x = std::fmod(longitude_deg, 360.0);
  if (x < 0.0) {
    x += 360.0;
  }
  return x;
}

double NormalizeLocalTimeHours(double local_time_h) {
  double x = std::fmod(local_time_h, 24.0);
  if (x < 0.0) {
    x += 24.0;
  }
  return x;
}

}  // namespace

std::optional<Dtm2020Operational> Dtm2020Operational::LoadFromFile(
    const std::filesystem::path& coeff_file,
    Error& error) {
  return LoadFromFile(coeff_file, error, Options{});
}

std::optional<Dtm2020Operational> Dtm2020Operational::LoadFromFile(
    const std::filesystem::path& coeff_file,
    Error& error,
    Options options) {
  Coefficients coeffs;
  std::ifstream in(coeff_file);
  if (!in.is_open()) {
    error = {ErrorCode::kFileOpenFailed, "Failed to open coefficient file"};
    return std::nullopt;
  }

  std::string title;
  std::getline(in, title);

  int npdtm = 0;
  in >> npdtm;
  if (!in || npdtm <= 0 || npdtm > Coefficients::kNlatm) {
    error = {ErrorCode::kFileParseFailed, "Invalid npdtm in coefficient file"};
    return std::nullopt;
  }

  for (int i = 0; i < npdtm; ++i) {
    int ni = 0;
    float dtt = 0.0F, dh = 0.0F, dhe = 0.0F, dox = 0.0F, daz2 = 0.0F;
    float do2 = 0.0F, daz = 0.0F, dt0 = 0.0F, dtp = 0.0F;
    in >> ni >> coeffs.tt[i] >> dtt >> coeffs.h[i] >> dh >> coeffs.he[i] >> dhe >>
        coeffs.o[i] >> dox >> coeffs.az2[i] >> daz2 >> coeffs.o2[i] >> do2 >>
        coeffs.az[i] >> daz >> coeffs.t0[i] >> dt0 >> coeffs.tp[i] >> dtp;

    if (!in) {
      error = {ErrorCode::kFileParseFailed, "Failed while parsing coefficient row"};
      return std::nullopt;
    }
  }

  if (!in.good() && !in.eof()) {
    error = {ErrorCode::kFileParseFailed, "Coefficient stream ended unexpectedly"};
    return std::nullopt;
  }
  error = {};
  return Dtm2020Operational(coeffs, options);
}

Outputs Dtm2020Operational::Evaluate(const OperationalInputs& in, Error& error) const {
  if (in.altitude_km <= 120.0) {
    error = {ErrorCode::kInvalidInput, "altitude_km must be > 120"};
    return {};
  }
  if (in.latitude_deg < -90.0 || in.latitude_deg > 90.0) {
    error = {ErrorCode::kInvalidInput, "latitude_deg must be in [-90, 90]"};
    return {};
  }

  const double longitude_deg = NormalizeLongitudeDeg(in.longitude_deg);
  const double local_time_h = NormalizeLocalTimeHours(in.local_time_h);
  (void)longitude_deg;
  (void)local_time_h;
  (void)kPi;

  error = {};
  return {};
}

float Dtm2020Operational::DensityUncertaintyPercent(const OperationalInputs& in) const {
  return SigmaFunctionPercent(static_cast<float>(in.latitude_deg),
                              static_cast<float>(in.local_time_h),
                              static_cast<float>(in.day_of_year),
                              static_cast<float>(in.altitude_km),
                              static_cast<float>(in.f107m),
                              static_cast<float>(in.kp_delayed_3h));
}

}  // namespace dtm2020
