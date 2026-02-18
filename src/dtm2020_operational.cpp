// Author: Watson
// Purpose: Operational DTM2020 evaluation and coefficient loading (MCM variant behavior).

#include "dtm2020/dtm2020_operational.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

#include "dtm2020/sigma_uncertainty.hpp"

namespace dtm2020 {
namespace {

constexpr int kNlatm = Dtm2020Operational::Coefficients::kNlatm;
constexpr float kPiF = 3.14159265358979323846F;

struct LegendreContext {
  float p10 = 0.0F;
  float p20 = 0.0F;
  float p30 = 0.0F;
  float p40 = 0.0F;
  float p50 = 0.0F;
  float p60 = 0.0F;
  float p11 = 0.0F;
  float p21 = 0.0F;
  float p31 = 0.0F;
  float p41 = 0.0F;
  float p51 = 0.0F;
  float p22 = 0.0F;
  float p32 = 0.0F;
  float p42 = 0.0F;
  float p52 = 0.0F;
  float p62 = 0.0F;
  float p33 = 0.0F;
  float p10mg = 0.0F;
  float p20mg = 0.0F;
  float p40mg = 0.0F;
};

struct HarmonicContext {
  float ch = 0.0F;
  float sh = 0.0F;
  float c2h = 0.0F;
  float s2h = 0.0F;
  float c3h = 0.0F;
  float s3h = 0.0F;
};

using F1 = std::array<float, kNlatm + 1>;  // 1-based for parity with Fortran indexing.

float ToRadians(float deg) {
  return deg * kPiF / 180.0F;
}

float LocalTimeHoursToRadians(float hours) {
  return hours * kPiF / 12.0F;
}

double NormalizeLongitudeDeg(double longitude_deg) {
  if (longitude_deg >= 0.0 && longitude_deg < 360.0) {
    return longitude_deg;
  }
  double x = std::fmod(longitude_deg, 360.0);
  if (x < 0.0) {
    x += 360.0;
  }
  return x;
}

double NormalizeLocalTimeHours(double local_time_h) {
  if (local_time_h >= 0.0 && local_time_h < 24.0) {
    return local_time_h;
  }
  double x = std::fmod(local_time_h, 24.0);
  if (x < 0.0) {
    x += 24.0;
  }
  return x;
}

bool ParseFixedRealField(const std::string& line, std::size_t pos, std::size_t len, float& out) {
  if (pos + len > line.size()) {
    return false;
  }
  std::string token = line.substr(pos, len);
  std::replace(token.begin(), token.end(), 'D', 'E');
  std::replace(token.begin(), token.end(), 'd', 'e');
  try {
    out = std::stof(token);
  } catch (...) {
    return false;
  }
  return true;
}

bool ParseFixedIntField(const std::string& line, std::size_t pos, std::size_t len, int& out) {
  if (pos + len > line.size()) {
    return false;
  }
  std::string token = line.substr(pos, len);
  try {
    out = std::stoi(token);
  } catch (...) {
    return false;
  }
  return true;
}

float Gldtm(const std::array<float, 3>& f,
            const std::array<float, 3>& fbar,
            const std::array<float, 5>& akp,
            float day,
            const F1& a,
            F1& da,
            float ff0,
            float xlon,
            const LegendreContext& l,
            const HarmonicContext& h) {
  constexpr float rot = 0.017214206F;
  constexpr float rot2 = 0.034428412F;

  da[2] = l.p20;
  da[3] = l.p40;
  da[74] = l.p10;
  da[77] = l.p30;
  da[78] = l.p50;
  da[79] = l.p60;

  const float fmfb1 = f[1] - fbar[1];
  const float fbm1501 = fbar[1] - 150.0F;
  da[4] = fmfb1;
  da[6] = fbm1501;
  da[5] = da[4] * da[4];
  da[69] = da[6] * da[6];
  da[82] = da[4] * l.p10;
  da[83] = da[4] * l.p20;
  da[84] = da[4] * l.p30;
  da[85] = da[6] * l.p20;
  da[86] = da[6] * l.p30;
  da[87] = da[6] * l.p40;

  const int ikp = 62;
  const int ikpm = 67;
  const float c2fi = 1.0F - l.p10mg * l.p10mg;
  const float dkp = akp[1] + (a[ikp] + c2fi * a[ikp + 1]) * akp[2];
  float dakp = a[7] + a[8] * l.p20mg + a[68] * l.p40mg +
               2.0F * dkp * (a[60] + a[61] * l.p20mg + a[75] * 2.0F * dkp * dkp);
  da[ikp] = dakp * akp[2];
  da[ikp + 1] = da[ikp] * c2fi;

  const float dkpm = akp[3] + a[ikpm] * akp[4];
  const float dakpm = a[64] + a[65] * l.p20mg + a[72] * l.p40mg +
                      2.0F * dkpm * (a[66] + a[73] * l.p20mg + a[76] * 2.0F * dkpm * dkpm);
  da[ikpm] = dakpm * akp[4];

  da[7] = dkp;
  da[8] = l.p20mg * dkp;
  da[68] = l.p40mg * dkp;
  da[60] = dkp * dkp;
  da[61] = l.p20mg * da[60];
  da[75] = da[60] * da[60];
  da[64] = dkpm;
  da[65] = l.p20mg * dkpm;
  da[72] = l.p40mg * dkpm;
  da[66] = dkpm * dkpm;
  da[73] = l.p20mg * da[66];
  da[76] = da[66] * da[66];

  float f0 = a[4] * da[4] + a[5] * da[5] + a[6] * da[6] + a[69] * da[69] + a[82] * da[82] +
             a[83] * da[83] + a[84] * da[84] + a[85] * da[85] + a[86] * da[86] + a[87] * da[87];
  const float f1f = 1.0F + f0 * ff0;
  f0 = f0 + a[2] * da[2] + a[3] * da[3] + a[74] * da[74] + a[77] * da[77] + a[7] * da[7] +
       a[8] * da[8] + a[60] * da[60] + a[61] * da[61] + a[68] * da[68] + a[64] * da[64] +
       a[65] * da[65] + a[66] * da[66] + a[72] * da[72] + a[73] * da[73] + a[75] * da[75] +
       a[76] * da[76] + a[78] * da[78] + a[79] * da[79];

  da[9] = std::cos(rot * (day - a[11]));
  da[10] = l.p20 * da[9];
  da[12] = std::cos(rot2 * (day - a[14]));
  da[13] = l.p20 * da[12];

  const float coste = std::cos(rot * (day - a[18]));
  da[15] = l.p10 * coste;
  da[16] = l.p30 * coste;
  da[17] = da[6] * da[15];
  const float cos2te = std::cos(rot2 * (day - a[20]));
  da[19] = l.p10 * cos2te;
  da[39] = l.p30 * cos2te;
  da[59] = da[6] * da[19];

  da[21] = l.p11 * h.ch;
  da[22] = l.p31 * h.ch;
  da[23] = da[6] * da[21];
  da[24] = da[21] * coste;
  da[25] = l.p21 * h.ch * coste;
  da[26] = l.p11 * h.sh;
  da[27] = l.p31 * h.sh;
  da[28] = da[6] * da[26];
  da[29] = da[26] * coste;
  da[30] = l.p21 * h.sh * coste;
  da[94] = l.p51 * h.ch;
  da[95] = l.p51 * h.sh;

  da[31] = l.p22 * h.c2h;
  da[37] = l.p42 * h.c2h;
  da[32] = l.p32 * h.c2h * coste;
  da[33] = l.p22 * h.s2h;
  da[38] = l.p42 * h.s2h;
  da[34] = l.p32 * h.s2h * coste;
  da[88] = l.p32 * h.c2h;
  da[89] = l.p32 * h.s2h;
  da[90] = da[6] * da[31];
  da[91] = da[6] * da[33];
  da[92] = l.p62 * h.c2h;
  da[93] = l.p62 * h.s2h;
  da[35] = l.p33 * h.c3h;
  da[36] = l.p33 * h.s3h;

  float fp = a[9] * da[9] + a[10] * da[10] + a[12] * da[12] + a[13] * da[13] + a[15] * da[15] +
             a[16] * da[16] + a[17] * da[17] + a[19] * da[19] + a[21] * da[21] + a[22] * da[22] +
             a[23] * da[23] + a[24] * da[24] + a[25] * da[25] + a[26] * da[26] + a[27] * da[27] +
             a[28] * da[28] + a[29] * da[29] + a[30] * da[30] + a[31] * da[31] + a[32] * da[32] +
             a[33] * da[33] + a[34] * da[34] + a[35] * da[35] + a[36] * da[36] + a[37] * da[37] +
             a[38] * da[38] + a[39] * da[39] + a[59] * da[59] + a[88] * da[88] + a[89] * da[89] +
             a[90] * da[90] + a[91] * da[91] + a[92] * da[92] + a[93] * da[93] + a[94] * da[94] +
             a[95] * da[95];

  da[40] = l.p10 * coste * dkp;
  da[41] = l.p30 * coste * dkp;
  da[42] = l.p50 * coste * dkp;
  da[43] = l.p11 * h.ch * dkp;
  da[44] = l.p31 * h.ch * dkp;
  da[45] = l.p51 * h.ch * dkp;
  da[46] = l.p11 * h.sh * dkp;
  da[47] = l.p31 * h.sh * dkp;
  da[48] = l.p51 * h.sh * dkp;

  fp = fp + a[40] * da[40] + a[41] * da[41] + a[42] * da[42] + a[43] * da[43] + a[44] * da[44] +
       a[45] * da[45] + a[46] * da[46] + a[47] * da[47] + a[48] * da[48];

  dakp = (a[40] * l.p10 + a[41] * l.p30 + a[42] * l.p50) * coste +
         (a[43] * l.p11 + a[44] * l.p31 + a[45] * l.p51) * h.ch +
         (a[46] * l.p11 + a[47] * l.p31 + a[48] * l.p51) * h.sh;
  da[ikp] = da[ikp] + dakp * akp[2];
  da[ikp + 1] = da[ikp] + dakp * c2fi * akp[2];

  const float clfl = std::cos(xlon);
  da[49] = l.p11 * clfl;
  da[50] = l.p21 * clfl;
  da[51] = l.p31 * clfl;
  da[52] = l.p41 * clfl;
  da[53] = l.p51 * clfl;

  const float slfl = std::sin(xlon);
  da[54] = l.p11 * slfl;
  da[55] = l.p21 * slfl;
  da[56] = l.p31 * slfl;
  da[57] = l.p41 * slfl;
  da[58] = l.p51 * slfl;

  fp = fp + a[49] * da[49] + a[50] * da[50] + a[51] * da[51] + a[52] * da[52] + a[53] * da[53] +
       a[54] * da[54] + a[55] * da[55] + a[56] * da[56] + a[57] * da[57] + a[58] * da[58];

  return f0 + fp * f1f;
}

}  // namespace

Result<Dtm2020Operational, Error> Dtm2020Operational::LoadFromFile(
    const std::filesystem::path& coeff_file) {
  return LoadFromFile(coeff_file, Options{});
}

Result<Dtm2020Operational, Error> Dtm2020Operational::LoadFromFile(
    const std::filesystem::path& coeff_file,
    Options options) {
  Coefficients coeffs;

  std::ifstream in(coeff_file);
  if (!in.is_open()) {
    return Result<Dtm2020Operational, Error>::Err(
        MakeError(ErrorCode::kFileOpenFailed,
                  "Failed to open coefficient file",
                  coeff_file.string(),
                  "Dtm2020Operational::LoadFromFile"));
  }

  std::string title;
  std::getline(in, title);

  int npdtm = 0;
  {
    std::string second_line;
    std::getline(in, second_line);
    std::stringstream ss(second_line);
    ss >> npdtm;
  }
  if (npdtm <= 0 || npdtm > kNlatm) {
    return Result<Dtm2020Operational, Error>::Err(
        MakeError(ErrorCode::kFileParseFailed,
                  "Invalid npdtm in coefficient file",
                  coeff_file.string(),
                  "Dtm2020Operational::LoadFromFile"));
  }

  for (int i = 0; i < npdtm; ++i) {
    std::string row;
    std::getline(in, row);
    if (row.empty()) {
      --i;
      continue;
    }

    int ni = 0;
    std::array<float, 9> values{};
    bool parsed = false;

    if (ParseFixedIntField(row, 0, 4, ni)) {
      parsed = true;
      for (int k = 0; k < 9; ++k) {
        const std::size_t vpos = 4 + static_cast<std::size_t>(k) * 22;
        const std::size_t upos = 17 + static_cast<std::size_t>(k) * 22;
        float unc = 0.0F;
        if (!ParseFixedRealField(row, vpos, 13, values[k]) || !ParseFixedRealField(row, upos, 9, unc)) {
          parsed = false;
          break;
        }
      }
    }

    if (!parsed) {
      std::stringstream ss(row);
      ss >> ni;
      if (!ss) {
        return Result<Dtm2020Operational, Error>::Err(
            MakeError(ErrorCode::kFileParseFailed,
                      "Failed while parsing coefficient index",
                      coeff_file.string(),
                      "Dtm2020Operational::LoadFromFile"));
      }

      for (int k = 0; k < 9; ++k) {
        std::string vtok;
        std::string utok;
        ss >> vtok >> utok;
        if (!ss) {
          return Result<Dtm2020Operational, Error>::Err(
              MakeError(ErrorCode::kFileParseFailed,
                        "Failed while parsing tokenized coefficient row",
                        coeff_file.string(),
                        "Dtm2020Operational::LoadFromFile"));
        }
        std::replace(vtok.begin(), vtok.end(), 'D', 'E');
        std::replace(vtok.begin(), vtok.end(), 'd', 'e');
        try {
          values[k] = std::stof(vtok);
        } catch (...) {
          return Result<Dtm2020Operational, Error>::Err(
              MakeError(ErrorCode::kFileParseFailed,
                        "Failed while parsing coefficient value token",
                        coeff_file.string(),
                        "Dtm2020Operational::LoadFromFile"));
        }
      }
    }

    if (ni != i + 1) {
      return Result<Dtm2020Operational, Error>::Err(
          MakeError(ErrorCode::kFileParseFailed,
                    "Unexpected coefficient row index",
                    coeff_file.string(),
                    "Dtm2020Operational::LoadFromFile"));
    }

    const int fi = i + 1;
    coeffs.tt[fi] = values[0];
    coeffs.h[fi] = values[1];
    coeffs.he[fi] = values[2];
    coeffs.o[fi] = values[3];
    coeffs.az2[fi] = values[4];
    coeffs.o2[fi] = values[5];
    coeffs.az[fi] = values[6];
    coeffs.t0[fi] = values[7];
    coeffs.tp[fi] = values[8];
  }

  return Result<Dtm2020Operational, Error>::Ok(Dtm2020Operational(coeffs, options));
}

Result<Outputs, Error> Dtm2020Operational::Evaluate(const OperationalInputs& in) const {
  if (!std::isfinite(in.altitude_km) || !std::isfinite(in.latitude_deg) ||
      !std::isfinite(in.longitude_deg) || !std::isfinite(in.local_time_h) ||
      !std::isfinite(in.day_of_year) || !std::isfinite(in.f107) || !std::isfinite(in.f107m) ||
      !std::isfinite(in.kp_delayed_3h) || !std::isfinite(in.kp_mean_24h)) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "all inputs must be finite", {}, "Dtm2020Operational::Evaluate"));
  }
  if (in.altitude_km <= 120.0) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "altitude_km must be > 120", {}, "Dtm2020Operational::Evaluate"));
  }
  if (in.latitude_deg < -90.0 || in.latitude_deg > 90.0) {
    return Result<Outputs, Error>::Err(MakeError(
        ErrorCode::kInvalidInput, "latitude_deg must be in [-90, 90]", {}, "Dtm2020Operational::Evaluate"));
  }
  if (in.day_of_year < 1.0 || in.day_of_year > 366.0) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "day_of_year must be in [1, 366]", {}, "Dtm2020Operational::Evaluate"));
  }

  const float alti = static_cast<float>(in.altitude_km);
  const float day = static_cast<float>(in.day_of_year);
  const float latitude_rad = ToRadians(static_cast<float>(in.latitude_deg));
  const float longitude_rad = ToRadians(static_cast<float>(NormalizeLongitudeDeg(in.longitude_deg)));
  const float hl = LocalTimeHoursToRadians(static_cast<float>(NormalizeLocalTimeHours(in.local_time_h)));

  std::array<float, 3> f = {0.0F, static_cast<float>(in.f107), 0.0F};
  std::array<float, 3> fbar = {0.0F, static_cast<float>(in.f107m), 0.0F};
  std::array<float, 5> akp = {0.0F, static_cast<float>(in.kp_delayed_3h), 0.0F,
                              static_cast<float>(in.kp_mean_24h), 0.0F};

  constexpr std::array<float, 6> alefa = {-0.40F, -0.38F, 0.0F, 0.0F, 0.0F, 0.0F};
  constexpr std::array<int, 6> ma = {1, 4, 16, 28, 32, 14};
  constexpr std::array<float, 6> vma = {1.6606e-24F, 6.6423e-24F, 26.569e-24F,
                                        46.4958e-24F, 53.1381e-24F, 23.2479e-24F};

  constexpr float cpmg = 0.19081F;
  constexpr float spmg = 0.98163F;
  constexpr float xlmg = -1.2392F;
  constexpr float re = 6356.77F;
  constexpr float rgas = 831.4F;
  constexpr float gsurf = 980.665F;
  constexpr float zlb = 120.0F;

  const F1& tt = coeffs_.tt;
  const F1& h = coeffs_.h;
  const F1& he = coeffs_.he;
  const F1& o = coeffs_.o;
  const F1& az2 = coeffs_.az2;
  const F1& o2 = coeffs_.o2;
  const F1& az = coeffs_.az;
  const F1& t0 = coeffs_.t0;
  const F1& tp = coeffs_.tp;

  const float c = std::sin(latitude_rad);
  const float c2 = c * c;
  const float c4 = c2 * c2;
  const float s = std::cos(latitude_rad);
  const float s2 = s * s;

  LegendreContext l;
  l.p10 = c;
  l.p20 = 1.5F * c2 - 0.5F;
  l.p30 = c * (2.5F * c2 - 1.5F);
  l.p40 = 4.375F * c4 - 3.75F * c2 + 0.375F;
  l.p50 = c * (7.875F * c4 - 8.75F * c2 + 1.875F);
  l.p60 = (5.5F * c * l.p50 - 2.5F * l.p40) / 3.0F;
  l.p11 = s;
  l.p21 = 3.0F * c * s;
  l.p31 = s * (7.5F * c2 - 1.5F);
  l.p41 = c * s * (17.5F * c2 - 7.5F);
  l.p51 = s * (39.375F * c4 - 26.25F * c2 + 1.875F);
  l.p22 = 3.0F * s2;
  l.p32 = 15.0F * c * s2;
  l.p42 = s2 * (52.5F * c2 - 7.5F);
  l.p52 = 3.0F * c * l.p42 - 2.0F * l.p32;
  l.p62 = 2.75F * c * l.p52 - 1.75F * l.p42;
  l.p33 = 15.0F * s * s2;

  const float clmlmg = std::cos(longitude_rad - xlmg);
  const float sp = s * cpmg * clmlmg + c * spmg;
  const float cmg = sp;
  const float cmg2 = cmg * cmg;
  const float cmg4 = cmg2 * cmg2;
  l.p10mg = cmg;
  l.p20mg = 1.5F * cmg2 - 0.5F;
  l.p40mg = 4.375F * cmg4 - 3.75F * cmg2 + 0.375F;

  HarmonicContext hc;
  hc.ch = std::cos(hl);
  hc.sh = std::sin(hl);
  hc.c2h = hc.ch * hc.ch - hc.sh * hc.sh;
  hc.s2h = 2.0F * hc.ch * hc.sh;
  hc.c3h = hc.c2h * hc.ch - hc.s2h * hc.sh;
  hc.s3h = hc.s2h * hc.ch + hc.c2h * hc.sh;

  F1 da{};
  float gdelt = Gldtm(f, fbar, akp, day, tt, da, 1.0F, longitude_rad, l, hc);
  const float tinf = tt[1] * (1.0F + gdelt);

  da.fill(0.0F);
  float gdelt0 = Gldtm(f, fbar, akp, day, t0, da, 1.0F, longitude_rad, l, hc);
  const float t120 = t0[1] * (1.0F + gdelt0);

  da.fill(0.0F);
  float gdeltp = Gldtm(f, fbar, akp, day, tp, da, 1.0F, longitude_rad, l, hc);
  const float tp120 = tp[1] * (1.0F + gdeltp);

  const float sigma = tp120 / (tinf - t120);
  const float dzeta = (re + zlb) / (re + alti);
  const float zeta = (alti - zlb) * dzeta;
  const float sigzeta = sigma * zeta;
  const float expsz = std::exp(-sigzeta);
  const float tz = tinf - (tinf - t120) * expsz;

  std::array<float, 6> dbase = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

  da.fill(0.0F);
  const float gdelh = Gldtm(f, fbar, akp, day, h, da, 0.0F, longitude_rad, l, hc);
  dbase[0] = h[1] * std::exp(gdelh);

  da.fill(0.0F);
  const float gdelhe = Gldtm(f, fbar, akp, day, he, da, 0.0F, longitude_rad, l, hc);
  dbase[1] = he[1] * std::exp(gdelhe);

  da.fill(0.0F);
  const float gdelo = Gldtm(f, fbar, akp, day, o, da, 1.0F, longitude_rad, l, hc);
  dbase[2] = o[1] * std::exp(gdelo);

  const F1* az2_eval = &az2;
  F1 az2_input{};
  if (options_.emulate_mcm_transition && alti <= 150.0F) {
    az2_input = az2;
    const float atten = (alti - 120.0F) * (alti - 120.0F) * (alti - 120.0F) / 27000.0F;
    for (int idx : {21, 22, 23, 26, 27, 28, 31, 33, 35, 36, 90, 91}) {
      az2_input[idx] = atten * az2[idx];
    }
    az2_eval = &az2_input;
  }

  da.fill(0.0F);
  const float gdelaz2 = Gldtm(f, fbar, akp, day, *az2_eval, da, 1.0F, longitude_rad, l, hc);
  dbase[3] = az2[1] * std::exp(gdelaz2);

  da.fill(0.0F);
  const float gdelo2 = Gldtm(f, fbar, akp, day, o2, da, 1.0F, longitude_rad, l, hc);
  dbase[4] = o2[1] * std::exp(gdelo2);

  da.fill(0.0F);
  const float gdelaz = Gldtm(f, fbar, akp, day, az, da, 1.0F, longitude_rad, l, hc);
  dbase[5] = az[1] * std::exp(gdelaz);

  float ro = 0.0F;
  std::array<float, 6> cc = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 6> d = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

  float glb = gsurf / ((1.0F + zlb / re) * (1.0F + zlb / re));
  glb = glb / (sigma * rgas * tinf);
  const float t120tz = t120 / tz;

  for (std::size_t i = 0; i < d.size(); ++i) {
    const float gamma = static_cast<float>(ma[i]) * glb;
    const float upapg = 1.0F + alefa[i] + gamma;
    const float fz = std::pow(t120tz, upapg) * std::exp(-sigzeta * gamma);
    cc[i] = dbase[i] * fz;
    d[i] = cc[i] * vma[i];
    ro += d[i];
  }

  const float wmm = ro / (vma[0] * (cc[0] + cc[1] + cc[2] + cc[3] + cc[4] + cc[5]));

  Outputs out;
  out.temperature_k = static_cast<double>(tz);
  out.exospheric_temp_k = static_cast<double>(tinf);
  out.density_g_cm3 = static_cast<double>(ro);
  out.mean_mol_mass_g = static_cast<double>(wmm);
  out.d_h_g_cm3 = static_cast<double>(d[0]);
  out.d_he_g_cm3 = static_cast<double>(d[1]);
  out.d_o_g_cm3 = static_cast<double>(d[2]);
  out.d_n2_g_cm3 = static_cast<double>(d[3]);
  out.d_o2_g_cm3 = static_cast<double>(d[4]);
  out.d_n_g_cm3 = static_cast<double>(d[5]);

  if (!std::isfinite(out.temperature_k) || !std::isfinite(out.density_g_cm3)) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "non-finite result generated", {}, "Dtm2020Operational::Evaluate"));
  }

  return Result<Outputs, Error>::Ok(out);
}

float Dtm2020Operational::DensityUncertaintyPercent(const OperationalInputs& in) const {
  return SigmaFunctionPercent(static_cast<float>(in.latitude_deg),
                              static_cast<float>(NormalizeLocalTimeHours(in.local_time_h)),
                              static_cast<float>(in.day_of_year),
                              static_cast<float>(in.altitude_km),
                              static_cast<float>(in.f107m),
                              static_cast<float>(in.kp_delayed_3h));
}

}  // namespace dtm2020
