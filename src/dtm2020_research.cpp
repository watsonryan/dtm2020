/**
 * @file dtm2020_research.cpp
 * @brief Implementation of research DTM2020 (dtm5/F30-ap60) loading and evaluation.
 */

#include "dtm2020/dtm2020_research.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>

namespace dtm2020 {
namespace {

constexpr int kNlatm = Dtm2020Research::Coefficients::kNlatm;
constexpr float kPiF = 3.14159265358979323846F;

using F1 = std::array<float, kNlatm + 1>;  // 1-based for parity with Fortran indexing.

struct LegendreContextHp {
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
  float p11mg = 0.0F;
  float p22mg = 0.0F;
  float p31mg = 0.0F;
  float p30mg = 0.0F;
  float p50mg = 0.0F;
  float p60mg = 0.0F;
};

struct HarmonicContext {
  float ch = 0.0F;
  float sh = 0.0F;
  float c2h = 0.0F;
  float s2h = 0.0F;
  float c3h = 0.0F;
  float s3h = 0.0F;
};

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

float BintOpenEnded(float ap) {
  static constexpr std::array<float, 38> aap = {
      0.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 9.0F, 12.0F, 15.0F, 18.0F, 22.0F, 27.0F,
      32.0F, 39.0F, 48.0F, 56.0F, 67.0F, 80.0F, 94.0F, 111.0F, 132.0F, 154.0F, 179.0F,
      207.0F, 236.0F, 265.0F, 294.0F, 324.0F, 355.0F, 388.0F, 421.0F, 456.0F, 494.0F,
      534.0F, 574.0F, 617.0F, 657.0F};
  static constexpr std::array<float, 38> kp = {
      0.0F, 0.33F, 0.66F, 1.0F, 1.33F, 1.66F, 2.0F, 2.33F, 2.66F, 3.0F, 3.33F, 3.66F,
      4.0F, 4.33F, 4.66F, 5.0F, 5.33F, 5.66F, 6.0F, 6.33F, 6.66F, 7.0F, 7.33F, 7.66F,
      8.0F, 8.33F, 8.66F, 9.0F, 9.33F, 9.66F, 10.0F, 10.33F, 10.66F, 11.0F, 11.33F,
      11.66F, 12.0F, 12.33F};

  if (ap <= aap.front()) {
    return kp.front();
  }
  if (ap >= aap.back()) {
    return kp.back();
  }
  const auto it = std::upper_bound(aap.begin(), aap.end(), ap);
  const std::size_t i = static_cast<std::size_t>(std::distance(aap.begin(), it));
  const std::size_t i1 = i - 1;
  if (aap[i1] == ap) {
    return kp[i1];
  }
  return kp[i1] + ((kp[i] - kp[i1]) / (aap[i] - aap[i1])) * (ap - aap[i1]);
}

void Geogm(float xlat_deg, float xlon_deg, float& gmlat_deg, float& gmlon_deg) {
  const float rfac = kPiF / 180.0F;
  const float platr = 78.5F * rfac;
  const float plongr = 291.0F * rfac;
  const float spl = std::sin(platr);
  const float cpl = std::cos(platr);
  const float rlat = xlat_deg * rfac;

  float xlong_local = xlon_deg;
  if (xlong_local == 291.0F) {
    xlong_local = 291.1F;
  }
  const float rlong = xlong_local * rfac;

  const float slm = spl * std::sin(rlat) + cpl * std::cos(rlat) * std::cos(plongr - rlong);
  const float clm = std::sqrt(std::max(0.0F, 1.0F - slm * slm));
  const float phim1 = std::cos(rlat) * std::sin(rlong - plongr) / clm;
  const float phim2 = (spl * slm - std::sin(rlat)) / (cpl * clm);

  gmlat_deg = std::asin(slm) / rfac;

  const float aphim1 = std::abs(std::asin(std::max(-1.0F, std::min(1.0F, phim1))));
  if (phim1 >= 0.0F && phim2 >= 0.0F) {
    gmlon_deg = aphim1 / rfac;
  } else if (phim1 >= 0.0F && phim2 < 0.0F) {
    gmlon_deg = (kPiF - aphim1) / rfac;
  } else if (phim1 < 0.0F && phim2 < 0.0F) {
    gmlon_deg = (kPiF + aphim1) / rfac;
  } else {
    gmlon_deg = (2.0F * kPiF - aphim1) / rfac;
  }
}

float GldtmHp(const std::array<float, 3>& f,
              const std::array<float, 3>& fbar,
              std::array<float, 9> akp,
              float day,
              const F1& a,
              F1& da,
              float ff0,
              float xlon,
              const LegendreContextHp& l,
              const HarmonicContext& hctx) {
  constexpr float rot = 0.017214206F;
  constexpr float rot2 = 0.034428412F;

  da[2] = l.p20;
  da[3] = l.p40;
  da[74] = l.p10;
  da[77] = l.p30;
  da[78] = l.p50;
  da[79] = l.p60;

  da[4] = f[1] - fbar[1];
  da[6] = fbar[1] - 150.0F;
  da[5] = da[4] * da[4];
  da[69] = da[6] * da[6];
  da[82] = da[4] * l.p10;
  da[83] = da[4] * l.p20;
  da[84] = da[4] * l.p30;
  da[85] = da[6] * l.p20;
  da[86] = da[6] * l.p30;
  da[87] = da[6] * l.p40;

  if (akp[1] >= 9.0F && akp[3] > 7.5F) akp[1] = 9.0F + (akp[1] - 9.0F) / 5.0F;
  if (akp[4] >= 9.0F && akp[3] > 7.5F) akp[4] = 9.0F + (akp[4] - 9.0F) / 5.0F;
  if (akp[5] >= 9.0F && akp[3] > 7.5F) akp[5] = 9.0F + (akp[5] - 9.0F) / 5.0F;
  if (akp[6] >= 9.0F && akp[3] > 7.5F) akp[6] = 9.0F + (akp[6] - 9.0F) / 5.0F;
  if (akp[7] >= 9.0F && akp[3] > 7.5F) akp[7] = 9.0F + (akp[7] - 9.0F) / 5.0F;
  if (akp[8] >= 9.0F && akp[3] > 7.5F) akp[8] = 9.0F + (akp[8] - 9.0F) / 5.0F;

  const float dkp = akp[1];
  const float dkpm = akp[3];

  da[7] = dkp;
  da[8] = l.p20mg * dkp;
  da[60] = dkp * dkp;
  da[61] = l.p20mg * da[60];
  da[62] = l.p30mg * dkp;
  da[63] = l.p10mg * dkp;
  da[67] = l.p60mg * dkp;
  da[68] = l.p40mg * dkp;

  da[64] = dkpm;
  da[65] = l.p20mg * dkpm;
  da[66] = dkpm * dkpm;
  da[73] = l.p20mg * da[66];

  const float flux = std::max(f[1], fbar[1]);
  const int iflux = static_cast<int>(flux);

  if (iflux >= 200) {
    da[75] = 0.333F * da[60] * da[60];
    da[76] = 0.1F * akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = 0.1F * akp[8] * akp[8] * akp[8] * akp[8];
    da[71] = 0.1F * akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = 0.1F * akp[6] * akp[6] * akp[6] * akp[6];
  } else if (iflux >= 190) {
    da[75] = 0.55F * da[60] * da[60];
    da[76] = 0.15F * akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = 0.15F * akp[8] * akp[8] * akp[8] * akp[8];
    da[71] = 0.15F * akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = 0.15F * akp[6] * akp[6] * akp[6] * akp[6];
  } else if (iflux >= 180) {
    da[75] = 0.733F * da[60] * da[60];
    da[76] = 0.2F * akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = 0.2F * akp[8] * akp[8] * akp[8] * akp[8];
    da[71] = 0.2F * akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = 0.2F * akp[6] * akp[6] * akp[6] * akp[6];
  } else if (iflux >= 160) {
    da[75] = da[60] * da[60];
    da[76] = 0.4F * akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = 0.4F * akp[8] * akp[8] * akp[8] * akp[8];
    da[71] = 0.4F * akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = 0.4F * akp[6] * akp[6] * akp[6] * akp[6];
  } else if (iflux >= 140) {
    da[75] = da[60] * da[60];
    da[76] = 0.8F * akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = 0.8F * akp[8] * akp[8] * akp[8] * akp[8];
    da[71] = 0.8F * akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = 0.8F * akp[6] * akp[6] * akp[6] * akp[6];
  } else {
    da[75] = da[60] * da[60];
    da[71] = akp[5] * akp[5] * akp[5] * akp[5];
    da[72] = akp[6] * akp[6] * akp[6] * akp[6];
    da[76] = akp[7] * akp[7] * akp[7] * akp[7];
    da[79] = akp[8] * akp[8] * akp[8] * akp[8];
  }
  da[70] = akp[2];

  float f0 = a[4] * da[4] + a[5] * da[5] + a[6] * da[6] + a[69] * da[69] + a[82] * da[82] +
             a[83] * da[83] + a[84] * da[84] + a[85] * da[85] + a[86] * da[86] + a[87] * da[87];
  const float f1f = 1.0F + f0 * ff0;

  f0 = f0 + a[2] * da[2] + a[3] * da[3] + a[74] * da[74] + a[77] * da[77] + a[7] * da[7] +
       a[8] * da[8] + a[60] * da[60] + a[61] * da[61] + a[68] * da[68] + a[64] * da[64] +
       a[65] * da[65] + a[66] * da[66] + a[72] * da[72] + a[73] * da[73] + a[75] * da[75] +
       a[76] * da[76] + a[78] * da[78] + a[79] * da[79] + a[70] * da[70] + a[71] * da[71] +
       a[62] * da[62] + a[63] * da[63] + a[67] * da[67];

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

  da[21] = l.p11 * hctx.ch;
  da[22] = l.p31 * hctx.ch;
  da[23] = da[6] * da[21];
  da[24] = da[21] * coste;
  da[25] = l.p21 * hctx.ch * coste;
  da[26] = l.p11 * hctx.sh;
  da[27] = l.p31 * hctx.sh;
  da[28] = da[6] * da[26];
  da[29] = da[26] * coste;
  da[30] = l.p21 * hctx.sh * coste;
  da[94] = l.p51 * hctx.ch;
  da[95] = l.p51 * hctx.sh;

  da[31] = l.p22 * hctx.c2h;
  da[37] = l.p42 * hctx.c2h;
  da[32] = l.p32 * hctx.c2h * coste;
  da[33] = l.p22 * hctx.s2h;
  da[38] = l.p42 * hctx.s2h;
  da[34] = l.p32 * hctx.s2h * coste;
  da[88] = l.p32 * hctx.c2h;
  da[89] = l.p32 * hctx.s2h;
  da[90] = da[6] * da[31];
  da[91] = da[6] * da[33];
  da[92] = l.p62 * hctx.c2h;
  da[93] = l.p62 * hctx.s2h;
  da[35] = l.p33 * hctx.c3h;
  da[36] = l.p33 * hctx.s3h;

  float fp = a[9] * da[9] + a[10] * da[10] + a[12] * da[12] + a[13] * da[13] + a[15] * da[15] +
             a[16] * da[16] + a[17] * da[17] + a[19] * da[19] + a[21] * da[21] + a[22] * da[22] +
             a[23] * da[23] + a[24] * da[24] + a[25] * da[25] + a[26] * da[26] + a[27] * da[27] +
             a[28] * da[28] + a[29] * da[29] + a[30] * da[30] + a[31] * da[31] + a[32] * da[32] +
             a[33] * da[33] + a[34] * da[34] + a[35] * da[35] + a[36] * da[36] + a[37] * da[37] +
             a[38] * da[38] + a[39] * da[39] + a[59] * da[59] + a[88] * da[88] + a[89] * da[89] +
             a[90] * da[90] + a[91] * da[91] + a[92] * da[92] + a[93] * da[93] + a[94] * da[94] +
             a[95] * da[95];

  da[40] = l.p10mg * cos2te * dkpm;
  da[41] = l.p10mg * coste * dkpm;
  da[42] = l.p10mg * cos2te * dkp;
  da[43] = l.p11mg * hctx.ch * dkp;
  da[44] = l.p31mg * hctx.ch * dkp;
  da[45] = l.p22mg * hctx.c2h * dkp;
  da[46] = l.p11mg * hctx.sh * dkp;
  da[47] = l.p31mg * hctx.sh * dkp;
  da[48] = l.p22mg * hctx.s2h * dkp;

  fp = fp + a[40] * da[40] + a[41] * da[41] + a[42] * da[42] + a[43] * da[43] + a[44] * da[44] +
       a[45] * da[45] + a[46] * da[46] + a[47] * da[47] + a[48] * da[48];

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

Result<Dtm2020Research, Error> Dtm2020Research::LoadFromFile(const std::filesystem::path& coeff_file) {
  Coefficients coeffs;

  std::ifstream in(coeff_file);
  if (!in.is_open()) {
    return Result<Dtm2020Research, Error>::Err(
        MakeError(ErrorCode::kFileOpenFailed,
                  "Failed to open coefficient file",
                  coeff_file.string(),
                  "Dtm2020Research::LoadFromFile"));
  }

  std::string title;
  std::getline(in, title);
  std::string second_line;
  std::getline(in, second_line);

  int npdtm = 0;
  {
    std::stringstream ss(second_line);
    ss >> npdtm;
  }
  if (npdtm <= 0 || npdtm > kNlatm) {
    return Result<Dtm2020Research, Error>::Err(
        MakeError(ErrorCode::kFileParseFailed,
                  "Invalid npdtm in coefficient file",
                  coeff_file.string(),
                  "Dtm2020Research::LoadFromFile"));
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
        return Result<Dtm2020Research, Error>::Err(
            MakeError(ErrorCode::kFileParseFailed,
                      "Failed while parsing coefficient index",
                      coeff_file.string(),
                      "Dtm2020Research::LoadFromFile"));
      }
      for (int k = 0; k < 9; ++k) {
        std::string vtok;
        std::string utok;
        ss >> vtok >> utok;
        if (!ss) {
          return Result<Dtm2020Research, Error>::Err(
              MakeError(ErrorCode::kFileParseFailed,
                        "Failed while parsing tokenized coefficient row",
                        coeff_file.string(),
                        "Dtm2020Research::LoadFromFile"));
        }
        std::replace(vtok.begin(), vtok.end(), 'D', 'E');
        std::replace(vtok.begin(), vtok.end(), 'd', 'e');
        try {
          values[k] = std::stof(vtok);
        } catch (...) {
          return Result<Dtm2020Research, Error>::Err(
              MakeError(ErrorCode::kFileParseFailed,
                        "Failed while parsing coefficient value token",
                        coeff_file.string(),
                        "Dtm2020Research::LoadFromFile"));
        }
      }
    }

    if (ni != i + 1) {
      return Result<Dtm2020Research, Error>::Err(
          MakeError(ErrorCode::kFileParseFailed,
                    "Unexpected coefficient row index",
                    coeff_file.string(),
                    "Dtm2020Research::LoadFromFile"));
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

  return Result<Dtm2020Research, Error>::Ok(Dtm2020Research(coeffs));
}

Result<Outputs, Error> Dtm2020Research::Evaluate(const ResearchInputs& in) const {
  if (!std::isfinite(in.altitude_km) || !std::isfinite(in.latitude_deg) || !std::isfinite(in.longitude_deg) ||
      !std::isfinite(in.local_time_h) || !std::isfinite(in.day_of_year) || !std::isfinite(in.f30) ||
      !std::isfinite(in.f30m)) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "all scalar inputs must be finite", {}, "Dtm2020Research::Evaluate"));
  }
  for (double ap : in.ap60) {
    if (!std::isfinite(ap)) {
      return Result<Outputs, Error>::Err(
          MakeError(ErrorCode::kInvalidInput, "ap60 inputs must be finite", {}, "Dtm2020Research::Evaluate"));
    }
  }
  if (in.altitude_km <= 120.0) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "altitude_km must be > 120", {}, "Dtm2020Research::Evaluate"));
  }
  if (in.latitude_deg < -90.0 || in.latitude_deg > 90.0) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "latitude_deg must be in [-90, 90]", {}, "Dtm2020Research::Evaluate"));
  }
  if (in.day_of_year < 1.0 || in.day_of_year > 366.0) {
    return Result<Outputs, Error>::Err(
        MakeError(ErrorCode::kInvalidInput, "day_of_year must be in [1, 366]", {}, "Dtm2020Research::Evaluate"));
  }

  const float alti = static_cast<float>(in.altitude_km);
  const float day = static_cast<float>(in.day_of_year);
  const float dlat_deg = static_cast<float>(in.latitude_deg);
  const float dlon_deg = static_cast<float>(NormalizeLongitudeDeg(in.longitude_deg));
  const float alat = ToRadians(dlat_deg);
  const float xlon = ToRadians(dlon_deg);
  const float hl = LocalTimeHoursToRadians(static_cast<float>(NormalizeLocalTimeHours(in.local_time_h)));

  std::array<float, 3> f = {0.0F, static_cast<float>(in.f30), 0.0F};
  std::array<float, 3> fbar = {0.0F, static_cast<float>(in.f30m), 0.0F};

  std::array<float, 9> akp = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 10> ap60f{};
  for (std::size_t i = 0; i < ap60f.size(); ++i) {
    ap60f[i] = static_cast<float>(in.ap60[i]);
  }
  std::array<float, 10> kp60{};
  for (std::size_t i = 0; i < kp60.size(); ++i) {
    kp60[i] = BintOpenEnded(ap60f[i]);
  }

  const int latabs = static_cast<int>(std::abs(dlat_deg));
  if (latabs >= 70 && latabs <= 90) {
    const float xl75 = (90.0F - static_cast<float>(latabs)) / 20.0F;
    akp[1] = BintOpenEnded((1.0F - xl75) *
                               ((ap60f[2] + ap60f[1] + ap60f[3]) / 3.0F) +
                           xl75 * ((ap60f[2] + ap60f[3] + ap60f[4]) / 3.0F));
  } else if (latabs >= 30 && latabs <= 69) {
    const float xl45 = (69.0F - static_cast<float>(latabs)) / 40.0F;
    akp[1] = BintOpenEnded((1.0F - xl45) *
                               ((ap60f[2] + ap60f[3] + ap60f[4]) / 3.0F) +
                           xl45 * ((ap60f[4] + ap60f[3] + ap60f[0]) / 3.0F));
  } else {
    akp[1] = BintOpenEnded((ap60f[3] + ap60f[4] + ap60f[0]) / 3.0F);
  }

  akp[2] = kp60[1] - kp60[2];
  akp[3] = kp60[5];
  akp[4] = 0.0F;
  akp[5] = kp60[6];
  akp[6] = kp60[7];
  akp[7] = kp60[8];
  akp[8] = kp60[9];

  constexpr std::array<float, 6> alefa = {-0.40F, -0.38F, 0.0F, 0.0F, 0.0F, 0.0F};
  constexpr std::array<int, 6> ma = {1, 4, 16, 28, 32, 14};
  constexpr std::array<float, 6> vma = {1.6606e-24F, 6.6423e-24F, 26.569e-24F,
                                        46.4958e-24F, 53.1381e-24F, 23.2479e-24F};

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

  const float c = std::sin(alat);
  const float c2 = c * c;
  const float c4 = c2 * c2;
  const float s = std::cos(alat);
  const float s2 = s * s;

  LegendreContextHp l;
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

  float gmlat_deg = 0.0F;
  float gmlon_deg = 0.0F;
  Geogm(dlat_deg, dlon_deg, gmlat_deg, gmlon_deg);

  const float gmlat = ToRadians(gmlat_deg);
  const float cm = std::sin(gmlat);
  const float cm2 = cm * cm;
  const float cm4 = cm2 * cm2;
  const float sm = std::cos(gmlat);
  const float sm2 = sm * sm;

  l.p10mg = cm;
  l.p20mg = 1.5F * cm2 - 0.5F;
  l.p30mg = cm * (2.5F * cm2 - 1.5F);
  l.p40mg = 4.375F * cm4 - 3.75F * cm2 + 0.375F;
  l.p50mg = cm * (7.875F * cm4 - 8.75F * cm2 + 1.875F);
  l.p60mg = (5.5F * cm * l.p50mg - 2.5F * l.p40mg) / 3.0F;
  l.p11mg = sm;
  l.p22mg = sm2 * 3.0F;
  l.p31mg = sm * (7.5F * cm2 - 1.5F);

  HarmonicContext hc;
  hc.ch = std::cos(hl);
  hc.sh = std::sin(hl);
  hc.c2h = hc.ch * hc.ch - hc.sh * hc.sh;
  hc.s2h = 2.0F * hc.ch * hc.sh;
  hc.c3h = hc.c2h * hc.ch - hc.s2h * hc.sh;
  hc.s3h = hc.s2h * hc.ch + hc.c2h * hc.sh;

  F1 da{};
  float gdelt = GldtmHp(f, fbar, akp, day, tt, da, 1.0F, xlon, l, hc);
  const float tinf = tt[1] * (1.0F + gdelt);

  da.fill(0.0F);
  float gdelt0 = GldtmHp(f, fbar, akp, day, t0, da, 1.0F, xlon, l, hc);
  const float t120 = t0[1] * (1.0F + gdelt0);

  da.fill(0.0F);
  float gdeltp = GldtmHp(f, fbar, akp, day, tp, da, 1.0F, xlon, l, hc);
  const float tp120 = tp[1] * (1.0F + gdeltp);

  const float sigma = tp120 / (tinf - t120);
  const float dzeta = (re + zlb) / (re + alti);
  const float zeta = (alti - zlb) * dzeta;
  const float sigzeta = sigma * zeta;
  const float expsz = std::exp(-sigzeta);
  const float tz = tinf - (tinf - t120) * expsz;

  std::array<float, 6> dbase = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

  da.fill(0.0F);
  dbase[0] = h[1] * std::exp(GldtmHp(f, fbar, akp, day, h, da, 0.0F, xlon, l, hc));

  da.fill(0.0F);
  dbase[1] = he[1] * std::exp(GldtmHp(f, fbar, akp, day, he, da, 0.0F, xlon, l, hc));

  da.fill(0.0F);
  dbase[2] = o[1] * std::exp(GldtmHp(f, fbar, akp, day, o, da, 1.0F, xlon, l, hc));

  da.fill(0.0F);
  dbase[3] = az2[1] * std::exp(GldtmHp(f, fbar, akp, day, az2, da, 1.0F, xlon, l, hc));

  da.fill(0.0F);
  dbase[4] = o2[1] * std::exp(GldtmHp(f, fbar, akp, day, o2, da, 1.0F, xlon, l, hc));

  da.fill(0.0F);
  dbase[5] = az[1] * std::exp(GldtmHp(f, fbar, akp, day, az, da, 1.0F, xlon, l, hc));

  float glb = gsurf / ((1.0F + zlb / re) * (1.0F + zlb / re));
  glb = glb / (sigma * rgas * tinf);
  const float t120tz = t120 / tz;

  float ro = 0.0F;
  std::array<float, 6> cc = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  std::array<float, 6> d = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

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
        MakeError(ErrorCode::kInvalidInput, "non-finite result generated", {}, "Dtm2020Research::Evaluate"));
  }

  return Result<Outputs, Error>::Ok(out);
}

}  // namespace dtm2020
