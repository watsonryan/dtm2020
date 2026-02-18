#!/usr/bin/env bash
set -euo pipefail

# Author: Watson
# Purpose: Generate operational golden vectors from the MCM Fortran reference implementation.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REF_FORTRAN_ROOT="${REF_FORTRAN_ROOT:-}"
REF_OPERATIONAL_F90="${REF_OPERATIONAL_F90:-}"
REF_COEFF_FILE="${REF_COEFF_FILE:-}"
OUT_CSV="${ROOT}/testdata/operational_vectors.csv"
TMP_F90="$(mktemp /tmp/dtm2020_vecgen.XXXXXX.f90)"
TMP_EXE="$(mktemp /tmp/dtm2020_vecgen.XXXXXX.exe)"
trap 'rm -f "$TMP_F90" "$TMP_EXE"' EXIT

if [[ -n "${REF_FORTRAN_ROOT}" ]]; then
  if [[ -z "${REF_OPERATIONAL_F90}" ]]; then
    REF_OPERATIONAL_F90="${REF_FORTRAN_ROOT}/src/libswamif/dtm2020_F107_Kp-subr_MCM.f90"
  fi
  if [[ -z "${REF_COEFF_FILE}" ]]; then
    REF_COEFF_FILE="${REF_FORTRAN_ROOT}/data/DTM_2020_F107_Kp.dat"
  fi
fi

if [[ -z "${REF_OPERATIONAL_F90}" || -z "${REF_COEFF_FILE}" ]]; then
  echo "error: set REF_FORTRAN_ROOT or both REF_OPERATIONAL_F90 and REF_COEFF_FILE"
  echo "example: REF_FORTRAN_ROOT=/path/to/reference_fortran tools/generate_operational_vectors_fortran.sh"
  exit 1
fi

cat > "$TMP_F90" <<'F90'
program vecgen
  implicit none
  integer :: i, j, k
  real :: day, alti, hl, alat, xlon, tz, tinf, ro, wmm
  real, dimension(2) :: f, fbar
  real, dimension(4) :: akp
  real, dimension(6) :: d
  real, dimension(4) :: altitudes
  real, dimension(3) :: latitudes
  real, dimension(3) :: ltimes
  real, dimension(3) :: days
  real :: pi

  external :: dtm3, lecdtm

  pi = acos(-1.0)
  altitudes = (/130.0, 200.0, 400.0, 800.0/)
  latitudes = (/-45.0, 0.0, 45.0/)
  ltimes = (/2.0, 12.0, 20.0/)
  days = (/30.0, 180.0, 330.0/)

  f = (/130.0, 0.0/)
  fbar = (/125.0, 0.0/)
  akp = (/2.0, 0.0, 3.0, 0.0/)

  open(unit=42, file='__COEFF__', status='old')
  call lecdtm(42)
  close(42)

  open(unit=43, file='__OUT__', status='replace')
  write(43,'(A)') 'altitude_km,latitude_deg,longitude_deg,local_time_h,day_of_year,f107,f107m,kp_delayed_3h,kp_mean_24h,temperature_k,exospheric_temp_k,density_g_cm3,mean_mol_mass_g,d_h_g_cm3,d_he_g_cm3,d_o_g_cm3,d_n2_g_cm3,d_o2_g_cm3,d_n_g_cm3'

  do i = 1, size(altitudes)
    alti = altitudes(i)
    do j = 1, size(latitudes)
      alat = latitudes(j) * pi / 180.0
      xlon = modulo(30.0 * real(j), 360.0) * pi / 180.0
      do k = 1, size(ltimes)
        hl = ltimes(k) * pi / 12.0
        day = days(k)
        call dtm3(day, f, fbar, akp, alti, hl, alat, xlon, tz, tinf, ro, d, wmm)
        write(43,'(*(g0,:,","))') &
          alti, latitudes(j), modulo(30.0 * real(j), 360.0), ltimes(k), day, f(1), fbar(1), akp(1), akp(3), &
          tz, tinf, ro, wmm, d(1), d(2), d(3), d(4), d(5), d(6)
      end do
    end do
  end do

  close(43)
end program vecgen
F90

python3 - <<'PY' "$TMP_F90" "$REF_COEFF_FILE" "$OUT_CSV"
import pathlib, sys
f90 = pathlib.Path(sys.argv[1])
coeff = sys.argv[2]
out = sys.argv[3]
text = f90.read_text()
text = text.replace("__COEFF__", coeff)
text = text.replace("__OUT__", out)
f90.write_text(text)
PY

gfortran -std=legacy -O0 "$TMP_F90" "$REF_OPERATIONAL_F90" -o "$TMP_EXE"
"$TMP_EXE"

echo "Wrote ${OUT_CSV}"
