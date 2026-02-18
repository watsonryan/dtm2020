# Golden Test Data

Author: Watson

`operational_vectors.csv` should contain parity vectors generated from the canonical Fortran implementation.

Expected CSV columns:

- altitude_km
- latitude_deg
- longitude_deg
- local_time_h
- day_of_year
- f107
- f107m
- kp_delayed_3h
- kp_mean_24h
- temperature_k
- exospheric_temp_k
- density_g_cm3
- mean_mol_mass_g
- d_h_g_cm3
- d_he_g_cm3
- d_o_g_cm3
- d_n2_g_cm3
- d_o2_g_cm3
- d_n_g_cm3

Tolerance policy is implemented in `tests/operational_golden_test.cpp`.
