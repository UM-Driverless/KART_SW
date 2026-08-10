"""State-of-charge model for the kart's 13S4P Molicel P42A pack.

Why this exists
---------------
The JBD/Xiaoxiang smart BMS reports a state-of-charge byte that is an open-loop
coulomb count. It only re-zeros when the pack hits a full-charge termination or a
low-voltage cutoff; between those two events nothing corrects it, so its error
accumulates. On 2026-08-10 it reported 93% while the pack sat at 46.4 V, which on
13 series cells is 3.57 V/cell — roughly a quarter charged.

Voltage on its own is not the fix. This chemistry's open-circuit curve is flat
between about 3.6 and 3.9 V/cell, so in the middle of the range a small voltage
error becomes a large charge error. Under load it is worse: the pack is about
65 mOhm, so drawing the motor's 48 A peak sags the terminals by roughly 3 V,
which is the entire flat region.

So this module fuses the two. A scalar Kalman filter carries the charge estimate:

  * The prediction step advances it by the change in the BMS's own remaining-Ah
    figure. That is the BMS's internal high-rate integration, which is accurate
    in the short term — its fault is an accumulated offset, not its rate. Note
    that kart-brain deliberately does NOT integrate current itself: the BLE poll
    loop completes about every 2.2 s, far too slow to integrate the spiky current
    a throttle produces without aliasing badly.
  * The update step corrects with a charge figure read off the open-circuit-voltage
    curve, after compensating the terminal voltage for resistive sag.
  * The update's noise is scaled by the local slope of that curve and by the
    current being drawn. This is what makes the filter behave sensibly without any
    hand-tuned mode switch: on the flat plateau, and under heavy load, the implied
    noise is enormous and the voltage reading is effectively ignored, so the
    estimate coasts on the coulomb count. At rest, and near either end of the
    curve where it steepens, the noise collapses and voltage pulls the estimate
    back onto the truth.

Calibration status
------------------
OCV_TABLE and CELL_RESISTANCE_OHM below are PROVISIONAL generic NMC 21700 figures,
not measurements of this pack. Until they are replaced with measured values, the
fusion is off by default (see the `soc_fusion` parameter on the kb_bms node) and
the dashboard keeps showing the raw BMS figure.

To measure both, run `log_battery` (in this package) across one full discharge and
recharge. It records voltage, current and remaining Ah from /battery/state. The
curve comes from the voltage-against-charge trace at low current; the resistance
comes from dV/dI across consecutive samples where the current stepped.
"""

from __future__ import annotations

import bisect

# Pack geometry — 13 cells in series, 4 in parallel, Molicel P42A 21700.
SERIES_CELLS = 13
PARALLEL_CELLS = 4

# PROVISIONAL. Generic NMC 21700 open-circuit voltage per cell against fractional
# charge, low rate, ordered by increasing charge so it can be interpolated
# directly. Replace with a measured curve for this pack — see the module docstring.
OCV_TABLE: list[tuple[float, float]] = [
    (0.00, 3.00),
    (0.05, 3.36),
    (0.10, 3.45),
    (0.20, 3.55),
    (0.30, 3.61),
    (0.40, 3.66),
    (0.50, 3.72),
    (0.60, 3.79),
    (0.70, 3.87),
    (0.80, 3.96),
    (0.90, 4.06),
    (0.95, 4.11),
    (1.00, 4.18),
]

# PROVISIONAL. Molicel P42A DC internal resistance, per cell, in ohms.
CELL_RESISTANCE_OHM = 0.020

# Series-parallel arrangement: resistance multiplies with series count and divides
# with parallel count.
PACK_RESISTANCE_OHM = CELL_RESISTANCE_OHM * SERIES_CELLS / PARALLEL_CELLS

# Assumed fractional error on CELL_RESISTANCE_OHM. It sets how fast the filter
# stops believing the voltage as current rises, so an honest over-estimate here is
# much safer than an optimistic one.
RESISTANCE_UNCERTAINTY = 0.4

# Per-cell voltage measurement noise, in volts, at zero current. Covers the BMS's
# own quantisation (10 mV) plus cell-to-cell spread across the series string.
VOLTAGE_NOISE_V = 0.012

# Process noise: how much charge estimate uncertainty accrues per second of running
# on the coulomb count alone, as a fraction of full charge. Sized so that an hour of
# driving with no usable voltage correction widens the estimate by roughly 5 points,
# which is about what a few percent of capacity error would do.
PROCESS_NOISE_PER_SECOND = (0.05 ** 2) / 3600.0

# Charge estimate variance to start from when the only thing available to seed with
# is the BMS's own figure — deliberately wide, because that figure is the thing
# this module exists to distrust.
INITIAL_VARIANCE_FROM_BMS = 0.30 ** 2

# Below this current magnitude, in amps, the pack counts as resting: sag is small
# enough that the voltage reading is worth seeding a cold estimate from.
REST_CURRENT_A = 1.0


def ocv_to_soc(cell_ocv: float) -> float:
    """@brief Fractional charge implied by an open-circuit voltage, per cell.

    Linearly interpolates OCV_TABLE. Voltages outside the table clamp to its ends
    rather than extrapolating, since the curve turns sharply there and a linear
    guess beyond the last point would be badly wrong.

    @param cell_ocv Open-circuit voltage of a single cell, in volts.
    @return Fractional charge in [0, 1].
    """
    if cell_ocv <= OCV_TABLE[0][1]:
        return OCV_TABLE[0][0]
    if cell_ocv >= OCV_TABLE[-1][1]:
        return OCV_TABLE[-1][0]
    voltages = [v for _, v in OCV_TABLE]
    i = bisect.bisect_left(voltages, cell_ocv)
    soc_lo, v_lo = OCV_TABLE[i - 1]
    soc_hi, v_hi = OCV_TABLE[i]
    return soc_lo + (cell_ocv - v_lo) * (soc_hi - soc_lo) / (v_hi - v_lo)


def soc_to_ocv_slope(soc: float) -> float:
    """@brief Local steepness of the OCV curve, in volts per cell per unit charge.

    This is what tells the filter how informative a voltage reading is. On the flat
    plateau it is small, so a given voltage error implies a huge charge error and
    the reading gets discounted; near either end it is large and the reading is
    trusted.

    @param soc Fractional charge to evaluate the slope at.
    @return dV/dSOC at that charge, in volts per cell. Always positive.
    """
    soc = min(max(soc, 0.0), 1.0)
    charges = [s for s, _ in OCV_TABLE]
    i = bisect.bisect_left(charges, soc)
    i = min(max(i, 1), len(OCV_TABLE) - 1)
    soc_lo, v_lo = OCV_TABLE[i - 1]
    soc_hi, v_hi = OCV_TABLE[i]
    return (v_hi - v_lo) / (soc_hi - soc_lo)


def open_circuit_cell_voltage(pack_voltage: float, current: float) -> float:
    """@brief Undo resistive sag to recover the per-cell open-circuit voltage.

    @param pack_voltage Terminal voltage of the whole pack, in volts.
    @param current Pack current in amps, JBD sign convention: positive is charge
           into the pack, negative is discharge out of it. Charging lifts the
           terminals above open circuit and discharging pulls them below, so the
           correction subtracts the I*R product in both directions.
    @return Estimated open-circuit voltage of one cell, in volts.
    """
    return (pack_voltage - current * PACK_RESISTANCE_OHM) / SERIES_CELLS


class SocFilter:
    """@brief Scalar Kalman filter fusing the BMS coulomb count with pack voltage.

    Call `update` once per BMS reading. The estimate is available afterwards as
    `soc` (fractional charge) with `variance` as its uncertainty.
    """

    def __init__(self, capacity_ah: float):
        """@param capacity_ah Full charge of the pack in amp-hours, used to convert
        the BMS's remaining-Ah readings into fractional charge."""
        self.capacity_ah = capacity_ah
        self.soc: float | None = None
        self.variance = INITIAL_VARIANCE_FROM_BMS
        self._last_remain_ah: float | None = None
        self.seeded_from_voltage = False

    def _seed(self, pack_voltage: float, current: float, bms_soc: float) -> None:
        """Establish a starting estimate.

        A resting pack is seeded from its voltage, which is the whole point of doing
        this at boot: the kart is nearly always stationary when the node starts, so
        the very first estimate can be an honest one rather than an inherited drift.
        A pack already under load has no trustworthy voltage to seed from, so it
        falls back to the BMS figure with wide uncertainty and lets later resting
        moments correct it.
        """
        if abs(current) <= REST_CURRENT_A:
            cell_ocv = open_circuit_cell_voltage(pack_voltage, current)
            self.soc = ocv_to_soc(cell_ocv)
            slope = soc_to_ocv_slope(self.soc)
            self.variance = (VOLTAGE_NOISE_V / slope) ** 2
            self.seeded_from_voltage = True
        else:
            self.soc = bms_soc
            self.variance = INITIAL_VARIANCE_FROM_BMS
            self.seeded_from_voltage = False

    def _measurement_variance(self, current: float, soc: float) -> float:
        """Uncertainty of the charge figure implied by this voltage reading.

        Two things spoil it, and both are folded in here. Sag: the compensation uses
        an assumed resistance, so its error grows with current. Curve shape: a
        voltage error on the flat plateau implies a far larger charge error than the
        same error near the ends. Dividing the voltage uncertainty by the local
        slope converts one into the other.
        """
        sag_uncertainty_v = (
            abs(current) * CELL_RESISTANCE_OHM / PARALLEL_CELLS * RESISTANCE_UNCERTAINTY
        )
        voltage_sigma = VOLTAGE_NOISE_V + sag_uncertainty_v
        slope = soc_to_ocv_slope(soc)
        return (voltage_sigma / slope) ** 2

    def update(
        self,
        pack_voltage: float,
        current: float,
        remain_ah: float,
        bms_soc: float,
        dt: float,
    ) -> float:
        """@brief Fold one BMS reading into the estimate.

        @param pack_voltage Terminal voltage of the pack, in volts.
        @param current Pack current in amps, positive into the pack.
        @param remain_ah The BMS's remaining charge in amp-hours. Only its change
               between calls is used, never its absolute value, because the absolute
               value carries the same drift as the SOC byte.
        @param bms_soc The BMS's own fractional charge, used only to seed a cold
               estimate on a pack that is not at rest.
        @param dt Seconds since the previous call, for accruing process noise.
        @return The fused fractional charge, in [0, 1].
        """
        if self.soc is None:
            self._seed(pack_voltage, current, bms_soc)
            self._last_remain_ah = remain_ah
            return self.soc

        # Predict: advance by the BMS's own coulomb count, which is trustworthy as a
        # relative motion even when its absolute value is not.
        if self._last_remain_ah is not None and self.capacity_ah > 0:
            self.soc += (remain_ah - self._last_remain_ah) / self.capacity_ah
        self._last_remain_ah = remain_ah
        self.variance += PROCESS_NOISE_PER_SECOND * max(dt, 0.0)

        # Update against the voltage-implied charge.
        measured = ocv_to_soc(open_circuit_cell_voltage(pack_voltage, current))
        r = self._measurement_variance(current, self.soc)
        gain = self.variance / (self.variance + r)
        self.soc += gain * (measured - self.soc)
        self.variance *= 1.0 - gain

        self.soc = min(max(self.soc, 0.0), 1.0)
        return self.soc
