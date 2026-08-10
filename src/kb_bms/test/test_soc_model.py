"""Tests for the pack charge model in kb_bms/soc_model.py.

Pure Python — no ROS, no Bluetooth — so these run on a laptop as well as the Orin.

These check the filter's BEHAVIOUR (does it discount voltage under load, does it
recover from a drifted starting point, does it follow the coulomb count when the
voltage says nothing useful) rather than exact numbers, because the numbers depend
on OCV_TABLE and CELL_RESISTANCE_OHM, which are still provisional generic figures
and will change when the pack is measured. A test pinned to today's constants would
fail on that calibration and say nothing about whether the filter still works.
"""

import pytest

from kb_bms.soc_model import (
    PACK_RESISTANCE_OHM,
    SERIES_CELLS,
    SocFilter,
    ocv_to_soc,
    open_circuit_cell_voltage,
    soc_to_ocv_slope,
)


def pack_voltage_for(soc, current=0.0):
    """Terminal voltage a pack at this charge would show at this current."""
    from kb_bms.soc_model import OCV_TABLE

    charges = [s for s, _ in OCV_TABLE]
    volts = [v for _, v in OCV_TABLE]
    for i in range(1, len(charges)):
        if soc <= charges[i]:
            frac = (soc - charges[i - 1]) / (charges[i] - charges[i - 1])
            cell = volts[i - 1] + frac * (volts[i] - volts[i - 1])
            break
    else:
        cell = volts[-1]
    return cell * SERIES_CELLS + current * PACK_RESISTANCE_OHM


class TestOcvCurve:
    def test_monotonic(self):
        """Charge must rise with voltage, or the inverse lookup is meaningless."""
        previous = -1.0
        for volts in [3.1, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9, 4.0, 4.1, 4.15]:
            soc = ocv_to_soc(volts)
            assert soc > previous
            previous = soc

    def test_clamps_outside_the_table(self):
        """Beyond either end the curve turns sharply, so extrapolating linearly would
        be badly wrong. Clamping is the deliberate choice."""
        assert ocv_to_soc(2.0) == 0.0
        assert ocv_to_soc(5.0) == 1.0

    def test_the_observed_drift_case(self):
        """The reading that prompted this module: 46.4 V at rest on 13 cells is
        3.57 V/cell, which must come out far below the 93% the BMS claimed."""
        assert ocv_to_soc(46.4 / SERIES_CELLS) < 0.45

    def test_plateau_is_flatter_than_the_ends(self):
        """This is the property the filter's noise scheduling depends on: a voltage
        reading mid-pack carries much less charge information than one near empty."""
        assert soc_to_ocv_slope(0.5) < soc_to_ocv_slope(0.07)


class TestSagCompensation:
    def test_discharge_reads_higher_than_the_terminals(self):
        """Under discharge the terminals sit below open circuit, so the corrected
        figure must be higher than the raw one."""
        raw = 46.4 / SERIES_CELLS
        corrected = open_circuit_cell_voltage(46.4, -48.0)
        assert corrected > raw

    def test_charging_reads_lower_than_the_terminals(self):
        assert open_circuit_cell_voltage(53.0, +20.0) < 53.0 / SERIES_CELLS

    def test_at_rest_it_changes_nothing(self):
        assert open_circuit_cell_voltage(50.0, 0.0) == pytest.approx(
            50.0 / SERIES_CELLS
        )


class TestSocFilter:
    def test_seeds_from_voltage_when_resting(self):
        """The boot case. A stationary kart must start from its own voltage rather
        than inherit whatever the BMS's drifted counter claims."""
        f = SocFilter(capacity_ah=16.8)
        soc = f.update(
            pack_voltage=46.4, current=0.0, remain_ah=15.6, bms_soc=0.93, dt=0.0
        )
        assert f.seeded_from_voltage
        assert soc < 0.45  # not the 0.93 the BMS asserted

    def test_seeds_from_the_bms_when_under_load(self):
        """A pack already being driven has no trustworthy voltage to seed from, so
        the BMS figure is the only option — accepted, but with wide uncertainty."""
        f = SocFilter(capacity_ah=16.8)
        soc = f.update(
            pack_voltage=44.0, current=-45.0, remain_ah=15.6, bms_soc=0.93, dt=0.0
        )
        assert not f.seeded_from_voltage
        assert soc == pytest.approx(0.93)
        assert f.variance > 0.01

    def test_a_drifted_start_converges_while_resting(self):
        """Seeded wrongly under load, then parked: the voltage correction must pull
        the estimate down to the truth instead of holding the inherited error."""
        f = SocFilter(capacity_ah=16.8)
        f.update(pack_voltage=44.0, current=-45.0, remain_ah=15.6, bms_soc=0.93, dt=0.0)
        truth = 0.25
        resting_v = pack_voltage_for(truth)
        for _ in range(60):
            f.update(
                pack_voltage=resting_v,
                current=0.0,
                remain_ah=15.6,
                bms_soc=0.93,
                dt=2.2,
            )
        assert f.soc == pytest.approx(truth, abs=0.05)

    def test_heavy_load_does_not_yank_the_estimate(self):
        """The point of scaling the noise by current. A settled estimate must survive
        a burst of hard driving, whose sagging terminals would otherwise read as a
        near-empty pack."""
        f = SocFilter(capacity_ah=16.8)
        truth = 0.60
        f.update(
            pack_voltage=pack_voltage_for(truth),
            current=0.0,
            remain_ah=10.1,
            bms_soc=0.60,
            dt=0.0,
        )
        settled = f.soc
        sagging = pack_voltage_for(truth) - 3.0
        for _ in range(10):
            f.update(
                pack_voltage=sagging,
                current=-48.0,
                remain_ah=10.1,
                bms_soc=0.60,
                dt=2.2,
            )
        assert f.soc == pytest.approx(settled, abs=0.05)

    def test_follows_the_coulomb_count_across_the_plateau(self):
        """Mid-pack the voltage says almost nothing, so the estimate must track the
        BMS's remaining-Ah change rather than sitting still."""
        f = SocFilter(capacity_ah=16.8)
        f.update(
            pack_voltage=pack_voltage_for(0.60),
            current=0.0,
            remain_ah=10.1,
            bms_soc=0.60,
            dt=0.0,
        )
        start = f.soc
        # Two amp-hours drawn out, reported by the BMS, while driving hard enough
        # that the voltage carries no usable information.
        f.update(
            pack_voltage=pack_voltage_for(0.60) - 3.0,
            current=-48.0,
            remain_ah=8.1,
            bms_soc=0.48,
            dt=2.2,
        )
        assert f.soc < start - 0.08

    def test_stays_within_bounds(self):
        """A charge outside 0..1 is meaningless and would render as a broken dial."""
        f = SocFilter(capacity_ah=16.8)
        f.update(pack_voltage=54.6, current=0.0, remain_ah=16.8, bms_soc=1.0, dt=0.0)
        for _ in range(20):
            f.update(
                pack_voltage=54.6, current=0.0, remain_ah=20.0, bms_soc=1.0, dt=2.2
            )
        assert 0.0 <= f.soc <= 1.0

    def test_no_capacity_means_no_estimate(self):
        """Guards the division that turns amp-hours into a fraction."""
        f = SocFilter(capacity_ah=0.0)
        f.update(pack_voltage=46.4, current=0.0, remain_ah=15.6, bms_soc=0.93, dt=0.0)
        f.update(pack_voltage=46.4, current=0.0, remain_ah=15.0, bms_soc=0.93, dt=2.2)
        assert 0.0 <= f.soc <= 1.0
