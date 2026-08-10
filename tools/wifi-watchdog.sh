#!/bin/bash
#
# Wi-Fi role watchdog for the kart's Jetson Orin.
#
# The Orin has one Wi-Fi radio (wlP1p1s0, an RTL8822CE) and it cannot be an
# access point and a client at the same time: the driver advertises no "valid
# interface combinations", and a scan issued while the radio is in AP mode
# returns nothing at all. So the radio plays exactly one role at any moment, and
# this script decides which:
#
#   USB tether plugged in  ->  serve the `kart` access point.
#                              Internet already arrives over the tether (an
#                              `enx*` ethernet interface, Orin at 172.20.10.2),
#                              so the radio is free to give the trackside
#                              dashboard a fixed address at http://10.42.0.1/.
#
#   USB tether unplugged   ->  the Orin has no internet at all, which also kills
#                              https://kart.rubenayla.xyz and `ssh orin-remote`
#                              because both ride a Cloudflare tunnel. Give up
#                              the AP and join a known Wi-Fi network to get
#                              internet back. If nothing joins, put the AP back.
#
# Two rules keep this from ever stranding someone standing at the kart:
#
#   1. The AP is never taken down while a device is associated to it. Anyone on
#      the `kart` network is presumably watching the dashboard, and losing it
#      mid-run is worse than having no remote access.
#   2. A client attempt happens only once per tether-unplug event, never on a
#      repeating timer. Every attempt costs a window of up to 30 s with no `kart`
#      network, because the radio has to leave AP mode before it can even scan to
#      find out whether a known network is in range. Plugging the cable back in
#      re-arms the next attempt.
#
# A boot with no working tether DOES spend one client attempt (changed 2026-08-10).
# It used to keep the AP unconditionally, on the argument that a fixed 10.42.0.1 in
# the paddock beats remote access. That cost a real outage: after a battery power
# cycle on 2026-08-08 the Orin came up with no tether, never looked for a network,
# and stayed unreachable until someone plugged a phone in by hand. The unplug path
# had been working correctly the whole time -- a cold boot simply produces no unplug
# event to trigger it. The trade accepted here is that an Orin booted in the lab,
# with Robots_urjc in range and no phone attached, joins it and the dashboard leaves
# 10.42.0.1 for a DHCP address.
#
# ----------------------------------------------------------------------------
# Ranking the uplinks (added 2026-08-10)
#
# More than one iPhone can be tethered at once, and every iOS Personal Hotspot
# hands out the SAME network: 172.20.10.0/28, phone at 172.20.10.1, Orin at
# 172.20.10.2. So with two phones plugged in the Orin holds two interfaces with an
# identical source address and two default routes via an identical gateway. The
# addresses cannot tell the phones apart at all -- only the kernel's `enx<mac>`
# interface name can, which is why everything below keys on the device name and
# every reachability probe is bound to a device with `curl --interface`.
#
# Which phone wins is therefore decided by route metric, lowest first, and the
# ranking is set explicitly in TETHER_METRICS below. Left to itself NetworkManager
# assigns 100 to the first ethernet and 101 to the second, so the winner would be
# whichever phone was plugged in first -- an accident, not a decision.
#
# Metrics alone cannot see a dead uplink, and that is the failure this is really
# for: a phone plugged in with Personal Hotspot switched OFF still gives carrier,
# still completes DHCP and still wins on metric, so traffic goes into a black hole
# while a working phone sits unused at a higher metric. Only an end-to-end probe
# through that specific interface distinguishes "cable connected" from "internet
# works". A tether that fails FAIL_THRESHOLD probes in a row is demoted to
# DEMOTED_METRIC, which leaves the route in place but ranked below everything, so
# the next-best uplink takes over; it is promoted back the moment it answers again.
#
# Demotion is `nmcli connection modify ipv4.route-metric` plus `nmcli device
# reapply`, never `nmcli connection down`. `reapply` changes a live connection
# without deactivating it, so NetworkManager's autoconnect state is untouched. A
# `down` would mark the connection manually deactivated and nothing on the Orin
# would ever bring it back -- the same trap the last-resort block at the bottom of
# this file exists to undo.
#
# Handing the radio over is done with `nmcli connection down kart-ap` rather than
# by naming a network to join. NetworkManager treats a manually deactivated
# connection as ineligible for autoconnect, so once the AP is down it picks the
# highest-priority known network that is actually in range, by itself. Bringing
# the AP back up with `nmcli connection up kart-ap` clears that state.
#
# To force one client attempt without unplugging anything, useful for testing:
#     sudo touch /run/kart-wifi-try-client
#
# Progress goes to the journal: journalctl -t wifi-watchdog
#
# Source of truth for this file is the kart-brain repo, tools/wifi-watchdog.sh.
# Installed to /usr/local/bin/wifi-watchdog.sh and run by wifi-watchdog.service.

set -u

WIFI_DEV=wlP1p1s0
AP_CON=kart-ap
POLL_SECONDS=5
CLIENT_WAIT_SECONDS=30
INTERNET_GRACE_SECONDS=120
FORCE_FILE=/run/kart-wifi-try-client

# Preferred order of the USB tethers, lowest metric first. The keys are kernel
# interface names, derived from each phone's USB-ethernet MAC and therefore stable
# per phone. A phone not listed here gets TETHER_METRIC_UNKNOWN, which ranks below
# both known phones but still above Wi-Fi, so a visitor's phone is used when it is
# the only thing plugged in and never steals the route from a known one.
declare -A TETHER_METRICS=(
    [enxfe9ca7a9ecdb]=100   # Ruben's iPhone
    [enx7e4b26d3e33f]=110   # Jorge's iPhone
)
TETHER_METRIC_UNKNOWN=150

# Where a tether goes when its probes fail. Far above the Wi-Fi profiles (600-700)
# so a demoted tether loses to absolutely everything, while its route stays in the
# table ready to be promoted back.
DEMOTED_METRIC=900

# Consecutive failed probes before demoting. One missed probe on a cellular link is
# ordinary; demoting on a single failure would flap the default route.
FAIL_THRESHOLD=3

# A working uplink is re-probed every poll. A demoted one is probed less often --
# it is not carrying traffic, and each probe costs PROBE_TIMEOUT of waiting.
DEMOTED_PROBE_EVERY=6
PROBE_TIMEOUT=5
PROBE_URL=http://connectivitycheck.gstatic.com/generate_204

log() { logger -t wifi-watchdog "$*"; }

# Name of the connection currently active on the Wi-Fi radio; empty if none.
wifi_connection() {
    nmcli -t -f DEVICE,CONNECTION device status \
        | awk -F: -v d="$WIFI_DEV" '$1 == d { print $2 }'
}

wifi_state() {
    nmcli -t -f DEVICE,STATE device status \
        | awk -F: -v d="$WIFI_DEV" '$1 == d { print $2 }'
}

# The iPhone USB tether appears as an `enx<mac>` ethernet device, named after the
# phone's MAC, so it changes if the phone presents a different MAC. `eno1` is the
# on-board wired port and is deliberately not counted as the tether.
tether_present() {
    nmcli -t -f DEVICE,TYPE,STATE device status \
        | grep -qE '^enx[^:]*:ethernet:connected$'
}

# Every connected tether interface, one per line. Plural because two phones can be
# plugged in at once.
tether_devices() {
    nmcli -t -f DEVICE,TYPE,STATE device status \
        | awk -F: '$2 == "ethernet" && $3 == "connected" && $1 ~ /^enx/ { print $1 }'
}

# The NetworkManager connection currently active on a device; empty if none.
conn_for_dev() {
    nmcli -t -f DEVICE,CONNECTION device status \
        | awk -F: -v d="$1" '$1 == d { print $2 }'
}

# The metric this tether should hold when healthy.
base_metric_for() {
    echo "${TETHER_METRICS[$1]:-$TETHER_METRIC_UNKNOWN}"
}

# Does this specific interface actually reach the internet? Bound to the device
# with --interface rather than to an address, because both phones present the very
# same 172.20.10.2 and an address-bound probe could not say which one it tested.
uplink_ok() {
    local code
    code=$(curl -s --interface "$1" --max-time "$PROBE_TIMEOUT" \
        -o /dev/null -w '%{http_code}' "$PROBE_URL" 2>/dev/null)
    [ "$code" = 204 ]
}

# The metric the kernel is actually using for this device's default route. This is
# the only thing that decides which phone carries traffic -- what NetworkManager has
# stored in the profile is an intention, not a fact.
kernel_metric() {
    ip -4 route show default dev "$1" 2>/dev/null \
        | sed -n 's/.*metric \([0-9]*\).*/\1/p' | head -1
}

# Apply a route metric to a live connection without deactivating it. `reapply` is
# the whole point: `connection up` would bounce the interface and drop everything
# running over it, and `connection down` would mark the profile manually
# deactivated so NetworkManager never autoconnects it again.
#
# Verify against the kernel and retry once. Measured on the Orin 2026-08-10: a
# `modify` immediately followed by one `reapply` printed "Connection successfully
# reapplied" while the route stayed at its old metric, and a second `reapply` moved
# it. So a single successful-looking call is not evidence the route moved, and a
# silent failure here would leave a dead phone still carrying the default route --
# exactly the failure this function exists to prevent.
set_metric() {
    local dev=$1 metric=$2 con current try
    con=$(conn_for_dev "$dev")
    [ -z "$con" ] && return 1

    current=$(nmcli -g ipv4.route-metric connection show "$con" 2>/dev/null)
    if [ "$current" != "$metric" ]; then
        nmcli connection modify "$con" ipv4.route-metric "$metric" >/dev/null 2>&1 || return 1
    fi

    for try in 1 2 3; do
        [ "$(kernel_metric "$dev")" = "$metric" ] && return 0
        nmcli device reapply "$dev" >/dev/null 2>&1
        sleep 1
    done

    if [ "$(kernel_metric "$dev")" != "$metric" ]; then
        log "WARNING: $dev route metric is still '$(kernel_metric "$dev")' after 3 reapply attempts, wanted $metric"
        return 1
    fi
}

ap_client_count() {
    iw dev "$WIFI_DEV" station dump 2>/dev/null | grep -c '^Station'
}

have_internet() {
    [ "$(nmcli -t -f CONNECTIVITY general 2>/dev/null)" = full ]
}

# Name of the USB tether's ethernet device if the hardware is plugged in at all,
# whether or not a connection is currently active on it. Empty if unplugged.
tether_device() {
    nmcli -t -f DEVICE,TYPE device status \
        | awk -F: '$2 == "ethernet" && $1 ~ /^enx/ { print $1; exit }'
}

start_ap() {
    nmcli --wait 25 connection up "$AP_CON" ifname "$WIFI_DEV" >/dev/null 2>&1
}

# Release the AP and let NetworkManager find a known network. Returns 0 if a
# client connection came up, 1 if the AP had to be restored.
try_client() {
    log "releasing the $AP_CON AP to look for a known network"
    nmcli connection down "$AP_CON" >/dev/null 2>&1

    local waited=0 con state
    while [ "$waited" -lt "$CLIENT_WAIT_SECONDS" ]; do
        sleep 2
        waited=$((waited + 2))
        con=$(wifi_connection)
        state=$(wifi_state)
        if [ "$state" = connected ] && [ -n "$con" ] && [ "$con" != "$AP_CON" ]; then
            log "joined '$con' -- internet is back; the $AP_CON AP stays down until the tether returns"
            return 0
        fi
    done

    log "no known network joined within ${CLIENT_WAIT_SECONDS}s -- restoring the $AP_CON AP"
    start_ap
    return 1
}

# Consecutive probe failures per tether, and which tethers are currently demoted.
declare -A fail_count=()
declare -A demoted=()
poll_n=0

# Number of tethers that answered their probe on the last pass. Set by rank_tethers.
WORKING_TETHERS=0

# Probe every connected tether, demote the dead ones, promote the recovered ones,
# and keep the healthy ones pinned at their configured metric.
rank_tethers() {
    local dev base n_ok=0
    for dev in $(tether_devices); do
        base=$(base_metric_for "$dev")

        if [ "${demoted[$dev]:-0}" -eq 1 ]; then
            # A demoted tether carries no traffic, so probing it every poll only
            # spends PROBE_TIMEOUT waiting. Check it occasionally instead -- but do
            # check it, or the first demotion would be permanent in practice and
            # switching the hotspot back on would never be noticed.
            [ $((poll_n % DEMOTED_PROBE_EVERY)) -ne 0 ] && continue
            if uplink_ok "$dev"; then
                demoted[$dev]=0
                fail_count[$dev]=0
                set_metric "$dev" "$base"
                log "$dev answers again -- promoted back to metric $base"
                n_ok=$((n_ok + 1))
            fi
            continue
        fi

        if uplink_ok "$dev"; then
            fail_count[$dev]=0
            set_metric "$dev" "$base"
            n_ok=$((n_ok + 1))
        else
            fail_count[$dev]=$(( ${fail_count[$dev]:-0} + 1 ))
            if [ "${fail_count[$dev]}" -ge "$FAIL_THRESHOLD" ]; then
                demoted[$dev]=1
                set_metric "$dev" "$DEMOTED_METRIC"
                log "$dev failed $FAIL_THRESHOLD probes -- demoted to metric $DEMOTED_METRIC (plugged in, but no internet through it)"
            fi
        fi
    done
    WORKING_TETHERS=$n_ok
}

log "started (radio=$WIFI_DEV, ap=$AP_CON)"

# Whether the client attempt for the current tether-down period has been used.
# Starts unused, so a boot that finds no working tether spends one attempt looking
# for a known Wi-Fi network -- see the cold-boot note in the header.
client_attempt_spent=0

# When the Orin last had working internet, used by the last-resort block below.
no_internet_since=0

while true; do
    poll_n=$((poll_n + 1))
    rank_tethers

    if [ -e "$FORCE_FILE" ]; then
        rm -f "$FORCE_FILE"
        log "$FORCE_FILE present -- forcing one client attempt"
        try_client
        sleep "$POLL_SECONDS"
        continue
    fi

    con=$(wifi_connection)
    state=$(wifi_state)

    # A tether that is plugged in but carrying nothing does not count here. The old
    # test was tether_present, i.e. "is a cable connected", which kept the radio on
    # the AP while a phone with its hotspot switched off provided no internet at all.
    if [ "$WORKING_TETHERS" -gt 0 ]; then
        # Internet is arriving over USB, so the next loss gets a fresh attempt.
        client_attempt_spent=0
        if [ "$con" != "$AP_CON" ]; then
            log "working tether present but radio is on '${con:-nothing}' -- returning to the $AP_CON AP"
            start_ap
        fi
    elif [ "$con" = "$AP_CON" ]; then
        if [ "$client_attempt_spent" -eq 0 ] && [ "$(ap_client_count)" -eq 0 ]; then
            client_attempt_spent=1
            log "no tether is providing internet and nobody is associated to the $AP_CON AP"
            try_client
        fi
    elif [ "$state" != connected ]; then
        # Neither serving an AP nor joined to a network: the radio is doing
        # nothing useful and there is no dashboard at all. The AP is the safe
        # default, and this is the original purpose of this watchdog.
        log "radio is '$state' with no connection -- falling back to the $AP_CON AP"
        start_ap
    fi

    # Last resort: never sit with no internet at all while the tether hardware is
    # plugged in. `nmcli connection down` on the tether marks it manually deactivated
    # and NetworkManager will not autoconnect it again, and nothing else on the Orin
    # brings it back -- so one `down`, deliberate or accidental, would cut every
    # remote route until somebody could be physically at the kart. The grace period
    # matters: during a genuine fallback test the Wi-Fi does provide internet, so this
    # block sees connectivity and leaves the tether alone rather than fighting the test.
    if have_internet; then
        no_internet_since=0
    else
        [ "$no_internet_since" -eq 0 ] && no_internet_since=$SECONDS
        if [ $((SECONDS - no_internet_since)) -ge "$INTERNET_GRACE_SECONDS" ]; then
            dev=$(tether_device)
            if [ -n "$dev" ]; then
                log "no internet for ${INTERNET_GRACE_SECONDS}s and tether hardware is present on $dev -- bringing it back up"
                nmcli device connect "$dev" >/dev/null 2>&1
            fi
            no_internet_since=$SECONDS
        fi
    fi

    sleep "$POLL_SECONDS"
done
