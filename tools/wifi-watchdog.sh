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
# Deliberately NOT done: no client attempt at boot. If the Orin powers up with no
# tether it keeps the AP and does not go looking for a network, because in the
# paddock a dashboard at a fixed 10.42.0.1 matters more than remote access.
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
FORCE_FILE=/run/kart-wifi-try-client

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

ap_client_count() {
    iw dev "$WIFI_DEV" station dump 2>/dev/null | grep -c '^Station'
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

log "started (radio=$WIFI_DEV, ap=$AP_CON)"

# Whether the client attempt for the current tether-down period has been used.
# Starts used, so a boot with no tether keeps the AP -- see the header.
client_attempt_spent=1

while true; do
    if [ -e "$FORCE_FILE" ]; then
        rm -f "$FORCE_FILE"
        log "$FORCE_FILE present -- forcing one client attempt"
        try_client
        sleep "$POLL_SECONDS"
        continue
    fi

    con=$(wifi_connection)
    state=$(wifi_state)

    if tether_present; then
        # Tether is back, so the next unplug gets a fresh attempt.
        client_attempt_spent=0
        if [ "$con" != "$AP_CON" ]; then
            log "tether present but radio is on '${con:-nothing}' -- returning to the $AP_CON AP"
            start_ap
        fi
    elif [ "$con" = "$AP_CON" ]; then
        if [ "$client_attempt_spent" -eq 0 ] && [ "$(ap_client_count)" -eq 0 ]; then
            client_attempt_spent=1
            log "USB tether is gone and nobody is associated to the $AP_CON AP"
            try_client
        fi
    elif [ "$state" != connected ]; then
        # Neither serving an AP nor joined to a network: the radio is doing
        # nothing useful and there is no dashboard at all. The AP is the safe
        # default, and this is the original purpose of this watchdog.
        log "radio is '$state' with no connection -- falling back to the $AP_CON AP"
        start_ap
    fi

    sleep "$POLL_SECONDS"
done
