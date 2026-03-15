#!/bin/bash
# Fix the outputLimit bug: uint8_t (0-100) -> float (0.0-1.0)
# Run on Orin: bash fix_output_limit.sh

set -e

MEDULLA=~/Desktop/kart_medulla

echo "=== Fixing outputLimit bug ==="

# 1. Change type from uint8_t to float in header
sed -i 's/uint8_t outputLimit;.*$/float outputLimit;   \/\/ Range of output allow [0.0-1.0]/' \
    "$MEDULLA/components/km_act/km_act.h"
echo "Fixed km_act.h: uint8_t -> float"

# 2. Remove * 100 scaling in KM_ACT_Init
sed -i 's/act\.outputLimit = (uint8_t)(limit_clamp \* 100);/act.outputLimit = limit_clamp;/' \
    "$MEDULLA/components/km_act/km_act.c"
echo "Fixed km_act.c: KM_ACT_Init"

# 3. Remove * 100 scaling in KM_ACT_SetLimit
sed -i 's/act->outputLimit = (uint8_t)(limit_clamp \* 100);/act->outputLimit = limit_clamp;/' \
    "$MEDULLA/components/km_act/km_act.c"
echo "Fixed km_act.c: KM_ACT_SetLimit"

# Verify
echo ""
echo "=== Verification ==="
grep "outputLimit" "$MEDULLA/components/km_act/km_act.h" | head -1
grep "outputLimit = " "$MEDULLA/components/km_act/km_act.c"

echo ""
echo "Done. Build with: cd $MEDULLA && ~/.local/bin/pio run --environment esp32dev"
