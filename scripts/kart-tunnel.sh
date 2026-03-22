#!/bin/bash
# Toggle the kart dashboard Cloudflare Tunnel on/off
# Usage: kart-tunnel.sh [on|off|toggle|status]

CONFIG=/etc/cloudflared/config.yml

FULL='tunnel: 6ac3f5b6-9140-47ad-82fb-55b1a6c4afa1
credentials-file: /etc/cloudflared/6ac3f5b6-9140-47ad-82fb-55b1a6c4afa1.json

ingress:
  - hostname: orin.rubenayla.xyz
    service: ssh://localhost:22
  - hostname: kart.rubenayla.xyz
    service: http://localhost:8080
  - service: http_status:404'

SSH_ONLY='tunnel: 6ac3f5b6-9140-47ad-82fb-55b1a6c4afa1
credentials-file: /etc/cloudflared/6ac3f5b6-9140-47ad-82fb-55b1a6c4afa1.json

ingress:
  - hostname: orin.rubenayla.xyz
    service: ssh://localhost:22
  - service: http_status:404'

case "${1:-toggle}" in
  on)    echo "$FULL"     | sudo tee $CONFIG > /dev/null && sudo systemctl restart cloudflared && echo "Dashboard ON  → kart.rubenayla.xyz" ;;
  off)   echo "$SSH_ONLY" | sudo tee $CONFIG > /dev/null && sudo systemctl restart cloudflared && echo "Dashboard OFF → kart.rubenayla.xyz disabled" ;;
  toggle)
    if grep -q kart.rubenayla.xyz $CONFIG; then
      $0 off
    else
      $0 on
    fi ;;
  status)
    if grep -q kart.rubenayla.xyz $CONFIG; then echo "Dashboard: ON"; else echo "Dashboard: OFF"; fi ;;
  *) echo "Usage: kart-tunnel [on|off|toggle|status]" ;;
esac
