#!/bin/sh
# Output docker container CPU% in InfluxDB line protocol
NOW=$(date +%s)000000000
docker stats --no-stream --format '{{.Name}}\t{{.CPUPerc}}' 2>/dev/null | while IFS=$'\t' read name cpu; do
    short="${name##*/}"
    short="${short##*-}"
    cpu_pct="${cpu%%\%}"
    [ -z "$cpu_pct" ] && continue
    [ "$short" = "NAME" ] && continue
    echo "docker_container_cpu,host=telegraf_host,container_name=${short} usage_percent=${cpu_pct} ${NOW}"
done
