#!/bin/sh
docker stats --no-stream --format '{{.Name}} {{.CPUPerc}}' 2>/dev/null | awk '{
    name=$1
    cpu=$2
    gsub("%","",cpu)
    sub(".*_","",name)
    sub("-[0-9]*$","",name)
    if(name!="NAME" && cpu!="") {
        printf "docker_container_cpu,container_name=%s usage_percent=%s\n", name, cpu
    }
}'
