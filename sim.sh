#!/bin/bash
# Parse Arguments
usage() {
    echo "Usage: $0 [OPTIONS] world_file param_file entrypoint"
    echo "Options:"
    echo "  -r <simulation_path>    Specify requirements.txt path (default: ~/vehicle/requirements.txt)"
    echo ""
    echo "Note: Entrypoint and requirements.txt must be relative paths from the root of the project directory"
    exit 1
}

pos_args=()
while [ $OPTIND -le "$#" ]
do
    if getopts S:s:h:p:v:m:x:r: opt
    then
        case "$opt" in
            r) requirements="$OPTARG";;
            *) usage;;
        esac
    else
        pos_args+=("${!OPTIND}")
        shift
    fi
done

export WORLD="${pos_args[0]}"
export PARAM="${pos_args[1]}"
entrypoint="${pos_args[2]}"

docker-compose up --wait

# Check if the platform is Windows
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    mintty winpty bash -lc "docker-compose attach simulation"
else
    x-terminal-emulator -e "docker-compose attach simulation"
fi
docker-compose exec simulation ./vehicle/scripts/sim-run.sh "${requirements:+"-r $requirements "}$entrypoint"