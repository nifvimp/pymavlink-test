#!/bin/bash
# Parse Arguments
usage() {
    echo "Usage: $0 [OPTIONS] entrypoint"
    echo "Options:"
    echo "  -r <simulation_path>    Specify requirements.txt path (default: ~/vehicle/requirements.txt)"
    echo ""
    echo "Note: Entrypoint and requirements.txt must be relative paths from the root of the project directory"
    exit 1
}

pos_args=()
while [ $OPTIND -le "$#" ]
do
    if getopts r: opt
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

if [ ${#pos_args[@]} -ne 1 ]; then
    usage
else
    entrypoint="${pos_args[0]}"
fi

# Set Defaults
requirements="$HOME/vehicle/${requirements:-"requirements.txt"}"
entrypoint="$HOME/vehicle/${entrypoint:-"main.py"}"

# Execute
pip install -r $requirements
python3 "$entrypoint"