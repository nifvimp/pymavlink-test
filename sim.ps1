[CmdletBinding()]
param(
    # [Parameter(Mandatory=$true)]
    [string]$World,
#    [Parameter(Mandatory=$true)]
    [string]$Param,
    [Parameter(Mandatory=$true)]
    [string]$Entrypoint,
    [string]$Requirements = "requirements.txt"
)

$env:WORLD = $World
$env:PARAM = $Param

docker-compose up --wait;
start powershell { docker-compose attach simulation }
docker-compose exec simulation ./vehicle/scripts/sim-run.sh -r $Requirements $Entrypoint