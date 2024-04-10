[CmdletBinding()]
param(
    [Parameter(Mandatory=$true)]
    [string]$Entrypoint,
#    [Parameter(Mandatory=$true)]
    [string]$World,
#    [Parameter(Mandatory=$true)]
    [string]$Param,
    [string]$Requirements = "requirements.txt"
)

$env:WORLD = $World
$env:PARAM = $Param

docker-compose up --wait;
start powershell -ArgumentList "-Command docker-compose attach simulation"
docker-compose exec simulation ./vehicle/scripts/sim-run.sh -r $Requirements $Entrypoint