param (
    [string]$ContainerName = "drone-container",
    [string]$ArchiveName = "drone-container.tar.xz",
    [string]$BuildDir = "$PSScriptRoot/build",
    [string]$BuildInstructions = "$PSScriptRoot/build-instructions.sh",
    [string]$BaseDownloadURL = "https://cloud-images.ubuntu.com/minimal/releases/focal/release/ubuntu-20.04-minimal-cloudimg-amd64-root.tar.xz"
)

# Ensure slashes are replaced (Windows-style backslashes to Unix-style slashes)
$BuildInstructions = $BuildInstructions -Replace "\\","/"
$BuildDir = $BuildDir -Replace "\\","/"

# Define the tarball paths
$BaseTarball="$BuildDir/$([System.IO.Path]::GetFileName($BaseDownloadURL))"
$BuildTarball="$BuildDir/$ArchiveName"

# Check if Container already exists
if ((wsl -l -q) -Split "\s+" -Contains "$ContainerName")
{
    Write-Warning "Container '$ContainerName' already exists. Continuing will forcefully recreate the container."
    Read-Host -Prompt "Press any key to continue, or CTRL+C to cancel" | Out-Null
    wsl --unregister "$ContainerName"
}

# Create build directory
if (!(Test-Path "$BuildDir"))
{
    New-Item -ItemType Directory "$BuildDir" | Out-Null
}

# Build container from cached images
if (Test-Path "$BuildTarball")
{
    wsl --import "$ContainerName" "$BuildDir" "$BuildTarball"
}
else
{
    if (!(Test-Path "$BaseTarball"))
    {
        Invoke-WebRequest $BaseDownloadURL -OutFile "$BaseTarball"
    }
    wsl --import "$ContainerName" "$BuildDir" "$BaseTarball"
    wsl -d "$ContainerName" -e bash -c ". `$(wslpath -u $BuildInstructions)"
    if ($?)
    {
        wsl --export "$ContainerName" "$BuildTarball"
    }
    else
    {
        wsl --unregister "$ContainerName"
    }
}
