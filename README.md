# Directions
## Requirements
- windows 11+ only
- wsl installed (ubuntu image)
- Docker installed
``` 
wsl --install
```
## Docker setup
Setup docker image and container and then detach the process by running:
```
docker-compose up -d
```
You can then enter the docker by entering the following command:
```
docker-compose exec simulation /bin/bash
```
## Webots
Launch webots by the following commands:

with gui (broken?): 
```
webots /path/to/world/file
```
no gui:
```
xvfb-run --auto-servernum webots --mode=fast --no-rendering --stdout --stderr --minimize --batch /path/to/world/file
```
## NOTES
Uses `.gitignore` file to specify what files to not copy into docker. This should probably change.
## ISSUES
 - GUI ISSUE - doesn't display 3D view of webots, probably because of this warning or driver issue.
```
WARNING: System below the minimal requirements.

Webots has detected that your GPU vendor is 'Mesa/X.org'. A recent NVIDIA or AMD graphics adapter is highly recommended to run Webots smoothly. Webots has detected that your computer uses a slow 3D software rendering system. It is strongly recommended to install the latest graphics drivers provided by your GPU manufacturer. Webots will run much faster after the installation of the correct driver.

 - Shadows have been deactivated.
 - Anti-aliasing has been deactivated.
 - Main 3D view global ambient occlusion has been de-activated.
 - Texture quality has been reduced.

You can try to re-activate some OpenGL features from the Webots preferences.
```
Can look into side stepping this issue by fowarding a port (localhost:1234) to the localmachine and hosting a webserver that streams 
the webots view instead by using the --stream option.

You can view the stream via the website `localhost:1234/index.html`

## FIXES
If it doesn't work do
```
docker system prune -a
```
then try building docker again