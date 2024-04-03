# TODO: add options

docker-compose exec simulation rsync -ar /usr/local/vehicle ~/ --exclude-from=/usr/local/vehicle/.gitignore
docker-compose exec simulation ./vehicle/scripts/sim-run.sh