#pass main recroding dir as first argument and either make_rough.sh or make_refined.sh as second arg
docker pull furbrain/cyclops_mvs:latest
docker run --user $(id -u):$(id -g) -w /working/map_1 -v $1:/working -it furbrain/cyclops_mvs:latest $2
