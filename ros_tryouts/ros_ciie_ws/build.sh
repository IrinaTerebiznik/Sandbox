#!/bin/bash
set -e

IMAGE_NAME="ros-ciie"

echo ">>> Construyendo imagen Docker: $IMAGE_NAME"
docker build -t $IMAGE_NAME .

echo ">>> Imagen construida con éxito."
echo
echo ">>> Para correr el contenedor:"
echo "1. Copiá y renombra config/example.env a config/.env y editá el puerto USB"
echo "2. Ejecutá: docker run --env-file config/.env --device=\$USB_PORT -it --rm $IMAGE_NAME"
