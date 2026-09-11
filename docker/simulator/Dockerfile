# This is just a bare bones docker file to allow us to build something that stays on the system
# permanently, rather than the previous system where the docker image was pulled from the internet
# every time.

# Start from the base image created by Murillo
FROM murilomarinho/coppeliasim:latest

# Use bash as the default shell
SHELL ["/bin/bash", "-c"]

# Make sure all the packages are up to date
# RUN apt-get update && apt-get upgrade && apt-get install -y
RUN apt-get update && apt-get upgrade -y
