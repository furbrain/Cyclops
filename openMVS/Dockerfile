# syntax=docker/dockerfile:1
FROM furbrain/openmvs:latest
USER root
COPY make_refined.sh /usr/local/bin
COPY make_rough.sh /usr/local/bin
RUN chmod 755 /usr/local/bin/*sh
USER ubuntu

