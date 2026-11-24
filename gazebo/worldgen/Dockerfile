# Runtime for the `worldgen` compose service: Python plus the generator's
# dependencies. The scripts themselves (gazebo/make_gz_world.py and
# gazebo/worldgen/prepare_site.py) are bind-mounted by compose, so editing them
# needs no rebuild.
FROM python:3.12-slim

RUN pip install --no-cache-dir "requests>=2.31" "pillow>=10.0" "numpy>=1.26"

WORKDIR /app
ENV PYTHONUNBUFFERED=1
CMD ["python3", "/app/prepare_site.py"]
