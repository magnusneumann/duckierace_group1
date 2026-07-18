FROM ros:noetic-ros-base

# Installation von Tools und mDNS Support
RUN apt-get update && apt-get install -y \
    python3-opencv \
    python3-pip \
    python3-tk \
    avahi-utils \
    libnss-mdns \
    iputils-ping \
    git \
    build-essential \
    cmake \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip setuptools wheel
RUN pip3 install transitions

# Copy project requirements and install them in the image so the container
# has the same Python environment as listed in `requirements.txt`.
COPY ../requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || \
    (echo "One or more packages from requirements.txt failed to install.\n" \
          "You can inspect /tmp/requirements.txt and install missing system dependencies." && exit 0)

# Fix für mDNS: Erlaubt dem Container, .local Namen aufzulösen
RUN sed -i 's/hosts: files dns/hosts: files mdns4_minimal [NOTFOUND=return] dns mdns4/' /etc/nsswitch.conf

# Automatisches ROS Sourcing für jedes neue Terminal
RUN echo 'export ROS_IP=$(hostname -I | awk "{print \$1}")' >> /root/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/DuckieRace/devel/setup.bash" >> /root/.bashrc