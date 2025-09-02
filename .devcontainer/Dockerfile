# Official ROS 2 Jazzy Development Container
FROM osrf/ros:jazzy-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace

# Install system dependencies
RUN apt-get update && apt-get install -y \
    # Development tools
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    nano \
    htop \
    tree \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    # Python development
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    # ROS 2 additional packages
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-lifecycle-manager \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    # GUI support
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # VNC and noVNC for GUI applications
    tigervnc-standalone-server \
    tigervnc-common \
    fluxbox \
    wget \
    # Additional utilities
    bash-completion \
    command-not-found \
    iputils-ping \
    net-tools \
    ssh \
    gosu \
    && rm -rf /var/lib/apt/lists/*

# Install uv (fast Python package manager)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.cargo/bin:$PATH"

# Install Dagger CLI for CI/CD
RUN curl -L https://dl.dagger.io/dagger/install.sh | sh && \
    mv bin/dagger /usr/local/bin/

# Create Python virtual environment with uv
RUN uv venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Install Python packages with uv
RUN uv pip install \
    # Development tools
    ruff \
    black \
    mypy \
    pytest \
    pytest-cov \
    pre-commit \
    # ROS 2 Python packages
    rosdep \
    colcon-common-extensions \
    vcstool \
    # Scientific computing
    numpy \
    scipy \
    matplotlib \
    # Utilities
    click \
    pyyaml \
    jinja2 \
    # CI/CD
    dagger-io

# Setup noVNC for web-based GUI access
RUN mkdir -p /opt/noVNC/utils/websockify && \
    wget -qO- https://github.com/novnc/noVNC/archive/v1.3.0.tar.gz | tar xz --strip 1 -C /opt/noVNC && \
    wget -qO- https://github.com/novnc/websockify/archive/v0.10.0.tar.gz | tar xz --strip 1 -C /opt/noVNC/utils/websockify && \
    ln -s /opt/noVNC/vnc.html /opt/noVNC/index.html

# Create ros user
RUN useradd -m -s /bin/bash ros && \
    usermod -aG sudo ros && \
    echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Setup ROS 2 environment for ros user
USER ros
WORKDIR /home/ros

# Create .bashrc with ROS 2 setup
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /opt/venv/bin/activate" >> ~/.bashrc && \
    echo "export WORKSPACE=/workspace" >> ~/.bashrc && \
    echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc && \
    echo "cd /workspace" >> ~/.bashrc

# Setup VNC
RUN mkdir -p ~/.vnc && \
    echo "#!/bin/bash" > ~/.vnc/xstartup && \
    echo "fluxbox &" >> ~/.vnc/xstartup && \
    chmod +x ~/.vnc/xstartup

# Set VNC password (default: 'vncpassword')
RUN echo "vncpassword" | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd

# Switch back to root for final setup
USER root

# Create workspace directory
RUN mkdir -p ${WORKSPACE} && chown -R ros:ros ${WORKSPACE}

# Setup rosdep
RUN rosdep init || true
USER ros
RUN rosdep update

# Copy entrypoint script
COPY .devcontainer/entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh

# Set working directory
WORKDIR ${WORKSPACE}

# Expose ports
EXPOSE 5901 6901 8080 9090

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
