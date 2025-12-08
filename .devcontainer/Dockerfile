FROM nvidia/cuda:12.9.0-devel-ubuntu22.04
# FROM nvcr.io/nvidia/12.6.11-devel:12.6.11-devel-aarch64-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update --fix-missing && apt-get dist-upgrade -y

RUN apt-get install software-properties-common curl -y && \
    add-apt-repository universe -y && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y ros-humble-desktop \
                        ros-dev-tools \
                        ros-humble-foxglove-bridge \
                        ros-humble-urg-node-msgs \
                        ros-humble-slam-toolbox \
                        ros-humble-navigation2 

RUN apt-get install -y zsh \
                       git \
                       git-lfs \
                       vim \
                       tmux \
                       python3-pip \
                       libeigen3-dev \
                       build-essential \
                       g++ \
                       gcc \
                       gdbserver \
                       gdb \
                       ninja-build \
                       ccache

RUN pip3 install transforms3d scipy plotly pandas "numpy<2"

RUN git lfs install

# ============================================================================
# Custom shell setup
# ============================================================================
COPY .p10k.zsh /root/.p10k.zsh

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" && \
    git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git "/root/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting" && \
    git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git "/root/.oh-my-zsh/custom/plugins/zsh-autosuggestions" && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "/root/.oh-my-zsh/custom/themes/powerlevel10k" && \
    sed -i 's/^plugins=.*/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' /root/.zshrc && \
    sed -i 's/^ZSH_THEME=.*/ZSH_THEME="powerlevel10k\/powerlevel10k"/' /root/.zshrc && \
    echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> /root/.zshrc && \
    echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc && \
    echo "source /workspaces/f1Tenth/install/setup.zsh" >> /root/.zshrc && \
    echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc && \
    echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Change default shell to zsh
RUN chsh -s $(which zsh)
