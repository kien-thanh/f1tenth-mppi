FROM ros:humble

RUN apt-get update --fix-missing && \
    apt-get install -y zsh \
                       git \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       gdbserver \
                       gdb \
                       ninja-build \
                       ccache \
                       ros-humble-foxglove-bridge \
                       ros-humble-urg-node-msgs \
                       ros-humble-slam-toolbox \
                       ros-humble-navigation2 
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d scipy

# Install oh-my-zsh and plugins
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" \
    && git clone --depth=1 https://github.com/zsh-users/zsh-syntax-highlighting.git "/root/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting" \
    && git clone --depth=1 https://github.com/zsh-users/zsh-autosuggestions.git "/root/.oh-my-zsh/custom/plugins/zsh-autosuggestions" \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "/root/.oh-my-zsh/custom/themes/powerlevel10k" \
    && sed -i 's/^plugins=.*/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' /root/.zshrc \
    && sed -i 's/^ZSH_THEME=.*/ZSH_THEME="powerlevel10k\/powerlevel10k"/' /root/.zshrc

# Setup custom prompt
COPY .p10k.zsh /root/.p10k.zsh
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> /root/.zshrc

# Source ROS2 and workspace setup files
RUN echo "source /opt/ros/humble/setup.zsh" >> /root/.zshrc
RUN echo "source /workspaces/f1Tenth/install/setup.zsh" >> /root/.zshrc

# Enable tab completion for ros2 and colcon commands
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Change default shell to zsh
RUN chsh -s $(which zsh)
