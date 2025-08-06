FROM ros:foxy

RUN apt-get update --fix-missing && \
    apt-get install -y zsh \
                       git \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" \
    && git clone https://github.com/zsh-users/zsh-syntax-highlighting.git "/root/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting" \
    && git clone https://github.com/zsh-users/zsh-autosuggestions.git "/root/.oh-my-zsh/custom/plugins/zsh-autosuggestions" \
    && git clone --depth=1 https://github.com/romkatv/powerlevel10k.git "/root/.oh-my-zsh/custom/themes/powerlevel10k" \
    && sed -i 's/^plugins=.*/plugins=(git zsh-syntax-highlighting zsh-autosuggestions)/' /root/.zshrc \
    && sed -i 's/^ZSH_THEME=.*/ZSH_THEME="powerlevel10k\/powerlevel10k"/' /root/.zshrc
    
COPY .p10k.zsh /root/.p10k.zsh
RUN echo "[[ ! -f ~/.p10k.zsh ]] || source ~/.p10k.zsh" >> /root/.zshrc

RUN echo "source /opt/ros/foxy/setup.zsh" >> /root/.zshrc

RUN chsh -s $(which zsh)
