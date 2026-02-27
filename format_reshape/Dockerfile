FROM ros:noetic-ros-base

# 安装必要的 Python 依赖
RUN apt-get update && apt-get install -y \
    python3-yaml \
    python3-numpy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 安装额外的 Python 包（如果需要）
RUN pip3 install pyyaml numpy

# 创建非 root 用户（与宿主机用户匹配）
RUN groupadd -g 1000 geely && \
    useradd -u 1000 -g geely -s /bin/bash -m geely && \
    mkdir -p /workspace && \
    chown -R geely:geely /workspace

# 设置工作目录
WORKDIR /workspace

# 复制脚本并设置权限
COPY --chown=geely:geely reshape.py /workspace/

# 切换到非 root 用户
USER geely

# 默认命令
CMD ["python3", "reshape.py"]