# LLM Response ROS2 Package

这是一个ROS2包，基于tkinter设计了一个与大模型对话的gui界面，通过api获取大模型的response，并对response进行处理（最好自己改prompt要求大模型输出指定格式），获取response后处理为ros2节点消息并发送出去，该项目仅仅是初步框架，可根据需要进行修改。

## 功能特性

- 从终端实时获取用户输入的文本信息
- 将输入文本转换为prompt发送给大语言模型API
- 获取大语言模型的响应并发布到`/target`话题
- 支持多种大语言模型API（OpenAI、本地部署等）
- 线程安全的输入处理
- 错误处理和日志记录

## 安装依赖

```bash
pip install -r requirements.txt
```

## 配置环境变量

在运行节点之前，请设置以下环境变量（可选）：

```bash
# API配置
export LLM_API_URL="http://localhost:8000/v1/chat/completions"  # API端点
export LLM_API_KEY="your-api-key"                              # API密钥（如果需要）
export LLM_MODEL_NAME="gpt-3.5-turbo"                          # 模型名称
```

## 使用方法

### 1. 构建包

```bash
cd llm_response_package
colcon build
source install/setup.bash
```

### 2. 运行节点

#### 方法1: 直接运行
```bash
ros2 run llm_response_package llm_response_node
```

#### 方法2: 使用launch文件
```bash
ros2 launch llm_response_package llm_response_launch.py
```

### 3. 使用节点

启动节点后，您可以在终端中输入文本信息。节点会：

1. 接收您的输入
2. 将输入发送给配置的大语言模型API
3. 获取响应并发布到`/target`话题
4. 在终端显示处理结果

输入`quit`可以退出节点。

### 4. 监听话题

在另一个终端中，您可以监听`/target`话题来接收响应：

```bash
ros2 topic echo /target
```

## 支持的API格式

该节点支持标准的OpenAI兼容API格式，包括：

- OpenAI API
- 本地部署的OpenAI兼容服务（如Ollama、LM Studio等）
- 其他兼容OpenAI API格式的服务

## 错误处理

节点包含完善的错误处理机制：

- 网络连接错误
- API响应格式错误
- 输入验证
- 超时处理

所有错误信息都会发布到`/target`话题，并在终端日志中显示。

## 自定义配置

您可以通过修改环境变量来自定义节点行为：

- `LLM_API_URL`: 设置API端点
- `LLM_API_KEY`: 设置API密钥
- `LLM_MODEL_NAME`: 设置使用的模型名称

## 许可证

Apache License 2.0 