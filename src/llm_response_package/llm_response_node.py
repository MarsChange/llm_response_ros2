#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
import threading
import time
import sys
import os
import tkinter as tk
from tkinter import ttk, scrolledtext
sys.path.append(f'/home/marc/.conda/envs/python310/lib/python3.10/site-packages')
from gradio_client import Client, handle_file
from bs4 import BeautifulSoup

class LLMResponseNode(Node):
    """
    ROS2节点，用于从GUI获取输入，调用大语言模型API，并发布响应
    """
    
    def __init__(self):
        super().__init__('llm_response_node')
        
        # 创建publisher，发布到/target话题
        self.publisher = self.create_publisher(String, '/target', 10)
        
        # 声明ROS2参数，从配置文件读取
        self.declare_parameter('api_url', 'http://localhost:8000/v1/chat/completions')
        self.declare_parameter('api_key', '')
        self.declare_parameter('model_name', 'gpt-3.5-turbo')
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('enable_thinking', False)
        self.declare_parameter('timeout', 30)
        self.declare_parameter('gradio_ip', 'http://45.76.185.223:5709/')
        self.declare_parameter('gradio_display_image_path', '')
        self.declare_parameter('gradio_api_name', '/process_image')
        self.declare_parameter('system_prompt','')
        self.declare_parameter('use_gradio', False)
        
        # 获取参数值
        self.timeout = self.get_parameter('timeout').value
        self.api_url = self.get_parameter('api_url').value
        self.api_key = self.get_parameter('api_key').value
        self.model_name = self.get_parameter('model_name').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value
        self.enable_thinking = self.get_parameter('enable_thinking').value
        self.gradio_ip = self.get_parameter('gradio_ip').value
        self.gradio_display_image_path = self.get_parameter('gradio_display_image_path').value
        self.gradio_api_name = self.get_parameter('gradio_api_name').value
        self.use_gradio = self.get_parameter('use_gradio').value
        self.system_prompt = self.get_parameter('system_prompt').value

        self.get_logger().info(f'LLM Response Node 已启动')
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'模型: {self.model_name}')
        self.get_logger().info(f'最大令牌数: {self.max_tokens}')
        self.get_logger().info(f'温度: {self.temperature}')
        self.get_logger().info(f'思考模式: {self.enable_thinking}')
        self.get_logger().info(f'超时: {self.timeout}秒')
        
        # 初始化GUI
        self.setup_gui()
        
        # 创建定时器处理GUI事件
        self.timer = self.create_timer(0.1, self._process_gui_events)
        
        self.running = True

    def _setup_chinese_font(self):
        """设置中文字体支持"""
        # 尝试设置中文字体，按优先级排序
        chinese_fonts = [
            'PingFang SC',      # macOS 苹方字体
            'Hiragino Sans GB', # macOS 冬青黑体
            'Microsoft YaHei',  # Windows 微软雅黑
            'SimHei',          # Windows 黑体
            'WenQuanYi Micro Hei', # Linux 文泉驿微米黑
            'DejaVu Sans',     # 通用字体
            'Arial'            # 备用字体
        ]
        
        self.chinese_font = 'Arial'  # 默认字体
        
        for font in chinese_fonts:
            try:
                # 测试字体是否可用
                test_label = tk.Label(self.root, text="测试", font=(font, 10))
                test_label.destroy()
                self.chinese_font = font
                self.get_logger().info(f'使用中文字体: {font}')
                break
            except:
                continue
        
        self.get_logger().info(f'最终使用字体: {self.chinese_font}')
    
    def setup_gui(self):
        """设置美化后的GUI界面"""
        self.root = tk.Tk()
        self.root.title("LLM Response Node - ROS2")
        self.root.geometry("900x700")
        self.root.minsize(700, 500)
        self.root.configure(bg="#f5f6fa")

        # 配置中文字体支持
        self._setup_chinese_font()

        # ttk主题美化
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TLabelFrame", background="#f5f6fa", font=(self.chinese_font, 12, "bold"))
        style.configure("TButton", font=(self.chinese_font, 12), padding=8)
        style.configure("TLabel", background="#f5f6fa", font=(self.chinese_font, 11))

        # 主框架
        main_frame = ttk.Frame(self.root, padding=20)
        main_frame.pack(fill="both", expand=True)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(2, weight=1)

        # 标题
        title_label = ttk.Label(main_frame, text="LLM Response Node", font=(self.chinese_font, 20, "bold"))
        title_label.grid(row=0, column=0, pady=(0, 18), sticky="n")

        # 输入区分组
        input_labelframe = ttk.LabelFrame(main_frame, text="输入区", padding=(18, 12))
        input_labelframe.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        input_labelframe.columnconfigure(0, weight=1)
        input_labelframe.columnconfigure(1, minsize=100)

        # 输入文本框
        self.input_text = tk.Text(input_labelframe, height=3, font=(self.chinese_font, 13), wrap="word")
        self.input_text.grid(row=0, column=0, sticky="ew", padx=(0, 12), pady=(0, 6))

        # 发送按钮
        self.send_button = ttk.Button(input_labelframe, text="发送", command=self._send_input)
        self.send_button.grid(row=0, column=1, sticky="e", ipadx=12, ipady=6)

        # 输入提示
        input_hint = ttk.Label(input_labelframe, text="支持多行中文输入，Ctrl+Enter发送", font=(self.chinese_font, 10))
        input_hint.grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))

        # 日志区分组
        log_labelframe = ttk.LabelFrame(main_frame, text="日志信息", padding=(18, 12))
        log_labelframe.grid(row=2, column=0, sticky="nsew")
        log_labelframe.rowconfigure(0, weight=1)
        log_labelframe.columnconfigure(0, weight=1)

        self.log_text = scrolledtext.ScrolledText(log_labelframe, height=18, font=(self.chinese_font, 11), bg="#f8f8ff", state="normal", wrap="word")
        self.log_text.grid(row=0, column=0, sticky="nsew")

        # 状态栏
        self.status_var = tk.StringVar()
        self.status_var.set("就绪")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, anchor="w", font=(self.chinese_font, 10), relief=tk.SUNKEN, padding=(10, 4))
        status_bar.pack(fill="x", side="bottom")

        # 绑定回车键（Ctrl+Enter发送）
        self.input_text.bind('<Control-Return>', lambda event: self._send_input())

        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

        # 添加初始日志
        self._add_log("LLM Response Node 已启动")
        self._add_log(f"API URL: {self.api_url}")
        self._add_log(f"模型: {self.model_name}")
        self._add_log(f"思考模式: {self.enable_thinking}")
        self._add_log("请在输入框中输入文本，然后点击发送或按Ctrl+Enter键")
        self._add_log("支持中文输入，请确保系统已安装中文字体")
    
    def _add_log(self, message):
        """添加日志信息到GUI"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        self.root.update_idletasks()
    
    def _send_input(self):
        """发送输入"""
        user_input = self.input_text.get("1.0", tk.END).strip()
        if not user_input:
            return
        
        # 清空输入框
        self.input_text.delete("1.0", tk.END)
        
        # 处理输入
        self._process_input(user_input)
    
    def _process_gui_events(self):
        """处理GUI事件"""
        try:
            self.root.update()
        except tk.TclError:
            # GUI已关闭
            self.running = False
            rclpy.shutdown()
    
    def _process_input(self, user_input):
        """处理用户输入"""
        if user_input.lower() == 'quit':
            self._add_log("正在退出...")
            self.running = False
            self.root.quit()
            rclpy.shutdown()
            return
        
        self._add_log(f"收到输入: {user_input}")
        self.status_var.set("正在处理...")
        try:
            # 调用大语言模型API
            response = self._call_llm_api(user_input)
            
            # 发布响应
            msg = String()
            msg.data = response
            self.publisher.publish(msg)
            
            self._add_log(f"已发布响应: {response}")
            self.status_var.set("处理完成")
            
        except Exception as e:
            error_msg = f'处理输入时出错: {str(e)}'
            self._add_log(f"错误: {error_msg}")
            self.status_var.set("处理失败")
            
            # 发布错误信息
            msg = String()
            msg.data = f"错误: {error_msg}"
            self.publisher.publish(msg)
    
    def _call_llm_api(self, prompt):
        prompt = self.system_prompt + prompt
        """调用大语言模型API"""
        try:
            if self.use_gradio:
                client = Client(f'{self.gradio_ip}')
                self._add_log(f"正在调用Gradio API: {self.gradio_ip}")
                result = client.predict(
                        message=f'{prompt}',
                        chat_history=[],
                        display_image=f'{self.gradio_display_image_path}',
                        api_name=f'{self.gradio_api_name}',
                )

                if result != '':
                    result = [item['content'] for item in result[0]]
                    soup = BeautifulSoup(result[0], 'html.parser')  # 解析HTML
                    result = soup.get_text().strip()
                    result = result.replace("[Thought]:", "").strip()
                    return result
                else:
                    raise Exception(f"API无消息反馈。")
            else:
                self._add_log(f"正在调用API: {self.api_url}")
                # 构建请求头
                headers = {
                    'Content-Type': 'application/json',
                    'Authorization': f'Bearer {self.api_key}'
                }
                
                # 构建请求数据
                data = {
                    'model': self.model_name,
                    'messages': [
                        {'role': 'user', 'content': prompt}
                    ],
                    'enable_thinking': self.enable_thinking,
                    'max_tokens': self.max_tokens,
                    'temperature': self.temperature
                }
                
                # 发送请求
                response = requests.post(
                    f"{self.api_url}",
                    headers=headers,
                    json=data,
                    timeout=self.timeout
                )
                
                self._add_log(f"API响应状态码: {response.status_code}")
                
                if response.status_code == 200:
                    result = response.json()
                    response_content = result['choices'][0]['message']['content']
                    self._add_log(f"API调用成功，响应长度: {len(response_content)}")
                    return response_content
                else:
                    error_detail = response.text if response.text else "无详细信息"
                    raise Exception(f'API请求失败，状态码: {response.status_code}, 详情: {error_detail}')
                    
        except Exception as e:
            raise Exception(f'API调用错误: {str(e)}')
    
    def _on_closing(self):
        """处理窗口关闭事件"""
        self._add_log("正在退出...")
        self.running = False
        self.root.quit()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    node = LLMResponseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到中断信号，正在退出...')
        node.running = False
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 