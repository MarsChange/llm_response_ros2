#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestLLMResponseNode:
    """测试LLM响应节点的基本功能"""
    
    @pytest.fixture(autouse=True)
    def setup(self):
        """设置测试环境"""
        rclpy.init()
        yield
        rclpy.shutdown()
    
    def test_node_creation(self):
        """测试节点创建"""
        from llm_response_package.llm_response_node import LLMResponseNode
        
        node = LLMResponseNode()
        assert node is not None
        assert node.get_name() == 'llm_response_node'
        
        # 检查publisher是否创建
        publishers = node.get_publishers()
        assert len(publishers) > 0
        
        node.destroy_node()
    
    def test_publisher_topic(self):
        """测试publisher话题名称"""
        from llm_response_package.llm_response_node import LLMResponseNode
        
        node = LLMResponseNode()
        
        # 检查publisher是否发布到正确的話題
        publishers = node.get_publishers()
        topic_names = [pub.topic_name for pub in publishers]
        assert '/target' in topic_names
        
        node.destroy_node()

if __name__ == '__main__':
    pytest.main([__file__]) 