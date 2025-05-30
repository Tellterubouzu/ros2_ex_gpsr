#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import openai
import yaml
import os
from dotenv import load_dotenv

class LLMInterface:
    def __init__(self):
        # 環境変数の読み込み
        load_dotenv()
        self.api_key = os.getenv('OPENAI_API_KEY')
        openai.api_key = self.api_key

        # 設定ファイルの読み込み
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../config/llm_config.yaml'
        )
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.model = self.config['model']
        self.temperature = self.config['temperature']
        self.max_tokens = self.config['max_tokens']

    def get_next_state(self, userdata):
        """
        現在の状態とコンテキストに基づいて次の状態を決定
        """
        try:
            # プロンプトの作成
            prompt = self._create_prompt(userdata)
            
            # LLMへのリクエスト
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a task planner for a robot."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )

            # レスポンスの解析
            next_state = self._parse_response(response)
            return next_state

        except Exception as e:
            rospy.logerr(f"LLM Error: {str(e)}")
            return "Error"

    def _create_prompt(self, userdata):
        """
        現在の状態とコンテキストからプロンプトを生成
        """
        current_state = userdata.get('current_state', 'Unknown')
        context = userdata.get('context', {})
        
        prompt = f"""
        Current State: {current_state}
        Context: {context}
        
        Based on the current state and context, determine the next appropriate state.
        Available states: {userdata['state_list']}
        
        Return only the state name.
        """
        return prompt

    def _parse_response(self, response):
        """
        LLMのレスポンスを解析して次の状態を決定
        """
        try:
            next_state = response.choices[0].message.content.strip()
            return next_state
        except Exception as e:
            rospy.logerr(f"Response parsing error: {str(e)}")
            return "Error" 