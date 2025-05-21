from .llm_model import LargeLanguageModel
import openai
import os

class ChatGPTModel(LargeLanguageModel):

    def __init__(self, name, params):
        self._client = openai.OpenAI(api_key=os.environ['GPT_API_KEY'])
        self._params = params
        self._name = name

    def generate(self, prompt):
        response = self._client.responses.create(
            model=self._name,
            input=[
                {
                    "role": "developer",
                    "content": prompt['system']
                },
                {
                    "role": "user",
                    "content": prompt['user']
                }
            ]
        )

        return response.output_text

    def unload(self):
        pass

    def get_model_name(self):
        return self._name