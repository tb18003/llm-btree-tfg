from .llm_model import LargeLanguageModel
from google import genai
import os

class GeminiModel(LargeLanguageModel):

    def __init__(self, name, params):
        super().__init__(name, params)
        self._client = genai.Client(api_key=os.environ['GEMINI_API_KEY'])

    def generate(self, prompt):
        response = self._client.models.generate_content(
            model=self._name,
            contents=[prompt['user']],
            config=genai.types.GenerateContentConfig(
                system_instruction=prompt['system'],
                max_output_tokens=self._params['max_new_tokens'],
                temperature=self._params['temperature'],
                top_k=self._params['top_k'],
                top_p=self._params['top_p'],
            )
        )

        return response.text

    def unload(self):
        pass

    def get_model_name(self):
        return self._name