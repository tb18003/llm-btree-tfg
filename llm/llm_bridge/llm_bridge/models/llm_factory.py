from .gemini_model import GeminiModel
from .hf_local_model import HuggingFaceLocalModel
from .gpt_model import ChatGPTModel
from .llm_model import LargeLanguageModel

class LargeLanguageModelFactory():

    __models_db: dict[str, LargeLanguageModel] = {
        'meta-llama/Llama-3.3-70B-Instruct': HuggingFaceLocalModel,
        'deepseek-ai/deepseek-llm-67b-chat': HuggingFaceLocalModel,
        'meta-llama/Llama-3.1-8B-Instruct': HuggingFaceLocalModel,
        'google/gemma-3-27b-it': HuggingFaceLocalModel,
        'deepseek-ai/deepseek-llm-7b-chat': HuggingFaceLocalModel,
        'gpt-4o-mini': ChatGPTModel,
        'o4-mini': ChatGPTModel,
        'gemini-2.0-flash': GeminiModel,
        'gemini-1.5-flash': GeminiModel
    }

    @classmethod
    def get_instance(cls, name: str, params: dict[str,str]):
        if name not in cls.__models_db.keys():
            return HuggingFaceLocalModel("meta-llama/Llama-3.1-8B-Instruct", params)

        _cls = cls.__models_db.get(name)

        return _cls(name, params)