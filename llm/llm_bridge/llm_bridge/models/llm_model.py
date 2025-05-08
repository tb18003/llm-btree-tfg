from abc import ABC, abstractmethod

class LargeLanguageModel(ABC):

    @abstractmethod
    def generate(self, prompt: dict[str,str]):
        pass

    @abstractmethod
    def get_model_name(self):
        pass

    @abstractmethod
    def unload(self):
        pass


