from abc import ABC, abstractmethod

class LargeLanguageModel(ABC):

    def __init__(self, name, params):
        self._name = name
        self._params = params

    @abstractmethod
    def generate(self, prompt: dict[str,str]):
        pass

    @abstractmethod
    def get_model_name(self):
        pass

    @abstractmethod
    def unload(self):
        pass


