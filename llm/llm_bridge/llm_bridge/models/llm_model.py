from abc import ABC, abstractmethod

class LargeLanguageModel(ABC):
    """
    Abstract base class for large language models.

    This class defines the interface that all large language model implementations must follow.
    Subclasses must implement the following methods:
    - generate(prompt): Generate a response given a prompt.
    - get_model_name(): Return the name of the model.
    - unload(): Release any resources held by the model.
    """

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


