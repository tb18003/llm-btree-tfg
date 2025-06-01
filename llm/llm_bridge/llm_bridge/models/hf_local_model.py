from .llm_model import LargeLanguageModel

# LLM

import gc


class HuggingFaceLocalModel(LargeLanguageModel):

    def __init__(self, name, params):
        super().__init__(name, params)
        from transformers import pipeline, AutoTokenizer
        from torch import bfloat16
        self._model = pipeline(
            "text-generation",
            model=name,
            model_kwargs={"torch_dtype": bfloat16},
            device_map="auto",
        )

    def generate(self, prompt):
        if self._model is None:
            raise Exception('ERROR: LLM Model is not loaded')

        inputs = [
            {
                "role": "system",
                "content": prompt['system']
            },
            {
                "role": "user",
                "content": prompt['user']
            }
        ]
        
        output = self._model(
            inputs, 
            max_new_tokens=self._params['max_new_tokens'],
            num_return_sequences=1,
            temperature=self._params['temperature'],
            top_k=self._params['top_k'],
            top_p=self._params['top_p'],
            disable_compile=True
        )

        return output[0]['generated_text'][-1]["content"]

    def unload(self):
        from torch.cuda import empty_cache
        del self._model
        gc.collect()
        empty_cache()
    
    def get_model_name(self):
        return self._name

if __name__ == "__main__":
    print("Loading model...")
    m = HuggingFaceLocalModel("google/gemma-3-27b-it", {
        'max_new_tokens': 50,
        'temperature': 0.7,
        'top_k': 50,
        'top_p': 0.95
    })

    p = input("Enter a prompt: ")

    print(m.generate({
        'system': 'You are a helpful assistant.',
        'user': p
    })[0]["generated_text"][-1])


