from .llm_model import LargeLanguageModel

# LLM

import gc


class HuggingFaceLocalModel(LargeLanguageModel):

    def __init__(self, id, params):
        from transformers import AutoTokenizer, AutoModelForCausalLM
        self._id = id
        self._model = AutoModelForCausalLM.from_pretrained(id, device_map="auto")
        self._tokenizer = AutoTokenizer.from_pretrained(id, device_map="auto")
        self._params = params

    def generate(self, prompt):
        from torch import ones_like
        if self._model is None:
            raise Exception('ERROR: LLM Model is not loaded')

        inputs = self._tokenizer.apply_chat_template([
            {
                "role": "system",
                "content": prompt['system']
            },
            {
                "role": "user",
                "content": prompt['user']
            }
        ], tokenize=True, add_generation_prompt=True, return_tensors="pt", return_attention_mask=True)
        inputs = inputs.to(self._model.device)
        attention_mask = torch.ones_like(inputs)
        
        output = self._model.generate(
            inputs, 
            max_new_tokens=self._params['max_new_tokens'],
            attention_mask=attention_mask,
            num_return_sequences=1,
            temperature=self._params['temperature'],
            top_k=self._params['top_k'],
            top_p=self._params['top_p'],
            do_sample=True,
            pad_token_id=self._tokenizer.eos_token_id # To avoid warning: The attention mask and the pad token id were not set. As a consequence, you may observe unexpected behavior. Please pass your input's `attention_mask` to obtain reliable results.
        )

        return self._tokenizer.decode(output[0][inputs.shape[-1]:], skip_special_tokens=True)

    def unload(self):
        from torch.cuda import empty_cache
        del self._model
        gc.collect()
        torch.cuda.empty_cache()
    
    def get_model_name(self):
        return self._id
        