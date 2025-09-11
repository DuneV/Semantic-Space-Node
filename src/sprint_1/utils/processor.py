#!/home/krita/mujoco-env/bin/python3

import stanza
import queue
import json
import tempfile
import os
from collections import Counter

CATEGORIES = ['NOUN', 'VERB', 'ADP']

class PromptProcessor:
    def __init__(self, lang='en', json_file='last_analysis.json', max_queue_size=1000):
        self.nlp = stanza.Pipeline(lang, processors='tokenize,pos,lemma,ner')
        self.queue = queue.Queue(maxsize=max_queue_size)
        self.models = [
            {CATEGORIES[0]: 2, CATEGORIES[1]: 1, CATEGORIES[2]: 1},
            {CATEGORIES[0]: 1, CATEGORIES[1]: 1, CATEGORIES[2]: 0},
            {CATEGORIES[0]: 3, CATEGORIES[1]: 2, CATEGORIES[2]: 2}
        ]
        self.categories = CATEGORIES
        self.json_file = json_file
        print("Pipeline y cola listos.")
    
    def __del__(self):
        self.nlp = None
        while not self.queue.empty():
            self.queue.get()
        if os.path.exists(self.json_file):
            os.remove(self.json_file)

    def thinking_chain(self, pos_counts):
        for i, model in enumerate(self.models):
            if all(pos_counts.get(cat, 0) == model.get(cat, 0) for cat in self.categories):
                return i
        return None

    def process(self, text):
        print(f"\nTexto de entrada: \"{text}\"")
        print("Procesando con Stanza...\n")

        doc = self.nlp(text)

        while not self.queue.empty():
            self.queue.get()

        pos_counts = Counter()
        noun_list = []
        verb_list = []
        adp_list = []

        for sentence in doc.sentences:
            for word in sentence.words:
                lemma = word.lemma or word.text
                pos = word.pos
                if pos in self.categories:
                    self.queue.put((lemma, pos))
                    pos_counts[pos] += 1
                    if pos == 'NOUN':
                        noun_list.append(lemma)
                    elif pos == 'VERB':
                        verb_list.append(lemma)
                    elif pos == 'ADP':
                        adp_list.append(lemma)

        print("Contenido de la cola (lemma, pos):")
        print("-" * 60)
        temp_queue = queue.Queue()
        while not self.queue.empty():
            lemma, pos = self.queue.get()
            print(f"{lemma:>12} | {pos:>6}")
            temp_queue.put((lemma, pos))
        while not temp_queue.empty():
            self.queue.put(temp_queue.get())

        print("\nConteo por categorías:")
        print("-" * 40)
        for category in self.categories:
            count = pos_counts.get(category, 0)
            print(f"   {category}: {count}")

        model_index = self.thinking_chain(pos_counts)
        if model_index is not None:
            print(f"\nCoincidencia con el modelo {model_index}: {self.models[model_index]}")
        else:
            print("\nNo hay coincidencia con ningún modelo.")

        print("\nTokens, lemas, POS y NER:")
        print("-" * 60)
        for sentence in doc.sentences:
            for word in sentence.words:
                ner_tag = getattr(word, 'ner', 'O')
                print(f"{word.text:>12} | {word.lemma:>12} | {word.pos:>6} | {ner_tag}")

        print("\nEntidades nombradas detectadas:")
        print("-" * 40)
        found_entities = False
        for sentence in doc.sentences:
            for ent in sentence.ents:
                print(f"   {ent.text} → {ent.type}")
                found_entities = True
        if not found_entities:
            print("   (Ninguna entidad detectada)")

        self.save_to_json(pos_counts, model_index, noun_list, verb_list, adp_list)

    def save_to_json(self, pos_counts, model_index, noun_list, verb_list, adp_list):
        json_data = {
            "queue_content": [],
            "category_counts": {cat: pos_counts.get(cat, 0) for cat in self.categories},
            "model_match": model_index if model_index is not None else "None",
            "nouns": noun_list,
            "verbs": verb_list,
            "adps": adp_list
        }

        temp_queue = queue.Queue()
        while not self.queue.empty():
            lemma, pos = self.queue.get()
            json_data["queue_content"].append({"lemma": lemma, "pos": pos})
            temp_queue.put((lemma, pos))
        while not temp_queue.empty():
            self.queue.put(temp_queue.get())

        try:
            # archivo temporal en el mismo directorio
            dir_name = os.path.dirname(self.json_file)
            with tempfile.NamedTemporaryFile('w', delete=False, dir=dir_name, encoding='utf-8') as tmp:
                json.dump(json_data, tmp, ensure_ascii=False, indent=4)
                temp_name = tmp.name
            os.replace(temp_name, self.json_file)  # reemplazo atómico
            print(f"\nEstado guardado en '{self.json_file}'")
        except Exception as e:
            print(f"Error al guardar JSON: {e}")

""" def main():
    processor = PromptProcessor(lang='es', json_file='last_analysis.json')
    while True:
        try:
            text = input("Ingresa un texto: ")
            if text.strip():
                processor.process(text)
            else:
                print(" Por favor, ingresa un texto no vacío.")
        except KeyboardInterrupt:
            print("\nSaliendo del programa...")
            break

if __name__ == '__main__':
    main() """