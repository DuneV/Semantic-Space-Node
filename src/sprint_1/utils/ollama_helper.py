import requests
import base64
import os

class ImageAnalyzer:
    def __init__(self, ollama_url="http://localhost:11434", model='llava'):
        self.ollama_url = ollama_url
        self.model = model # modelo identificador de imagenes

    def _encode_image(self, image_path):
        """
        Codifica una imagen en formato base64.

        Args:
            image_path (str): La ruta al archivo de imagen.

        Returns:
            str: La imagen codificada en base64.
        """
        try:
            with open(image_path, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')
        except FileNotFoundError:
            print(f"Error: El archivo de imagen no se encontró en {image_path}")
            return None
        except Exception as e:
            print(f"Error al codificar la imagen: {e}")
            return None

    def analyze_image(self, image_path, prompt):
        """
        Analiza una imagen usando el modelo LLaVA de Ollama.

        Args:
            image_path (str): La ruta al archivo de imagen.
            prompt (str): La pregunta o instrucción para el modelo sobre la imagen.

        Returns:
            str: La respuesta del modelo LLaVA, o None si ocurre un error.
        """
        encoded_image = self._encode_image(image_path)
        if not encoded_image:
            return None

        url = f"{self.ollama_url}/api/generate"
        headers = {"Content-Type": "application/json"}

        data = {
            "model": self.model,
            "prompt": prompt,
            "images": [encoded_image],
            "stream": False 
        }

        try:
            response = requests.post(url, json=data, headers=headers)
            response.raise_for_status()  # Lanza una excepción para códigos de estado de error (4xx o 5xx)
            result = response.json()
            return result.get("response", "No se pudo obtener una respuesta.")
        except requests.exceptions.RequestException as e:
            print(f"Error al comunicarse con Ollama: {e}")
            return None
        except Exception as e:
            print(f"Ocurrió un error inesperado: {e}")
            return None